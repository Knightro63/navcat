import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';
import './index.dart';

// Maximum number of iterations for contour walking to prevent infinite loops
const int maxContourWalkIterations = 40000;

class Contour {
  /// Simplified contour vertex and connection data. size: 4 * nVerts
  List<double> vertices;
  /// The number of vertices in the simplified contour
  int nVertices;
  /// Raw contour vertex and connection data
  List<double> rawVertices;
  /// The number of vertices in the raw contour
  int nRawVertices;
  /// The region id of the contour
  int reg;
  /// The area id of the contour
  int area;

  Contour({
    required this.vertices,
    required this.nVertices,
    required this.rawVertices,
    required this.nRawVertices,
    required this.reg,
    required this.area,
  });
}

class ContourSet {
  /// An array of the contours in the set
  List<Contour> contours;
  /// The bounds in world space
  BoundingBox bounds;
  /// The size of each cell
  double cellSize;
  /// The height of each cell
  double cellHeight;
  /// The width of the set
  double width;
  /// The height of the set
  double height;
  /// The aabb border size used to generate the source data that the contour set was derived from
  double borderSize;
  /// The max edge error that this contour set was simplified with
  double maxError;

  ContourSet({
    required this.contours,
    required this.bounds,
    required this.cellSize,
    required this.cellHeight,
    required this.width,
    required this.height,
    required this.borderSize,
    required this.maxError,
  });
}

class ContourBuildFlags {
  /// Tessellate solid (impassable) edges during contour simplification
  static const int contourTessWallEdges = 0x01;
  /// Tessellate edges between areas during contour simplification
  static const int contourTessAreaEdges = 0x02;
}

// Helper function to get corner height
int getCornerHeight(
  int x,
  int y,
  int i,
  int dir,
  CompactHeightfield chf,
  BooleanRef isBorderVertex,
) {
  final s = chf.spans[i];
  int ch = s.y;
  final dirp = (dir + 1) & 0x3;
  final regs = List<int>.filled(4, 0);

  // Combine region and area codes in order to prevent
  // border vertices which are in between two areas to be removed.
  regs[0] = chf.spans[i].region | (chf.areas[i] << 16);

  if (getCon(s, dir) != notConnected) {
    final ax = x + getDirOffsetX(dir);
    final ay = y + getDirOffsetY(dir);
    final ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
    final as = chf.spans[ai];
    ch = math.max(ch, as.y);
    regs[1] = chf.spans[ai].region | (chf.areas[ai] << 16);

    if (getCon(as, dirp) != notConnected) {
      final ax2 = ax + getDirOffsetX(dirp);
      final ay2 = ay + getDirOffsetY(dirp);
      final ai2 = chf.cells[ax2 + ay2 * chf.width].index + getCon(as, dirp);
      final as2 = chf.spans[ai2];
      ch = math.max(ch, as2.y);
      regs[2] = chf.spans[ai2].region | (chf.areas[ai2] << 16);
    }
  }

  if (getCon(s, dirp) != notConnected) {
    final ax = x + getDirOffsetX(dirp);
    final ay = y + getDirOffsetY(dirp);
    final ai = chf.cells[ax + ay * chf.width].index + getCon(s, dirp);
    final as = chf.spans[ai];
    ch = math.max(ch, as.y);
    regs[3] = chf.spans[ai].region | (chf.areas[ai] << 16);

    if (getCon(as, dir) != notConnected) {
      final ax2 = ax + getDirOffsetX(dir);
      final ay2 = ay + getDirOffsetY(dir);
      final ai2 = chf.cells[ax2 + ay2 * chf.width].index + getCon(as, dir);
      final as2 = chf.spans[ai2];
      ch = math.max(ch, as2.y);
      regs[2] = chf.spans[ai2].region | (chf.areas[ai2] << 16);
    }
  }

  // Check if the vertex is special edge vertex, these vertices will be removed later.
  for (int j = 0; j < 4; ++j) {
    final a = j;
    final b = (j + 1) & 0x3;
    final c = (j + 2) & 0x3;
    final d = (j + 3) & 0x3;

    // The vertex is a border vertex there are two same exterior cells in a row,
    // followed by two interior cells and none of the regions are out of bounds.
    final twoSameExts = (regs[a] & regs[b] & borderReg) != 0 && regs[a] == regs[b];
    final twoInts = ((regs[c] | regs[d]) & borderReg) == 0;
    final intsSameArea = (regs[c] >> 16) == (regs[d] >> 16);
    final noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;

    if (twoSameExts && twoInts && intsSameArea && noZeros) {
      isBorderVertex.value = true;
      break;
    }
  }

  return ch;
}

// Helper function to walk contour
void _walkContour(
  int x,
  int y,
  int i,
  CompactHeightfield chf,
  List<int> flags,
  List<int> points,
) {
  // Choose the first non-connected edge
  int dir = 0;
  while ((flags[i] & (1 << dir)) == 0) {
    dir++;
  }
  
  final startDir = dir;
  final starti = i;
  final area = chf.areas[i];
  int iter = 0;
  int currentX = x;
  int currentY = y;
  int currentI = i;

  while (++iter < maxContourWalkIterations) {
    if ((flags[currentI] & (1 << dir)) != 0) {
      // Choose the edge corner
      final isBorderVertex = BooleanRef(false);
      bool isAreaBorder = false;
      int px = currentX;
      final py = getCornerHeight(currentX, currentY, currentI, dir, chf, isBorderVertex);
      int pz = currentY;

      switch (dir) {
        case 0:
          pz++;
          break;
        case 1:
          px++;
          pz++;
          break;
        case 2:
          px++;
          break;
      }

      int r = 0;
      final s = chf.spans[currentI];
      if (getCon(s, dir) != notConnected) {
        final ax = currentX + getDirOffsetX(dir);
        final ay = currentY + getDirOffsetY(dir);
        final ai = chf.cells[ax + ay * chf.width].index + getCon(s, dir);
        r = chf.spans[ai].region;
        if (area != chf.areas[ai]) {
          isAreaBorder = true;
        }
      }

      if (isBorderVertex.value) {
        r |= borderVertex;
      }
      if (isAreaBorder) {
        r |= areaBorder;
      }

      points.add(px);
      points.add(py);
      points.add(pz);
      points.add(r);

      flags[currentI] &= ~(1 << dir); // Remove visited edges
      dir = (dir + 1) & 0x3; // Rotate CW
    } else {
      int ni = -1;
      final nx = currentX + getDirOffsetX(dir);
      final ny = currentY + getDirOffsetY(dir);
      final s = chf.spans[currentI];

      if (getCon(s, dir) != notConnected) {
        final nc = chf.cells[nx + ny * chf.width];
        ni = nc.index + getCon(s, dir);
      }

      if (ni == -1) {
        // Should not happen.
        return;
      }

      currentX = nx;
      currentY = ny;
      currentI = ni;
      dir = (dir + 3) & 0x3; // Rotate CCW
    }

    if (starti == currentI && startDir == dir) {
      break;
    }
  }
}

double distancePtSegFromVectors(Vector3 a, Vector3 p, Vector3 q) {
  return distancePtSeg(a.x,a.z,p.x,p.z,q.x,q.z);
}

/// Helper function to calculate distance from point to line segment
double distancePtSeg(
  double x,
  double z,
  double px,
  double pz,
  double qx,
  double qz,
) {
  final pqx = qx - px;
  final pqz = qz - pz;
  final dx = x - px;
  final dz = z - pz;
  final d = pqx * pqx + pqz * pqz;
  double t = pqx * dx + pqz * dz;
  
  if (d > 0) {
    t /= d;
  }
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }
  
  final finalDx = px + t * pqx - x;
  final finalDz = pz + t * pqz - z;
  return finalDx * finalDx + finalDz * finalDz;
}

/// Helper function to simplify contour
void simplifyContour(
  List<int> points,
  List<int> simplified,
  double maxError,
  double maxEdgeLen,
  int buildFlags,
) {
  // Add initial points.
  bool hasConnections = false;
  for (int i = 0; i < points.length; i += 4) {
    if ((points[i + 3] & contourRegMask) != 0) {
      hasConnections = true;
      break;
    }
  }

  if (hasConnections) {
    // The contour has some portals to other regions.
    // Add a new point to every location where the region changes.
    final ni = (points.length / 4).floor();
    for (int i = 0; i < ni; ++i) {
      final ii = (i + 1) % ni;
      final differentRegs = (points[i * 4 + 3] & contourRegMask) != 
                            (points[ii * 4 + 3] & contourRegMask);
      final areaBorders = (points[i * 4 + 3] & areaBorder) != 
                          (points[ii * 4 + 3] & areaBorder);
      
      if (differentRegs || areaBorders) {
        simplified.add(points[i * 4 + 0]);
        simplified.add(points[i * 4 + 1]);
        simplified.add(points[i * 4 + 2]);
        simplified.add(i);
      }
    }
  }

  if (simplified.isEmpty) {
    // If there is no connections at all,
    // create some initial points for the simplification process.
    // Find lower-left and upper-right vertices of the contour.
    int llx = points[0];
    int lly = points[1];
    int llz = points[2];
    int lli = 0;
    int urx = points[0];
    int ury = points[1];
    int urz = points[2];
    int uri = 0;

    for (int i = 0; i < points.length; i += 4) {
      final x = points[i + 0];
      final y = points[i + 1];
      final z = points[i + 2];
      
      if (x < llx || (x == llx && z < llz)) {
        llx = x;
        lly = y;
        llz = z;
        lli = (i / 4).floor();
      }
      if (x > urx || (x == urx && z > urz)) {
        urx = x;
        ury = y;
        urz = z;
        uri = (i / 4).floor();
      }
    }
    
    simplified.addAll([llx, lly, llz, lli, urx, ury, urz, uri]);
  }

  // Add points until all raw points are within error tolerance to the simplified shape.
  final pn = (points.length / 4).floor();
  for (int i = 0; i < (simplified.length / 4).floor();) {
    final ii = (i + 1) % (simplified.length / 4).floor();
    final ax = simplified[i * 4 + 0];
    final az = simplified[i * 4 + 2];
    final ai = simplified[i * 4 + 3];
    
    final bx = simplified[ii * 4 + 0];
    final bz = simplified[ii * 4 + 2];
    final bi = simplified[ii * 4 + 3];

    // Find maximum deviation from the segment.
    double maxd = 0.0;
    int maxi = -1;
    int ci;
    int cinc;
    int endi;

    // Traverse the segment in lexilogical order so that the max deviation
    // is calculated similarly when traversing opposite segments.
    int segAx = ax;
    int segAz = az;
    int segBx = bx;
    int segBz = bz;

    if (bx > ax || (bx == ax && bz > az)) {
      cinc = 1;
      ci = (ai + cinc) % pn;
      endi = bi;
    } else {
      cinc = pn - 1;
      ci = (bi + cinc) % pn;
      endi = ai;
      // Swap ax, bx and az, bz
      segAx = bx;
      segBx = ax;
      segAz = bz;
      segBz = az;
    }

    // Tessellate only outer edges or edges between areas.
    if ((points[ci * 4 + 3] & contourRegMask) == 0 || (points[ci * 4 + 3] & areaBorder) != 0) {
      while (ci != endi) {
        final d = distancePtSeg(
          points[ci * 4 + 0].toDouble(),
          points[ci * 4 + 2].toDouble(),
          segAx.toDouble(),
          segAz.toDouble(),
          segBx.toDouble(),
          segBz.toDouble(),
        );
        if (d > maxd) {
          maxd = d;
          maxi = ci;
        }
        ci = (ci + cinc) % pn;
      }
    }

    // If the max deviation is larger than accepted error, add a new point
    if (maxi != -1 && maxd > maxError * maxError) {
      // Direct element insertion to safely shift the array data downstream
      final insertIndex = (i + 1) * 4;
      simplified.insertAll(insertIndex, [
        points[maxi * 4 + 0],
        points[maxi * 4 + 1],
        points[maxi * 4 + 2],
        maxi,
      ]);
    } else {
      ++i;
    }
  }

  // Split too long edges.
  if (maxEdgeLen > 0 &&
      (buildFlags & (ContourBuildFlags.contourTessWallEdges | ContourBuildFlags.contourTessAreaEdges)) != 0) {
    for (int i = 0; i < (simplified.length / 4).floor();) {
      final ii = (i + 1) % (simplified.length / 4).floor();
      final ax = simplified[i * 4 + 0];
      final az = simplified[i * 4 + 2];
      final ai = simplified[i * 4 + 3];

      final bx = simplified[ii * 4 + 0];
      final bz = simplified[ii * 4 + 2];
      final bi = simplified[ii * 4 + 3];

      int maxi = -1;
      final ci = (ai + 1) % pn;

      // Tessellate only outer edges or edges between areas.
      bool tess = false;
      // Wall edges.
      if ((buildFlags & ContourBuildFlags.contourTessWallEdges) != 0 && (points[ci * 4 + 3] & contourRegMask) == 0) {
        tess = true;
      }
      // Edges between areas.
      if ((buildFlags & ContourBuildFlags.contourTessAreaEdges) != 0 && (points[ci * 4 + 3] & areaBorder) != 0) {
        tess = true;
      }

      if (tess) {
        final dx = bx - ax;
        final dz = bz - az;
        if (dx * dx + dz * dz > maxEdgeLen * maxEdgeLen) {
          // Round based on the segments in lexilogical order
          final n = bi < ai ? bi + pn - ai : bi - ai;
          if (n > 1) {
            if (bx > ax || (bx == ax && bz > az)) {
              maxi = (ai + (n / 2).floor()) % pn;
            } else {
              maxi = (ai + ((n + 1) / 2).floor()) % pn;
            }
          }
        }
      }

      // If an edge is too long, add a point to split it.
      if (maxi != -1) {
        final insertIndex = (i + 1) * 4;
        simplified.insertAll(insertIndex, [
          points[maxi * 4 + 0],
          points[maxi * 4 + 1],
          points[maxi * 4 + 2],
          maxi,
        ]);
      } else {
        ++i;
      }
    }
  }

  for (int i = 0; i < (simplified.length / 4).floor(); ++i) {
    // The edge vertex flag is taken from the current raw point,
    // and the neighbour region is taken from the next raw point.
    final ai = (simplified[i * 4 + 3] + 1) % pn;
    final bi = simplified[i * 4 + 3];
    simplified[i * 4 + 3] = (points[ai * 4 + 3] & (contourRegMask | areaBorder)) | 
                            (points[bi * 4 + 3] & borderVertex);
  }
}

// Global scratch variables to minimize garbage collection cycles
final Vector2 _intersectSegContour_p0 = Vector2.zero();
final Vector2 _intersectSegContour_p1 = Vector2.zero();
final Vector2 _inCone_pi = Vector2.zero();
final Vector2 _inCone_pi1 = Vector2.zero();
final Vector2 _inCone_pin1 = Vector2.zero();

// Helper function to calculate area of polygon
int calcAreaOfPolygon2D(List<double> verts, int nverts) {
  double area = 0;
  int j = nverts - 1;
  for (int i = 0; i < nverts; j = i++) {
    final vi = i * 4;
    final vj = j * 4;
    area += verts[vi] * verts[vj + 2] - verts[vj] * verts[vi + 2];
  }
  return ((area + 1) / 2).floor();
}

// Helper functions for polygon operations
int prev(int i, int n) => (i - 1 >= 0 ? i - 1 : n - 1);
int next(int i, int n) => (i + 1 < n ? i + 1 : 0);

double _area2(Vector2 a, Vector2 b, Vector2 c) {
  return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]);
}

// Returns true iff c is strictly to the left of the directed line through a to b.
bool _left(Vector2 a, Vector2 b, Vector2 c) {
  return _area2(a, b, c) < 0;
}

bool _leftOn(Vector2 a, Vector2 b, Vector2 c) {
  return _area2(a, b, c) <= 0;
}

bool _collinear(Vector2 a, Vector2 b, Vector2 c) {
  return _area2(a, b, c) == 0;
}

bool _xorb(bool x, bool y) {
  return !x != !y;
}

// Returns true iff ab properly intersects cd: they share a point interior to both segments.
bool _intersectProp(Vector2 a, Vector2 b, Vector2 c, Vector2 d) {
  // Eliminate improper cases.
  if (_collinear(a, b, c) || _collinear(a, b, d) || _collinear(c, d, a) || _collinear(c, d, b)) {
    return false;
  }
  return _xorb(_left(a, b, c), _left(a, b, d)) && _xorb(_left(c, d, a), _left(c, d, b));
}

// Returns true iff (a,b,c) are collinear and point c lies on the closed segment ab.
bool _between(Vector2 a, Vector2 b, Vector2 c) {
  if (!_collinear(a, b, c)) return false;
  // If ab not vertical, check betweenness on x; else on y.
  if (a[0] != b[0]) {
    return (a[0] <= c[0] && c[0] <= b[0]) || (a[0] >= c[0] && c[0] >= b[0]);
  } else {
    return (a[1] <= c[1] && c[1] <= b[1]) || (a[1] >= c[1] && c[1] >= b[1]);
  }
}

// Returns true iff segments ab and cd intersect, properly or improperly.
bool _intersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d) {
  if (_intersectProp(a, b, c, d)) {
    return true;
  } else if (_between(a, b, c) || _between(a, b, d) || _between(c, d, a) || _between(c, d, b)) {
    return true;
  } else {
    return false;
  }
}

bool intersectSegContour(Vector2 d0, Vector2 d1, int i, int n, List<double> verts) {
  // For each edge (k,k+1) of P
  for (int k = 0; k < n; k++) {
    final k1 = next(k, n);
    // Skip edges incident to i.
    if (i == k || i == k1) {
      continue;
    }
    final p0 = _intersectSegContour_p0.setValues(verts[k * 4], verts[k * 4 + 2]);
    final p1 = _intersectSegContour_p1.setValues(verts[k1 * 4], verts[k1 * 4 + 2]);
    
    if (d0.equals(p0) || d1.equals(p0) || 
        d0.equals(p1) || d1.equals(p1)) {
      continue;
    }
    if (_intersect(d0, d1, p0, p1)) {
      return true;
    }
  }
  return false;
}

bool _inCone(int i, int n, List<double> verts, Vector2 pj) {
  final piIdx = i * 4;
  final pi1Idx = next(i, n) * 4;
  final pin1Idx = prev(i, n) * 4;
  
  final pi = _inCone_pi.setValues(verts[piIdx], verts[piIdx + 2]);
  final pi1 = _inCone_pi1.setValues(verts[pi1Idx], verts[pi1Idx + 2]);
  final pin1 = _inCone_pin1.setValues(verts[pin1Idx], verts[pin1Idx + 2]);
  
  // If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
  if (_leftOn(pin1, pi, pi1)) {
    return _left(pi, pj, pin1) && _left(pj, pi, pi1);
  }
  // Assume (i-1,i,i+1) not collinear. Else P[i] is reflex.
  return !(_leftOn(pi, pj, pi1) && _leftOn(pj, pi, pin1));
}

bool vequal(List<int> verticesA, int vertexAIdx, List<int> verticesB, int vertexBIdx) {
  final offsetA = vertexAIdx * 4;
  final offsetB = vertexBIdx * 4;
  return verticesA[offsetA] == verticesB[offsetB] && 
         verticesA[offsetA + 2] == verticesB[offsetB + 2];
}

void removeDegenerateSegments(List<int> simplified) {
  // Remove adjacent vertices which are equal on xz-plane
  int npts = (simplified.length / 4).floor();
  for (int i = npts - 1; i >= 0; --i) {
    final ni = next(i, npts);
    if (vequal(simplified, i, simplified, ni)) {
      // Degenerate segment, remove.
      for (int j = i; j < (simplified.length / 4).floor() - 1; ++j) {
        simplified[j * 4 + 0] = simplified[(j + 1) * 4 + 0];
        simplified[j * 4 + 1] = simplified[(j + 1) * 4 + 1];
        simplified[j * 4 + 2] = simplified[(j + 1) * 4 + 2];
        simplified[j * 4 + 3] = simplified[(j + 1) * 4 + 3];
      }
      simplified.removeRange(simplified.length - 4, simplified.length);
      npts--;
    }
  }
}

bool mergeContours(Contour ca, Contour cb, int ia, int ib) {
  final maxVerts = ca.nVertices + cb.nVertices + 2;
  final verts = List<double>.filled(maxVerts * 4, 0);
  int nv = 0;
  
  // Copy contour A.
  for (int i = 0; i <= ca.nVertices; ++i) {
    final srcIndex = ((ia + i) % ca.nVertices) * 4;
    verts[nv * 4 + 0] = ca.vertices[srcIndex + 0];
    verts[nv * 4 + 1] = ca.vertices[srcIndex + 1];
    verts[nv * 4 + 2] = ca.vertices[srcIndex + 2];
    verts[nv * 4 + 3] = ca.vertices[srcIndex + 3];
    nv++;
  }
  
  // Copy contour B
  for (int i = 0; i <= cb.nVertices; ++i) {
    final srcIndex = ((ib + i) % cb.nVertices) * 4;
    verts[nv * 4 + 0] = cb.vertices[srcIndex + 0];
    verts[nv * 4 + 1] = cb.vertices[srcIndex + 1];
    verts[nv * 4 + 2] = cb.vertices[srcIndex + 2];
    verts[nv * 4 + 3] = cb.vertices[srcIndex + 3];
    nv++;
  }
  
  ca.vertices = verts;
  ca.nVertices = nv;
  cb.vertices = [];
  cb.nVertices = 0;
  return true;
}

class ContourHole {
  Contour contour;
  double minx;
  double minz;
  int leftmost;
  
  ContourHole({
    required this.contour,
    required this.minx,
    required this.minz,
    required this.leftmost,
  });
}

class ContourRegion {
  Contour? outline;
  List<ContourHole> holes;
  
  ContourRegion({this.outline, required this.holes});
}

class PotentialDiagonal {
  int vert;
  int dist;
  
  PotentialDiagonal({required this.vert, required this.dist});
}

class LeftMostResult {
  final double minx;
  final double minz;
  final int leftmost;
  LeftMostResult({required this.minx, required this.minz, required this.leftmost});
}

/// Finds the lowest leftmost vertex of a contour.
LeftMostResult findLeftMostVertex(Contour contour) {
  double minx = contour.vertices[0];
  double minz = contour.vertices[2];
  int leftmost = 0;
  
  for (int i = 1; i < contour.nVertices; i++) {
    final x = contour.vertices[i * 4 + 0];
    final z = contour.vertices[i * 4 + 2];
    if (x < minx || (x == minx && z < minz)) {
      minx = x;
      minz = z;
      leftmost = i;
    }
  }
  return LeftMostResult(minx: minx, minz: minz, leftmost: leftmost);
}

int compareHoles(ContourHole a, ContourHole b) {
  if (a.minx == b.minx) {
    if (a.minz < b.minz) return -1;
    if (a.minz > b.minz) return 1;
  } else {
    if (a.minx < b.minx) return -1;
    if (a.minx > b.minx) return 1;
  }
  return 0;
}

// Assuming types from previous steps are imported:
// typedef Vec2 = List<double>;
// typedef Box3 = List<List<double>>;

final Vector2 _mergeRegionHoles_corner = Vector2.zero();
final Vector2 _mergeRegionHoles_pt = Vector2.zero();

void mergeRegionHoles(BuildContextState ctx, ContourRegion region) {
  // Sort holes from left to right.
  for (int i = 0; i < region.holes.length; i++) {
    final result = findLeftMostVertex(region.holes[i].contour);
    region.holes[i].minx = result.minx;
    region.holes[i].minz = result.minz;
    region.holes[i].leftmost = result.leftmost;
  }
  region.holes.sort(compareHoles);

  int maxVerts = region.outline!.nVertices;
  for (int i = 0; i < region.holes.length; i++) {
    maxVerts += region.holes[i].contour.nVertices;
  }

  final List<PotentialDiagonal> diags = [];
  for(int i = 0; i < maxVerts; i++){
    diags.add(PotentialDiagonal(vert: 0, dist: 0));
  }
  final outline = region.outline!;

  // Merge holes into the outline one by one.
  for (int i = 0; i < region.holes.length; i++) {
    final hole = region.holes[i].contour;

    int index = -1;
    int bestVertex = region.holes[i].leftmost;

    for (int iter = 0; iter < hole.nVertices; iter++) {
      int ndiags = 0;
      final corner = _mergeRegionHoles_corner.setValues(
        hole.vertices[bestVertex * 4 + 0],
        hole.vertices[bestVertex * 4 + 2],
      );

      for (int j = 0; j < outline.nVertices; j++) {
        if (_inCone(j, outline.nVertices, outline.vertices, corner)) {
          final dx = outline.vertices[j * 4 + 0] - corner[0];
          final dz = outline.vertices[j * 4 + 2] - corner[1];
          diags[ndiags].vert = j;
          diags[ndiags].dist = (dx * dx + dz * dz).toInt();
          ndiags++;
        }
      }

      // Sort potential diagonals by distance to keep connection short.
      if (ndiags > 1) {
        for (int a = 0; a < ndiags - 1; a++) {
          for (int b = a + 1; b < ndiags; b++) {
            if (diags[a].dist > diags[b].dist) {
              final temp = diags[a];
              diags[a] = diags[b];
              diags[b] = temp;
            }
          }
        }
      }

      // Find a diagonal that is not intersecting the outline or the remaining holes.
      index = -1;
      for (int j = 0; j < ndiags; j++) {
        final ptIdx = diags[j].vert * 4;
        final pt = _mergeRegionHoles_pt.setValues(
          outline.vertices[ptIdx],
          outline.vertices[ptIdx + 2],
        );

        bool intersect = intersectSegContour(pt, corner, diags[i].vert, outline.nVertices, outline.vertices);

        for (int k = i; k < region.holes.length && !intersect; k++) {
          intersect = intersect ||
              intersectSegContour(pt, corner, -1, region.holes[k].contour.nVertices, region.holes[k].contour.vertices);
        }

        if (!intersect) {
          index = diags[j].vert;
          break;
        }
      }

      // If found non-intersecting diagonal, stop looking.
      if (index != -1) {
        break;
      }

      // All potential diagonals were intersecting, try next vertex.
      bestVertex = (bestVertex + 1) % hole.nVertices;
    }

    if (index == -1) {
      print('mergeHoles: Failed to find merge points for outline and hole.');
      continue;
    }

    if (!mergeContours(region.outline!, hole, index, bestVertex)) {
      print('mergeHoles: Failed to merge contours.');
    }
  }
}

ContourSet buildContours(
  BuildContextState ctx,
  CompactHeightfield compactHeightfield,
  double maxSimplificationError,
  double maxEdgeLength,
  int buildFlags,
) {
  final width = compactHeightfield.width;
  final height = compactHeightfield.height;
  final borderSize = compactHeightfield.borderSize;

  // Create bounding box deep copy
  final clonedBounds = BoundingBox(
    compactHeightfield.bounds.min,
    compactHeightfield.bounds.max,
  );

  // Initialize contour set
  final contourSet = ContourSet(
    contours: [],
    bounds: clonedBounds,
    cellSize: compactHeightfield.cellSize,
    cellHeight: compactHeightfield.cellHeight,
    width: compactHeightfield.width - compactHeightfield.borderSize * 2,
    height: compactHeightfield.height - compactHeightfield.borderSize * 2,
    borderSize: compactHeightfield.borderSize,
    maxError: maxSimplificationError,
  );

  // If the heightfield was built with borderSize, remove the offset.
  if (borderSize > 0) {
    final pad = borderSize * compactHeightfield.cellSize;
    contourSet.bounds.min.x += pad;
    contourSet.bounds.min.z += pad;
    contourSet.bounds.max.x -= pad;
    contourSet.bounds.max.z -= pad;
  }

  final flags = List<int>.filled(compactHeightfield.spanCount, 0);

  // Mark boundaries.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      final c = compactHeightfield.cells[x + y * width];
      for (int i = c.index; i < c.index + c.count; ++i) {
        int res = 0;
        final s = compactHeightfield.spans[i];
        if (compactHeightfield.spans[i].region == 0 || (compactHeightfield.spans[i].region & borderReg) != 0) {
          flags[i] = 0;
          continue;
        }
        for (int dir = 0; dir < 4; ++dir) {
          int r = 0;
          if (getCon(s, dir) != notConnected) {
            final ax = x + getDirOffsetX(dir);
            final ay = y + getDirOffsetY(dir);
            final ai = compactHeightfield.cells[ax + ay * width].index + getCon(s, dir);
            r = compactHeightfield.spans[ai].region;
          }
          if (r == compactHeightfield.spans[i].region) {
            res |= 1 << dir;
          }
        }
        flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
      }
    }
  }

  final List<int> verts = [];
  final List<int> simplified = [];

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      final c = compactHeightfield.cells[x + y * width];
      for (int i = c.index; i < c.index + c.count; ++i) {
        if (flags[i] == 0 || flags[i] == 0xf) {
          flags[i] = 0;
          continue;
        }
        final region = compactHeightfield.spans[i].region;
        if (region == 0 || (region & borderReg) != 0) {
          continue;
        }
        final area = compactHeightfield.areas[i];
        verts.clear();
        simplified.clear();

        _walkContour(x, y, i, compactHeightfield, flags, verts);
        simplifyContour(verts, simplified, maxSimplificationError, maxEdgeLength, buildFlags);
        removeDegenerateSegments(simplified);

        // Create contour.
        if ((simplified.length / 4).floor() >= 3) {
          final contour = Contour(
            nVertices: (simplified.length / 4).floor(),
            vertices: simplified.map((e) => e.toDouble()).toList(),
            nRawVertices: (verts.length / 4).floor(),
            rawVertices: verts.map((e) => e.toDouble()).toList(),
            reg: region,
            area: area,
          );

          if (borderSize > 0) {
            // If the heightfield was built with bordersize, remove the offset.
            for (int j = 0; j < contour.nVertices; ++j) {
              contour.vertices[j * 4 + 0] -= borderSize;
              contour.vertices[j * 4 + 2] -= borderSize;
            }
            for (int j = 0; j < contour.nRawVertices; ++j) {
              contour.rawVertices[j * 4 + 0] -= borderSize;
              contour.rawVertices[j * 4 + 2] -= borderSize;
            }
          }
          contourSet.contours.add(contour);
        }
      }
    }
  }

  // Merge holes if needed.
  if (contourSet.contours.isNotEmpty) {
    // Calculate winding of all polygons.
    final winding = List<int>.filled(contourSet.contours.length, 0);
    int nholes = 0;
    for (int i = 0; i < contourSet.contours.length; ++i) {
      final contour = contourSet.contours[i];
      // If the contour is wound backwards, it is a hole.
      winding[i] = calcAreaOfPolygon2D(contour.vertices, contour.nVertices) < 0 ? -1 : 1;
      if (winding[i] < 0) {
        nholes++;
      }
    }

    if (nholes > 0) {
      // Collect outline contour and holes contours per region.
      final nregions = compactHeightfield.maxRegions + 1;
      final List<ContourRegion> regions = [];
      for (int i = 0; i < nregions; i++) {
        regions.add(ContourRegion(outline: null, holes: []));
      }

      final List<ContourHole> holes = [];
      for (int i = 0; i < contourSet.contours.length; i++) {
          holes.add(ContourHole(
          contour: contourSet.contours[i],
          minx: 0,
          minz: 0,
          leftmost: 0,
        ));
      }

      for (int i = 0; i < contourSet.contours.length; ++i) {
        final contour = contourSet.contours[i];
        final region = regions[contour.reg];
        // Positively wound contours are outlines, negative holes.
        if (winding[i] > 0) {
          if (region.outline != null) {
            print('buildContours: Multiple outlines for region ${contour.reg}.');
          }
          region.outline = contour;
        } else {
          region.holes.add(holes[i]);
        }
      }

      // Finally merge each region's holes into the outline.
      for (int i = 0; i < nregions; i++) {
        final region = regions[i];
        if (region.holes.isEmpty) continue;
        if (region.outline != null) {
          mergeRegionHoles(ctx, region);
        } 
        else {
          print('buildContours: Bad outline for region $i, contour simplification is likely too aggressive.',);
        }
      }
    }
  }

  return contourSet;
}