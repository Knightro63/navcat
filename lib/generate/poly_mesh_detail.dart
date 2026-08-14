import 'dart:math' as math;
import 'package:navcat/math/circle.dart';
import 'package:navcat/navcat.dart';
import 'package:three_js_math/three_js_math.dart';

// --- Global Pipeline Core Constants ---
const int detailEdgeBoundary = 0x1;
const int unsetHeight = 0xffff;
const int maxVerts = 127;
const int maxTris = 255;
const int maxVertsPerEdge = 32;
const int retraceSize = 256;

// Edge values enum / identifiers
const int evUndef = -1;
const int evHull = -2;

// --- Core Helper Functions ---

/// Helper to extract 2D vector from 3D array (x, z components)
Vector2 getVec2XZ(Vector2 out, List<double> arr, [int index = 0]) {
  out.x = arr[index];       // x component
  out.y = arr[index + 2];   // z component (skip y mapping into standard Vector2.y)
  return out;
}

// Jitter functions for sampling
double getJitterX(int i) {
  return (((i * 0x8da6b343) & 0xffff) / 65535.0) * 2.0 - 1.0;
}

double getJitterY(int i) {
  return (((i * 0xd8163841) & 0xffff) / 65535.0) * 2.0 - 1.0;
}

/// Height sampling function with spiral search logic
int getHeight(
  double fx,
  double fy,
  double fz,
  double _cs,
  double ics,
  double ch,
  int radius,
  HeightPatch hp, // Maps directly to your dynamic HeightPatch mapping structure
) {
  int ix = (fx * ics + 0.01).floor();
  int iz = (fz * ics + 0.01).floor();
  
  // Custom integer clamping parameters mapping to dart math rules
  ix = (ix - hp.xmin).clamp(0, hp.width- 1);
  iz = (iz - hp.ymin).clamp(0, hp.height - 1);
  
  final int hpWidth = hp.width;
  final List<int> hpData = hp.data;
  
  int h = hpData[ix + iz * hpWidth];
  if (h == unsetHeight) {
    int x = 1;
    int z = 0;
    int dx = 1;
    int dz = 0;
    final int maxSize = radius * 2 + 1;
    final int maxIter = maxSize * maxSize - 1;
    int nextRingIterStart = 8;
    int nextRingIters = 16;
    double dmin = double.infinity;
    
    for (int i = 0; i < maxIter; i++) {
      final int nx = ix + x;
      final int nz = iz + z;
      if (nx >= 0 && nz >= 0 && nx < hpWidth && nz < hp.height) {
        final int nh = hpData[nx + nz * hpWidth];
        if (nh != unsetHeight) {
          final double d = (nh * ch - fy).abs();
          if (d < dmin) {
            h = nh;
            dmin = d;
          }
        }
      }
      
      if (i + 1 == nextRingIterStart) {
        if (h != unsetHeight) break;
        nextRingIterStart += nextRingIters;
        nextRingIters += 8;
      }
      
      if (x == z || (x < 0 && x == -z) || (x > 0 && x == 1 - z)) {
        final int tmp = dx;
        dx = -dz;
        dz = tmp;
      }
      x += dx;
      z += dz;
    }
  }
  return h;
}

// --- Edge management functions for triangulation ---

int findEdge(List<int> edges, int nedges, int s, int t) {
  for (int i = 0; i < nedges; i++) {
    final int e = i * 4;
    if ((edges[e] == s && edges[e + 1] == t) || (edges[e] == t && edges[e + 1] == s)) {
      return i;
    }
  }
  return evUndef;
}

int addEdge(
  BuildContextState ctx, // BuildContextState mapping target
  List<int> edges,
  Nint nedges, // Maps to wrap structural configuration mutations cleanly { 'value': int }
  int maxEdges,
  int s,
  int t,
  int l,
  int r,
) {
  if (nedges.value >= maxEdges) {
    print("addEdge: Too many edges (${nedges.value}/$maxEdges).");
    return evUndef;
  }
  
  // Add edge if not already in the triangulation matrix.
  final int e = findEdge(edges, nedges.value, s, t);
  if (e == evUndef) {
    final int edgeIdx = nedges.value  * 4;
    edges[edgeIdx] = s;
    edges[edgeIdx + 1] = t;
    edges[edgeIdx + 2] = l;
    edges[edgeIdx + 3] = r;
    
    return nedges.value ++;
  }
  return evUndef;
}

// --- Zero Allocation Scratchpad Infrastructure Layers ---
final Vector3 _completeFacetC = Vector3();
final Vector2 _completeFacetPointS = Vector2();
final Vector2 _completeFacetPointT = Vector2();
final Vector2 _completeFacetPointU = Vector2();
final Vector2 _completeFacetCircleCenter = Vector2();
final Vector2 _completeFacetDistanceCalc = Vector2();

final _triangleV1 = Vector2();
final _triangleV2 = Vector2();
final _triangleV3 = Vector2();
final Circle _circumcircleResult = Circle();   // Center coordinate representation matching circle x, y, r

/// Triangle completion function for Delaunay triangulation
void completeFacet(
  BuildContextState ctx,
  List<double> points,
  int nPoints,
  List<int> edges,
  Nint nEdges, // Maps to reference container: { 'value': int }
  int maxEdges,
  Nint nFaces, // Maps to reference container: { 'value': int }
  int e,
) {
  const double epsFacet = 1e-5;
  final int edgeIdx = e * 4;
  
  // Cache s and t.
  int s;
  int t;
  if (edges[edgeIdx + 2] == evUndef) {
    s = edges[edgeIdx];
    t = edges[edgeIdx + 1];
  } else if (edges[edgeIdx + 3] == evUndef) {
    s = edges[edgeIdx + 1];
    t = edges[edgeIdx];
  } else {
    // Edge already completed.
    return;
  }
  
  // Find best point on left of edge.
  int pt = nPoints;
  final Vector3 c = _completeFacetC.setValues(0.0, 0.0, 0.0);
  double r = -1.0;
  
  for (int u = 0; u < nPoints; ++u) {
    if (u == s || u == t) continue;
    
    // Calculate cross product to check if points are in correct order for triangle
    getVec2XZ(_completeFacetPointS, points, s * 3);
    getVec2XZ(_completeFacetPointT, points, t * 3);
    getVec2XZ(_completeFacetPointU, points, u * 3);
    
    _completeFacetPointT.sub(_completeFacetPointS); // t - s
    _completeFacetPointU.sub(_completeFacetPointS); // u - s
    
    final double crossProduct = _completeFacetPointT.x * _completeFacetPointU.y - _completeFacetPointT.y * _completeFacetPointU.x;
    
    if (crossProduct > epsFacet) {
      if (r < 0) {
        // The circle is not updated yet, do it now.
        pt = u;
        getVec2XZ(_triangleV1, points, s * 3);
        getVec2XZ(_triangleV2, points, t * 3);
        getVec2XZ(_triangleV3, points, u * 3);
        
        // Custom math binding wrapper to match your native geometry solver engine
        circumcircle(_circumcircleResult, _triangleV1, _triangleV2, _triangleV3);
        
        final Vector2 center = _circumcircleResult.center;
        c.x = center.x;
        c.y = 0.0;
        c.z = center.y;
        r = _circumcircleResult.radius;
        continue;
      }
      
      getVec2XZ(_completeFacetCircleCenter, [c.x, c.y, c.z], 0);
      getVec2XZ(_completeFacetDistanceCalc, points, u * 3);
      
      final double d = _completeFacetCircleCenter.distanceTo(_completeFacetDistanceCalc);
      const double tol = 0.001;
      
      if (d > r * (1 + tol)) {
        // Outside current circumcircle, skip
        continue;
      }
      
      if (d >= r * (1 - tol)) {
        // Inside epsilon circumcircle, do extra tests to make sure the edge is valid.
        if (overlapEdges(points, edges, nEdges.value, s, u)) continue;
        if (overlapEdges(points, edges, nEdges.value, t, u)) continue;
      }
      
      // Edge is valid.
      pt = u;
      getVec2XZ(_triangleV1, points, s * 3);
      getVec2XZ(_triangleV2, points, t * 3);
      getVec2XZ(_triangleV3, points, u * 3);
      
      circumcircle(_circumcircleResult, _triangleV1, _triangleV2, _triangleV3);
      
      final Vector2 center = _circumcircleResult.center;
      c.x = center.x;
      c.y = 0.0;
      c.z = center.y;
      r = _circumcircleResult.radius;
    }
  }
  
  // Add new triangle or update edge info if s-t is on hull.
  if (pt < nPoints) {
    // Update face information of edge being completed.
    updateLeftFace(edges, e, s, t, nFaces.value);
    
    // Add new edge or update face info of old edge.
    int newE = findEdge(edges, nEdges.value, pt, s);
    if (newE == evUndef) {
      addEdge(ctx, edges, nEdges, maxEdges, pt, s, nFaces.value, evUndef);
    } else {
      updateLeftFace(edges, newE, pt, s, nFaces.value);
    }
    
    // Add new edge or update face info of old edge.
    newE = findEdge(edges, nEdges.value, t, pt);
    if (newE == evUndef) {
      addEdge(ctx, edges, nEdges, maxEdges, t, pt, nFaces.value, evUndef);
    } else {
      updateLeftFace(edges, newE, t, pt, nFaces.value);
    }
    
    nFaces.value++;
  } else {
    updateLeftFace(edges, e, s, t, evHull);
  }
}

void updateLeftFace(List<int> edges, int edgeIdx, int s, int t, int f) {
  final int e = edgeIdx * 4;
  if (edges[e] == s && edges[e + 1] == t && edges[e + 2] == evUndef) {
    edges[e + 2] = f;
  } else if (edges[e + 1] == s && edges[e] == t && edges[e + 3] == evUndef) {
    edges[e + 3] = f;
  }
}

final Vector2 _overlapEdgesS0 = Vector2();
final Vector2 _overlapEdgesT0 = Vector2();
final Vector2 _overlapEdgesS1 = Vector2();
final Vector2 _overlapEdgesT1 = Vector2();

bool overlapEdges(List<double> pts, List<int> edges, int nedges, int s1, int t1) {
  for (int i = 0; i < nedges; ++i) {
    final int s0 = edges[i * 4];
    final int t0 = edges[i * 4 + 1];
    
    // Same or connected edges do not overlap.
    if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1) continue;
    
    getVec2XZ(_overlapEdgesS0, pts, s0 * 3);
    getVec2XZ(_overlapEdgesT0, pts, t0 * 3);
    getVec2XZ(_overlapEdgesS1, pts, s1 * 3);
    getVec2XZ(_overlapEdgesT1, pts, t1 * 3);
    
    if (overlapSegSeg2d(_overlapEdgesS0, _overlapEdgesT0, _overlapEdgesS1, _overlapEdgesT1)) return true;
  }
  return false;
}

/// Delaunay triangulation hull function
void delaunayHull(
  BuildContextState ctx,
  int npts,
  List<double> pts,
  int nhull,
  List<int> hull,
  List<int> tris,
  List<int> edges,
) {
  final Nint nfaces = Nint();
  final Nint nedges = Nint();
  final int maxEdges = npts * 10;
  
  // Re-allocating/clearing fixed buffers via replacement to match pre-allocated scaling rules
  if (edges.length < maxEdges * 4) {
    edges.addAll(List<int>.filled((maxEdges * 4) - edges.length, 0));
  }
  
  for (int i = 0, j = nhull - 1; i < nhull; j = i++) {
    addEdge(ctx, edges, nedges, maxEdges, hull[j], hull[i], evHull, evUndef);
  }
  
  int currentEdge = 0;
  while (currentEdge < nedges.value) {
    if (edges[currentEdge * 4 + 2] == evUndef) {
      completeFacet(ctx, pts, npts, edges, nedges, maxEdges, nfaces, currentEdge);
    }
    if (edges[currentEdge * 4 + 3] == evUndef) {
      completeFacet(ctx, pts, npts, edges, nedges, maxEdges, nfaces, currentEdge);
    }
    currentEdge++;
  }
  
  // Clean up and populate triangles array elements
  final int totalFaceElements = nfaces.value * 4;
  tris.clear();
  tris.addAll(List<int>.filled(totalFaceElements, -1));
  
  for (int i = 0; i < nedges.value; ++i) {
    final int e = i * 4;
    if (edges[e + 3] >= 0) {
      // Left face
      final int t = edges[e + 3] * 4;
      if (tris[t] == -1) {
        tris[t] = edges[e];
        tris[t + 1] = edges[e + 1];
      } else if (tris[t] == edges[e + 1]) {
        tris[t + 2] = edges[e];
      } else if (tris[t + 1] == edges[e]) {
        tris[t + 2] = edges[e + 1];
      }
    }
    if (edges[e + 2] >= 0) {
      // Right
      final int t = edges[e + 2] * 4;
      if (tris[t] == -1) {
        tris[t] = edges[e + 1];
        tris[t + 1] = edges[e];
      } else if (tris[t] == edges[e]) {
        tris[t + 2] = edges[e + 1];
      } else if (tris[t + 1] == edges[e + 1]) {
        tris[t + 2] = edges[e];
      }
    }
  }
  
  // Remove dangling faces
  for (int i = 0; i < tris.length ~/ 4; ++i) {
    final int t = i * 4;
    if (tris[t] == -1 || tris[t + 1] == -1 || tris[t + 2] == -1) {
      // Replace with your project logger wrapper context rule if needed
      print('delaunayHull: Removing dangling face $i [${tris[t]},${tris[t + 1]},${tris[t + 2]}].');
      
      tris[t] = tris[tris.length - 4];
      tris[t + 1] = tris[tris.length - 3];
      tris[t + 2] = tris[tris.length - 2];
      tris[t + 3] = tris[tris.length - 1];
      
      // Trim last 4 elements safely
      tris.removeRange(tris.length - 4, tris.length);
      --i;
    }
  }
}

final _circumcircleV1 = Vector2();
final _circumcircleV2 = Vector2();
final _circumcircleV3 = Vector2();

// Global placeholder integration helper for circumcircle algorithm updates
Circle circumcircle(Circle out, Vector2 a, Vector2 b, Vector2 c){
  // calculate the circle relative to p1, to avoid some precision issues.
  final v1 = _circumcircleV1;
  final v2 = _circumcircleV2;
  final v3 = _circumcircleV3;

  // v1 is the origin (p1 - p1 = 0), v2 and v3 are relative to p1
  v1.setValues(0, 0);
  v2.sub2(b, a);
  v3.sub2(c, a);

  // calculate cross product for 2D vectors (v2 - v1) × (v3 - v1)
  v2.sub2(v2, v1); // v2 - v1
  v3.sub2(v3, v1); // v3 - v1
  final cp = v2[0] * v3[1] - v2[1] * v3[0];

  if (cp.abs() > MathUtils.epsilon) {
    final v1Sq = _circumcircleV1.dot(_circumcircleV1);
    final v2Sq = _circumcircleV2.dot(_circumcircleV2);
    final v3Sq = _circumcircleV3.dot(_circumcircleV3);
    out.center[0] = (v1Sq * (v2[1] - v3[1]) + v2Sq * (v3[1] - v1[1]) + v3Sq * (v1[1] - v2[1])) / (2 * cp);
    out.center[1] = (v1Sq * (v3[0] - v2[0]) + v2Sq * (v1[0] - v3[0]) + v3Sq * (v2[0] - v1[0])) / (2 * cp);

    final r = out.center.distanceTo(v1);

    out.center.add2(out.center, a);

    out.radius = r;

    return out;
  }

  out.center.setFrom(a);

  out.radius = 0;

  return out;
}

// --- Zero Allocation Scratchpad Infrastructure Layers ---
final Vector2 _triangulateHullPrev = Vector2();
final Vector2 _triangulateHullCurrent = Vector2();
final Vector2 _triangulateHullNext = Vector2();
final Vector2 _triangulateHullRight = Vector2();

void triangulateHull(List<double> verts, int nhull, List<int> hull, int nin, List<int> tris) {
  int start = 0;
  int left = 1;
  int right = nhull - 1;
  
  // Start from an ear with shortest perimeter.
  double dmin = double.maxFinite;
  for (int i = 0; i < nhull; i++) {
    if (hull[i] >= nin) continue;
    
    // Ears are triangles with original vertices as middle vertex
    final int pi = prev(i, nhull);
    final int ni = next(i, nhull);
    final int pv = hull[pi] * 3;
    final int cv = hull[i] * 3;
    final int nv = hull[ni] * 3;
    
    // Calculate triangle perimeter using 2D distances
    getVec2XZ(_triangulateHullPrev, verts, pv);
    getVec2XZ(_triangulateHullCurrent, verts, cv);
    getVec2XZ(_triangulateHullNext, verts, nv);
    
    final double d = _triangulateHullPrev.distanceTo(_triangulateHullCurrent) +
                     _triangulateHullCurrent.distanceTo(_triangulateHullNext) +
                     _triangulateHullNext.distanceTo(_triangulateHullPrev);
    
    if (d < dmin) {
      start = i;
      left = ni;
      right = pi;
      dmin = d;
    }
  }
  
  // Add first triangle
  tris.add(hull[start]);
  tris.add(hull[left]);
  tris.add(hull[right]);
  tris.add(0);
  
  // Triangulate the polygon by moving left or right
  while (next(left, nhull) != right) {
    // Check to see if we should advance left or right.
    final int nleft = next(left, nhull);
    final int nright = prev(right, nhull);
    final int cvleft = hull[left] * 3;
    final int nvleft = hull[nleft] * 3;
    final int cvright = hull[right] * 3;
    final int nvright = hull[nright] * 3;
    
    // Calculate distances for left and right triangulation options
    getVec2XZ(_triangulateHullPrev, verts, cvleft);
    getVec2XZ(_triangulateHullCurrent, verts, nvleft);
    getVec2XZ(_triangulateHullNext, verts, cvright);
    getVec2XZ(_triangulateHullRight, verts, nvright);
    
    final double dleft = _triangulateHullPrev.distanceTo(_triangulateHullCurrent) +
                         _triangulateHullCurrent.distanceTo(_triangulateHullNext);
    final double dright = _triangulateHullNext.distanceTo(_triangulateHullRight) +
                          _triangulateHullPrev.distanceTo(_triangulateHullRight);
    
    if (dleft < dright) {
      tris.add(hull[left]);
      tris.add(hull[nleft]);
      tris.add(hull[right]);
      tris.add(0);
      left = nleft;
    } else {
      tris.add(hull[left]);
      tris.add(hull[nright]);
      tris.add(hull[right]);
      tris.add(0);
      right = nright;
    }
  }
}

// Check if edge is on hull
bool onHull(int a, int b, int nhull, List<int> hull) {
  // All internal sampled points come after the hull so we can early out for those.
  if (a >= nhull || b >= nhull) return false;
  for (int j = nhull - 1, i = 0; i < nhull; j = i++) {
    if (a == hull[j] && b == hull[i]) return true;
  }
  return false;
}

// Set triangle flags for boundary edges
void setTriFlags(List<int> tris, int nhull, List<int> hull) {
  for (int i = 0; i < tris.length; i += 4) {
    final int a = tris[i];
    final int b = tris[i + 1];
    final int c = tris[i + 2];
    int flags = 0;
    
    flags |= (onHull(a, b, nhull, hull) ? detailEdgeBoundary : 0) << 0;
    flags |= (onHull(b, c, nhull, hull) ? detailEdgeBoundary : 0) << 2;
    flags |= (onHull(c, a, nhull, hull) ? detailEdgeBoundary : 0) << 4;
    
    tris[i + 3] = flags;
  }
}

const List<List<int>> seedArrayWithPolyCenterOffset = [[0, 0], [-1, -1], [0, -1], [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0]];

// Seed array with polygon center for height data collection
void seedArrayWithPolyCenter(
  BuildContextState ctx,
  CompactHeightfield chf,
  List<int> poly,
  int polyStart,
  int nPolys,
  List<double> verts,
  int bs,
  HeightPatch hp,
  List<int> array,
) {
  final offset = seedArrayWithPolyCenterOffset;

  // Find cell closest to a poly vertex
  int startCellX = 0;
  int startCellY = 0;
  int startSpanIndex = -1;
  int dmin = unsetHeight;

  final int hpXmin = hp.xmin;
  final int hpWidth = hp.width;
  final int hpYmin = hp.ymin;
  final int hpHeight = hp.height;
  final List<int> hpData = hp.data;
  final int chfWidth = chf.width;

  for (int j = 0; j < nPolys && dmin > 0; ++j) {
    for (int k = 0; k < 9 && dmin > 0; ++k) {
      final double ax = verts[poly[polyStart + j] * 3] + offset[k][0];
      final double ay = verts[poly[polyStart + j] * 3 + 1];
      final double az = verts[poly[polyStart + j] * 3 + 2] + offset[k][1];

      if (ax < hpXmin || ax >= hpXmin + hpWidth || az < hpYmin || az >= hpYmin + hpHeight) {
        continue;
      }

      final c = chf.cells[(ax + bs + (az + bs) * chfWidth).toInt()];
      for (int i = c.index, ni = c.index + c.count; i < ni && dmin > 0; ++i) {
        final s = chf.spans[i];
        final int d = (ay - s.y).abs().toInt();
        if (d < dmin) {
          startCellX = ax.toInt();
          startCellY = az.toInt();
          startSpanIndex = i;
          dmin = d;
        }
      }
    }
  }

  // Find center of the polygon
  double pcxDouble = 0;
  double pcyDouble = 0;
  for (int j = 0; j < nPolys; ++j) {
    pcxDouble += verts[poly[polyStart + j] * 3];
    pcyDouble += verts[poly[polyStart + j] * 3 + 2];
  }
  
  final int pcx = (pcxDouble / nPolys).floor();
  final int pcy = (pcyDouble / nPolys).floor();

  // Use seeds array as a stack for DFS
  array.clear();
  array.add(startCellX);
  array.add(startCellY);
  array.add(startSpanIndex);

  final List<int> dirs = [0, 1, 2, 3];
  
  for (int i = 0; i < hpData.length; i++) {
    hpData[i] = 0;
  }

  int cx = -1;
  int cy = -1;
  int ci = -1;

  // Safety tracker loop counter to forcefully protect thread cycles
  int safetyCounter = 0;
  const int maxDfsIterations = 4096; 

  while (true) {
    if (safetyCounter++ > maxDfsIterations) {
      print('CRITICAL: DFS pathing exceeded iteration cap. Terminated to prevent crash.');
      break;
    }

    if (array.length < 3) {
      print('Walk towards polygon center failed to reach center');
      break;
    }
    
    ci = array.removeLast();
    cy = array.removeLast();
    cx = array.removeLast();

    if (cx == pcx && cy == pcy) break;

    int directDir;
    if (cx == pcx) {
      directDir = getDirForOffset(0, pcy > cy ? 1 : -1);
    } else {
      directDir = getDirForOffset(pcx > cx ? 1 : -1, 0);
    }

    // FIXED: Properly index swapping elements inside the collection array layer
    int temp = dirs[directDir];
    dirs[directDir] = dirs[3];
    dirs[3] = temp;

    final cs = ci == -1 ? null : chf.spans[ci];
    
    for (int i = 0; i < 4; i++) {
      final int dir = dirs[i];
      if (cs == null || getCon(cs, dir) == notConnected) continue;

      final int newX = cx + getDirOffsetX(dir);
      final int newY = cy + getDirOffsetY(dir);
      final int hpx = newX - hpXmin;
      final int hpy = newY - hpYmin;

      if (hpx < 0 || hpx >= hpWidth || hpy < 0 || hpy >= hpHeight) continue;
      
      final int dataIdx = hpx + hpy * hpWidth;
      if (hpData[dataIdx] != 0) continue;

      hpData[dataIdx] = 1;
      
      array.add(newX);
      array.add(newY);
      array.add(chf.cells[newX + bs + (newY + bs) * chfWidth].index + getCon(cs, dir));
    }
    
    // FIXED: Corrected cleanup inversion reset step layout
    temp = dirs[directDir];
    dirs[directDir] = dirs[3];
    dirs[3] = temp;
  }

  array.clear();
  array.add(cx + bs);
  array.add(cy + bs);
  array.add(ci);

  for (int i = 0; i < hpData.length; i++) {
    hpData[i] = unsetHeight;
  }

  if (ci != -1) {
    final s = chf.spans[ci];
    final int idx = cx - hpXmin + (cy - hpYmin) * hpWidth;
    if (idx >= 0 && idx < hpData.length) {
      hpData[idx] = s.y;
    }
  }
}


/// Get height data for a polygon
void getHeightData(
  BuildContextState ctx,
  CompactHeightfield chf,        // Maps to dynamic compact heightfield engine payload layout
  List<int> poly,
  int polyStart,
  int nPolys,
  List<double> verts,
  int bs,
  HeightPatch hp,         // Maps to structural HeightPatch map
  List<int> queue,
  int region,
) {
  // Clear the queue array
  queue.clear();
  
  final int hpWidth = hp.width;
  final int hpHeight = hp.height;
  final int hpXmin = hp.xmin;
  final int hpYmin = hp.ymin;
  final List<int> hpData = hp.data;
  final int chfWidth = chf.width;

  hpData.fillRange(0, hpData.length, unsetHeight);
  bool empty = true;
  
  // We cannot sample from this poly if it was created from polys of different regions.
  if (region != multipleRegs) {
    // Copy the height from the same region, and mark region borders as seed points to fill the rest.
    for (int hy = 0; hy < hpHeight; hy++) {
      final int y = hpYmin + hy + bs;
      for (int hx = 0; hx < hpWidth; hx++) {
        final int x = hpXmin + hx + bs;
        final c = chf.cells[x + y * chfWidth];
        
        final int cIndex = c.index;
        final int ni = cIndex + c.count;
        
        for (int i = cIndex; i < ni; ++i) {
          final s = chf.spans[i];
          if (s.region == region) {
            // Store height
            hpData[hx + hy * hpWidth] = s.y;
            empty = false;
            
            // If any of the neighbours is not in same region, add the current location as flood fill start
            bool border = false;
            for (int dir = 0; dir < 4; ++dir) {
              if (getCon(s, dir) != notConnected) {
                final ax = x + getDirOffsetX(dir);
                final ay = y + getDirOffsetY(dir);
                final ai = (chf.cells[ax + ay * chfWidth].index) + getCon(s, dir);
                final asSpan = chf.spans[ai];
                
                if (asSpan.region != region) {
                  border = true;
                  break;
                }
              }
            }
            if (border) {
              queue.add(x);
              queue.add(y);
              queue.add(i);
            }
            break;
          }
        }
      }
    }
  }
  
  // If the polygon does not contain any points from the current region or if it could potentially be overlapping polygons
  if (empty) {
    seedArrayWithPolyCenter(ctx, chf, poly, polyStart, nPolys, verts, bs, hp, queue);
  }
  
  int head = 0;
  
  // BFS to collect height data
  while (head * 3 < queue.length) {
    final int cx = queue[head * 3];
    final int cy = queue[head * 3 + 1];
    final int ci = queue[head * 3 + 2];
    head++;
    
    // Memory-retraction/splicing window shift optimizations matching original JS loop rules
    if (head >= retraceSize) {
      head = 0;
      queue.removeRange(0, retraceSize * 3);
    }
    
    final cs = ci == -1?null:chf.spans[ci];
    for (int dir = 0; dir < 4; ++dir) {
      if (cs == null || getCon(cs, dir) == notConnected) continue;
      
      final int ax = cx + getDirOffsetX(dir);
      final int ay = cy + getDirOffsetY(dir);
      final int hx = ax - hpXmin - bs;
      final int hy = ay - hpYmin - bs;
      
      if (hx < 0 || hx >= hpWidth || hy < 0 || hy >= hpHeight) continue;
      if (hpData[hx + hy * hpWidth] != unsetHeight) continue;
      
      final ai = chf.cells[ax + ay * chfWidth].index + getCon(cs, dir);
      final asSpan = chf.spans[ai];
      
      hpData[hx + hy * hpWidth] = asSpan.y;
      queue.add(ax);
      queue.add(ay);
      queue.add(ai);
    }
  }
}

// --- Zero Allocation Scratchpad Infrastructure Layers ---
final Vector3 _buildPolyDetailVj = Vector3();
final Vector3 _buildPolyDetailVi = Vector3();
final Vector3 _buildPolyDetailPt = Vector3();
final Vector3 _buildPolyDetailVa = Vector3();
final Vector3 _buildPolyDetailVb = Vector3();
final Vector3 _buildPolyDetailSamplePt = Vector3();
final Vector3 _buildPolyDetailGridPt = Vector3();
final Vector3 _bmin = Vector3();
final Vector3 _bmax = Vector3();

bool buildPolyDetail(
  BuildContextState ctx,
  List<double> inVerts,
  int nin,
  double sampleDist,
  double sampleMaxError,
  int heightSearchRadius,
  CompactHeightfield chf,        // Maps to dynamic compact heightfield payload engine
  HeightPatch hp,         // Maps to structural HeightPatch map tracking matrix
  List<double> verts,
  List<int> tris,
  List<int> edges,
  List<double> samples,
) {
  final List<double> edge = List<double>.filled((maxVertsPerEdge + 1) * 3, 0.0);
  final List<int> hull = List<int>.filled(maxVerts, 0);
  int nhull = 0;
  int nverts = nin;
  
  // Clear or stretch target output data vertex constraints
  if (verts.length < nin * 3) {
    verts.addAll(List<double>.filled((nin * 3) - verts.length, 0.0));
  }
  
  // Copy input vertices
  for (int i = 0; i < nin; ++i) {
    verts[i * 3] = inVerts[i * 3];
    verts[i * 3 + 1] = inVerts[i * 3 + 1];
    verts[i * 3 + 2] = inVerts[i * 3 + 2];
  }
  
  // Clear arrays 
  edges.clear();
  tris.clear();
  
  final double cs = chf.cellSize;
  final double ics = 1.0 / cs;
  final double chfCellHeight = chf.cellHeight;
  
  // Calculate minimum extents of the polygon based on input data.
  final double minExtent = polyMinExtent(verts, nverts);
  
  // Tessellate outlines.
  if (sampleDist > 0) {
    for (int i = 0, j = nin - 1; i < nin; j = i++) {
      int vjStart = j * 3;
      int viStart = i * 3;
      bool swapped = false;
      
      // Lexicographical segment sorting to prevent outline cracks/seams
      if ((inVerts[vjStart] - inVerts[viStart]).abs() < 1e-6) {
        if (inVerts[vjStart + 2] > inVerts[viStart + 2]) {
          final int tmp = viStart;
          viStart = vjStart;
          vjStart = tmp;
          swapped = true;
        }
      } else {
        if (inVerts[vjStart] > inVerts[viStart]) {
          final int tmp = viStart;
          viStart = vjStart;
          vjStart = tmp;
          swapped = true;
        }
      }
      
      final Vector3 vj = _buildPolyDetailVj.setValues(inVerts[vjStart], inVerts[vjStart + 1], inVerts[vjStart + 2]);
      final Vector3 vi = _buildPolyDetailVi.setValues(inVerts[viStart], inVerts[viStart + 1], inVerts[viStart + 2]);
      
      // Create samples along the edge.
      final double dx = vi.x - vj.x;
      final double dy = vi.y - vj.y;
      final double dz = vi.z - vj.z;
      final double d = math.sqrt(dx * dx + dz * dz);
      
      int nn = 1 + (d / sampleDist).floor();
      if (nn >= maxVertsPerEdge) nn = maxVertsPerEdge - 1;
      if (nverts + nn >= maxVerts) nn = maxVerts - 1 - nverts;
      
      for (int k = 0; k <= nn; ++k) {
        final double u = k / nn;
        final int pos = k * 3;
        edge[pos] = vj.x + dx * u;
        edge[pos + 1] = vj.y + dy * u;
        edge[pos + 2] = vj.z + dz * u;
        edge[pos + 1] = getHeight(edge[pos], edge[pos + 1], edge[pos + 2], cs, ics, chfCellHeight, heightSearchRadius, hp) * chfCellHeight;
      }
      
      // Simplify samples.
      final List<int> idx = List<int>.filled(maxVertsPerEdge, 0);
      idx[0] = 0;
      idx[1] = nn;
      int nidx = 2;
      
      for (int k = 0; k < nidx - 1; ) {
        final int a = idx[k];
        final int b = idx[k + 1];
        final int vaStart = a * 3;
        final int vbStart = b * 3;
        
        // Find maximum deviation along the segment.
        double maxd = 0;
        int maxi = -1;
        for (int m = a + 1; m < b; ++m) {
          final int mStart = m * 3;
          final Vector3 pt = _buildPolyDetailPt.setValues(edge[mStart], edge[mStart + 1], edge[mStart + 2]);
          final Vector3 va = _buildPolyDetailVa.setValues(edge[vaStart], edge[vaStart + 1], edge[vaStart + 2]);
          final Vector3 vb = _buildPolyDetailVb.setValues(edge[vbStart], edge[vbStart + 1], edge[vbStart + 2]);
          
          final double dev = distancePtSegFromVectors(pt, va, vb);
          if (dev > maxd) {
            maxd = dev;
            maxi = m;
          }
        }
        
        // If the max deviation is larger than accepted error, add new point
        if (maxi != -1 && maxd > sampleMaxError * sampleMaxError) {
          for (int m = nidx; m > k; --m) {
            idx[m] = idx[m - 1];
          }
          idx[k + 1] = maxi;
          nidx++;
        } else {
          ++k;
        }
      }
      
      hull[nhull++] = j;
      
      // Add new vertices layout references
      if (swapped) {
        for (int k = nidx - 2; k > 0; --k) {
          final int targetIdx = nverts * 3;
          if (verts.length <= targetIdx + 2) {
            verts.addAll(List<double>.filled((targetIdx + 3) - verts.length, 0.0));
          }
          verts[targetIdx] = edge[idx[k] * 3];
          verts[targetIdx + 1] = edge[idx[k] * 3 + 1];
          verts[targetIdx + 2] = edge[idx[k] * 3 + 2];
          hull[nhull++] = nverts;
          nverts++;
        }
      } else {
        for (int k = 1; k < nidx - 1; ++k) {
          final int targetIdx = nverts * 3;
          if (verts.length <= targetIdx + 2) {
            verts.addAll(List<double>.filled((targetIdx + 3) - verts.length, 0.0));
          }
          verts[targetIdx] = edge[idx[k] * 3];
          verts[targetIdx + 1] = edge[idx[k] * 3 + 1];
          verts[targetIdx + 2] = edge[idx[k] * 3 + 2];
          hull[nhull++] = nverts;
          nverts++;
        }
      }
    }
  }
  
  // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
  if (minExtent < sampleDist * 2) {
    triangulateHull(verts, nhull, hull, nin, tris);
    setTriFlags(tris, nhull, hull);
    return true;
  }
  
  // Tessellate the base mesh using triangulateHull
  triangulateHull(verts, nhull, hull, nin, tris);
  if (tris.isEmpty) {
    print('buildPolyDetail: Could not triangulate polygon ($nverts verts).');
    return true;
  }
  
  if (sampleDist > 0) {
    // Create sample locations in a grid.
    final Vector3 bmin = _bmin.setValues(inVerts[0], inVerts[1], inVerts[2]);
    final Vector3 bmax = _bmax.setValues(inVerts[0], inVerts[1], inVerts[2]);
    
    for (int i = 1; i < nin; ++i) {
      bmin.x = math.min(bmin.x, inVerts[i * 3]);
      bmin.y = math.min(bmin.y, inVerts[i * 3 + 1]);
      bmin.z = math.min(bmin.z, inVerts[i * 3 + 2]);
      
      bmax.x = math.max(bmax.x, inVerts[i * 3]);
      bmax.y = math.max(bmax.y, inVerts[i * 3 + 1]);
      bmax.z = math.max(bmax.z, inVerts[i * 3 + 2]);
    }
    
    final int x0 = (bmin.x / sampleDist).floor();
    final int x1 = (bmax.x / sampleDist).ceil();
    final int z0 = (bmin.z / sampleDist).floor();
    final int z1 = (bmax.z / sampleDist).ceil();
    
    samples.clear();
    
    for (int z = z0; z < z1; ++z) {
      for (int x = x0; x < x1; ++x) {
        final List<double> pt = [x * sampleDist, (bmax.y + bmin.y) * 0.5, z * sampleDist];
        
        // Make sure the samples are not too close to the edges.
        final Vector3 gridPt = _buildPolyDetailGridPt.setValues(pt[0], pt[1], pt[2]);
        if (distToPoly(nin, inVerts, gridPt) > -sampleDist / 2) continue;
        
        samples.add(x.toDouble());
        samples.add(getHeight(pt[0], pt[1], pt[2], cs, ics, chfCellHeight, heightSearchRadius, hp).toDouble());
        samples.add(z.toDouble());
        samples.add(0.0); // Not added
      }
    }
    
    // Add the samples starting from the one that has the most error.
    final int nsamples = (samples.length / 4).floor();
    for (int iter = 0; iter < nsamples; ++iter) {
      if (nverts >= maxVerts) break;
      
      final List<double> bestpt = [0.0, 0.0, 0.0];
      double bestd = 0;
      int besti = -1;
      
      for (int i = 0; i < nsamples; ++i) {
        final int s = i * 4;
        if (samples[s + 3] != 0.0) continue; // skip added
        
        final List<double> pt = [
          samples[s] * sampleDist + getJitterX(i) * cs * 0.1,
          samples[s + 1] * chfCellHeight,
          samples[s + 2] * sampleDist + getJitterY(i) * cs * 0.1,
        ];
        
        final Vector3 samplePt = _buildPolyDetailSamplePt.setValues(pt[0], pt[1], pt[2]);
        final double d = distToTriMesh(samplePt, verts, tris, (tris.length / 4).floor());
        
        if (d < 0) continue; // did not hit the mesh
        if (d > bestd) {
          bestd = d;
          besti = i;
          bestpt[0] = pt[0];
          bestpt[1] = pt[1];
          bestpt[2] = pt[2];
        }
      }
      
      // If the max error is within accepted threshold, stop tessellating.
      if (bestd <= sampleMaxError || besti == -1) break;
      
      // Mark sample as added.
      samples[besti * 4 + 3] = 1.0;
      
      // Add the new sample point.
      verts.add(bestpt[0]);
      verts.add(bestpt[1]);
      verts.add(bestpt[2]);
      nverts++;
      
      // Rebuild the Delaunay triangulation based on updated nodes
      edges.clear();
      tris.clear();
      delaunayHull(ctx, nverts, verts, nhull, hull, tris, edges);
    }
  }
  
  final int ntris = (tris.length / 4).floor();
  if (ntris > maxTris) {
    tris.removeRange(maxTris * 4, tris.length);
    print('buildPolyMeshDetail: Shrinking triangle count from $ntris to max $maxTris.');
  }
  
  setTriFlags(tris, nhull, hull);
  return true;
}

/// Contains triangle meshes that represent detailed height data associated 
/// with the polygons in its associated polygon mesh object.
PolyMeshDetail buildPolyMeshDetail(
  BuildContextState ctx,
  PolyMesh polyMesh,
  CompactHeightfield compactHeightfield,
  double sampleDist,
  double sampleMaxError,
) {
  final int polyMeshNVertices = polyMesh.nVertices;
  final int polyMeshNPolys = polyMesh.nPolys;

  if (polyMeshNVertices == 0 || polyMeshNPolys == 0) {
    return PolyMeshDetail(
      nMeshes: 0, 
      meshes: <int>[], 
      vertices: <double>[], 
      triangles: <int>[]
    );
  }

  final int nvp = polyMesh.maxVerticesPerPoly;
  final double cs = polyMesh.cellSize;
  final double ch = polyMesh.cellHeight;
  
  // Handling three.js bounds access structure (usually a multi-dimensional array or list of lists)
  final List<double> orig = [
    polyMesh.bounds.min[0],
    polyMesh.bounds.min[1],
    polyMesh.bounds.min[2],
  ];
  
  final int borderSize = polyMesh.borderSize.toInt();
  final int heightSearchRadius = math.max(1, (polyMesh.maxEdgeError).ceil());

  // HeightPatch layout matching previous segments
  final HeightPatch hp = HeightPatch();

  int maxhw = 0;
  int maxhh = 0;

  // Calculate bounds for each polygon
  final List<int> bounds = List<int>.filled(polyMeshNPolys * 4, 0);
  final List<double> poly = List<double>.filled(nvp * 3, 0.0);

  final int chfWidth = compactHeightfield.width;
  final int chfHeight = compactHeightfield.height;

  // Find max size for a polygon area.
  for (int i = 0; i < polyMeshNPolys; ++i) {
    final int p = i * nvp;
    int xmin = chfWidth;
    int xmax = 0;
    int ymin = chfHeight;
    int ymax = 0;

    for (int j = 0; j < nvp; ++j) {
      if (polyMesh.polys[p + j] == meshNullIdx) break;
      final int v = (polyMesh.polys[p + j]) * 3;
      
      xmin = math.min(xmin, polyMesh.vertices[v]).toInt();
      xmax = math.max(xmax, polyMesh.vertices[v]).toInt();
      ymin = math.min(ymin, polyMesh.vertices[v + 2]).toInt();
      ymax = math.max(ymax, polyMesh.vertices[v + 2]).toInt();
    }

    bounds[i * 4] = math.max(0, xmin.toInt() - 1);
    bounds[i * 4 + 1] = math.min(chfWidth.toInt(), xmax.toInt() + 1);
    bounds[i * 4 + 2] = math.max(0, ymin.toInt() - 1);
    bounds[i * 4 + 3] = math.min(chfHeight.toInt(), ymax.toInt() + 1);

    if (bounds[i * 4] >= bounds[i * 4 + 1] || bounds[i * 4 + 2] >= bounds[i * 4 + 3]) continue;

    maxhw = math.max(maxhw, (bounds[i * 4 + 1] - bounds[i * 4]).toInt());
    maxhh = math.max(maxhh, (bounds[i * 4 + 3] - bounds[i * 4 + 2]).toInt());
  }

  hp.data = List<int>.filled(maxhw * maxhh, 0);

  // Mesh serialization structure instantiation
  final PolyMeshDetail dmesh = PolyMeshDetail(
    meshes: List<int>.filled(polyMeshNPolys * 4, 0),
    vertices: <double>[],
    triangles: <int>[],
    nMeshes: polyMeshNPolys,
    nVertices: 0,
    nTriangles: 0,
  );

  for (int i = 0; i < polyMeshNPolys; ++i) {
    final int p = i * nvp;
    final List<int> edges = [];
    final List<int> tris = [];
    final List<int> arr = [];
    final List<double> samples = [];
    final List<double> verts = [];

    // Store polygon vertices for processing.
    int npoly = 0;
    for (int j = 0; j < nvp; ++j) {
      if (polyMesh.polys[p + j] == meshNullIdx) break;
      final int v = (polyMesh.polys[p + j]) * 3;
      
      poly[j * 3] = (polyMesh.vertices[v]) * cs;
      poly[j * 3 + 1] = (polyMesh.vertices[v + 1]) * ch;
      poly[j * 3 + 2] = (polyMesh.vertices[v + 2]) * cs;
      npoly++;
    }

    // Get the height data from the area of the polygon.
    hp.xmin = bounds[i * 4];
    hp.ymin = bounds[i * 4 + 2];
    hp.width = bounds[i * 4 + 1] - bounds[i * 4];
    hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
    
    getHeightData(
      ctx,
      compactHeightfield,
      List<int>.from(polyMesh.polys),
      p,
      npoly,
      List<double>.from(polyMesh.vertices),
      borderSize,
      hp,
      arr,
      polyMesh.regions[i],
    );

    // Build detail mesh.
    if (!buildPolyDetail(
      ctx,
      poly,
      npoly,
      sampleDist,
      sampleMaxError,
      heightSearchRadius,
      compactHeightfield,
      hp,
      verts,
      tris,
      edges,
      samples,
    )) {
      print('buildPolyMeshDetail: Failed to build detail mesh for poly $i.');
      continue;
    }

    // Move detail verts to world space.
    for (int k = 0; k < verts.length; k += 3) {
      verts[k] += orig[0];
      verts[k + 1] += orig[1] + compactHeightfield.cellHeight;
      verts[k + 2] += orig[2];
    }

    // Offset poly too, will be used to flag checking.
    for (int j = 0; j < npoly; ++j) {
      poly[j * 3] += orig[0];
      poly[j * 3 + 1] += orig[1];
      poly[j * 3 + 2] += orig[2];
    }

    // Store detail submesh.
    final int ntris = (tris.length / 4).floor();
    final List<int> dmeshMeshes = dmesh.meshes;
    
    dmeshMeshes[i * 4] = dmesh.nVertices;
    dmeshMeshes[i * 4 + 1] = verts.length ~/ 3;
    dmeshMeshes[i * 4 + 2] = dmesh.nTriangles;
    dmeshMeshes[i * 4 + 3] = ntris;

    // Store vertices
    final List<double> dmeshVertices = dmesh.vertices;
    for (int k = 0; k < verts.length; k += 3) {
      dmeshVertices.add(verts[k]);
      dmeshVertices.add(verts[k + 1]);
      dmeshVertices.add(verts[k + 2]);
      dmesh.nVertices++;
    }

    // Store triangles
    final List<int> dmeshTriangles = dmesh.triangles;
    for (int k = 0; k < tris.length; k += 4) {
      dmeshTriangles.add(tris[k]);
      dmeshTriangles.add(tris[k + 1]);
      dmeshTriangles.add(tris[k + 2]);
      dmeshTriangles.add(tris[k + 3]);
      dmesh.nTriangles++;
    }
  }

  return dmesh;
}

class HeightPatch {
  late List<int> data;
  int xmin;
  int ymin;
  int width;
  int height;
  HeightPatch({
    List<int>? data,
    this.xmin = 0,
    this.ymin = 0,
    this.width = 0,
    this.height = 0,
  }){
    this.data = data ?? [];
  }
}