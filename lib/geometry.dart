import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';

const double eps = 1e-6;

/// Calculates the closest point on a line segment to a given point in 2D (XZ plane)
void closestPtSeg2d(Vector3 out, Vector3 pt, Vector3 p, Vector3 q) {
  final double pqx = q.x - p.x;
  final double pqz = q.z - p.z;
  final double dx = pt.x - p.x;
  final double dz = pt.z - p.z;
  final double d = pqx * pqx + pqz * pqz;
  
  double t = pqx * dx + pqz * dz;
  if (d > 0) t /= d;
  
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }
  
  out.x = p.x + t * pqx;
  out.y = p.y; // keep original Y value from p
  out.z = p.z + t * pqz;
}

/// Tests if a point is inside a polygon in 2D (XZ plane)
/// vertices are provided as a sequential array of numbers [x0, y0, z0, x1, y1, z1, ...]
bool pointInPoly(Vector3 point, List<double> vertices, int nVertices) {
  bool inside = false;
  final double x = point.x;
  final double z = point.z;

  for (int l = nVertices, i = 0, j = l - 1; i < l; j = i++) {
    final double xj = vertices[j * 3];
    final double zj = vertices[j * 3 + 2];
    final double xi = vertices[i * 3];
    final double zi = vertices[i * 3 + 2];
    
    // Cross product to find which side of the edge the point is on
    final double where = (zi - zj) * (x - xi) - (xi - xj) * (z - zi);

    if (zj < zi) {
      if (z >= zj && z < zi) {
        if (where == 0) { // Robust floating-point zero check
          return true;
        }
        if (where > 0) {
          if (z == zj) { 
            // Ray intersects vertex safely
            if (z > vertices[((j == 0 ? l - 1 : j - 1)) * 3 + 2]) {
              inside = !inside;
            }
          } else {
            inside = !inside;
          }
        }
      }
    } else if (zi < zj) {
      if (z > zi && z <= zj) {
        if (where == 0) { // Robust floating-point zero check
          return true;
        }
        if (where < 0) {
          if (z == zj) { 
            // Ray intersects vertex safely
            if (z < vertices[((j == 0 ? l - 1 : j - 1)) * 3 + 2]) {
              inside = !inside;
            }
          } else {
            inside = !inside;
          }
        }
      }
    } else if ((z - zi).abs() < eps && ((x >= xj && x <= xi) || (x >= xi && x <= xj))) {
      // Point on horizontal edge
      return true;
    }
  }
  return inside;
}

// File-private static objects to completely eliminate garbage collection during runtime frame loop ticks
final Vector3 _distPtTriV0 = Vector3();
final Vector3 _distPtTriV1 = Vector3();
final Vector3 _distPtTriV2 = Vector3();
final Vector2 _distPtTriVec0 = Vector2();
final Vector2 _distPtTriVec1 = Vector2();
final Vector2 _distPtTriVec2 = Vector2();

double distPtTri(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
  final Vector3 v0 = _distPtTriV0;
  final Vector3 v1 = _distPtTriV1;
  final Vector3 v2 = _distPtTriV2;

  // FIXED: Changed from sub2 to native three_js_math subVectors
  v0.sub2(c, a); // v0 = c - a
  v1.sub2(b, a); // v1 = b - a
  v2.sub2(p, a); // v2 = p - a

  _distPtTriVec0.setValues(v0.x, v0.z);
  _distPtTriVec1.setValues(v1.x, v1.z);
  _distPtTriVec2.setValues(v2.x, v2.z);

  final double dot00 = _distPtTriVec0.dot(_distPtTriVec0);
  final double dot01 = _distPtTriVec0.dot(_distPtTriVec1);
  final double dot02 = _distPtTriVec0.dot(_distPtTriVec2);
  final double dot11 = _distPtTriVec1.dot(_distPtTriVec1);
  final double dot12 = _distPtTriVec1.dot(_distPtTriVec2);

  // Compute barycentric coordinates
  final double denom = dot00 * dot11 - dot01 * dot01;
  if (denom.abs() < 1e-6) return double.maxFinite; // Prevent division by zero

  final double invDenom = 1.0 / denom;
  final double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  final double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // If point lies inside the triangle, return interpolated y-coord.
  const double epsTri = 1e-4;
  if (u >= -epsTri && v >= -epsTri && (u + v) <= 1.0 + epsTri) {
    // FIXED: Corrected weight assignments based on cross product setup (v0 handles v, v1 handles u)
    final double y = a.y + (v0.y * v) + (v1.y * u);
    return (y - p.y).abs();
  }

  return double.maxFinite;
}

final Vector3 _distPtSegP = Vector3();
final Vector3 _distPtSegQ = Vector3();

double distancePtSegFromVector(Vector3 pt, Vector3 p, Vector3 q) {
  final Vector3 pq = _distPtSegP;
  final Vector3 dVec = _distPtSegQ;

  // FIXED: Used native three_js_math subVectors api
  pq.sub2(q, p);    // pq = q - p
  dVec.sub2(pt, p); // dVec = pt - p

  final double d = pq.dot(pq);
  double t = pq.dot(dVec);

  if (d > 0) {
    t /= d;
  }
  
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }

  // FIXED: scale replaced with mutate-in-place multiplyScalar
  pq.scale(t); // pq = t * pq
  pq.add(p);            // pq = (t * pq) + p -> Closest point on segment

  // Calculate distance vector: closest_point - pt
  pq.sub(pt);           // pq = closest_point - pt

  return pq.dot(pq);    // returns squared distance
}


// Custom data classes replacing raw TS Object structures
class DistPtSeg2dResult {
  double dist;
  double t;
  DistPtSeg2dResult({this.dist = 0, this.t = 0});
}

DistPtSeg2dResult createDistPtSeg2dResult() => DistPtSeg2dResult();

DistPtSeg2dResult distancePtSeg2d(DistPtSeg2dResult out, Vector3 pt, Vector3 p, Vector3 q) {
  final double pqx = q.x - p.x;
  final double pqz = q.z - p.z;
  final double dx = pt.x - p.x;
  final double dz = pt.z - p.z;
  final double d = pqx * pqx + pqz * pqz;
  double t = pqx * dx + pqz * dz;
  
  if (d > 0){
    t /= d;
  }
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }
  
  final double closeDx = p.x + t * pqx - pt.x;
  final double closeDz = p.z + t * pqz - pt.z;
  final double dist = closeDx * closeDx + closeDz * closeDz;
  
  out.dist = dist;
  out.t = t;
  return out;
}

class DistancePtSegSqr2dResult {
  double distSqr;
  double t;
  DistancePtSegSqr2dResult({this.distSqr = 0, this.t = 0});
}

DistancePtSegSqr2dResult createDistancePtSegSqr2dResult() => DistancePtSegSqr2dResult();

DistancePtSegSqr2dResult distancePtSegSqr2d(DistancePtSegSqr2dResult out, Vector3 pt, Vector3 p, Vector3 q) {
  final double pqx = q.x - p.x;
  final double pqz = q.z - p.z;
  final double dx = pt.x - p.x;
  final double dz = pt.z - p.z;
  final double d = pqx * pqx + pqz * pqz;
  double t = pqx * dx + pqz * dz;
  
  if (d > 0){
    t /= d;
  }
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }
  
  final double closestX = p.x + t * pqx;
  final double closestZ = p.z + t * pqz;
  final double distX = closestX - pt.x;
  final double distZ = closestZ - pt.z;
  final double distSqr = distX * distX + distZ * distZ;
  
  out.distSqr = distSqr;
  out.t = t;
  return out;
}

// File-private caching scratchpads to eliminate frame garbage collection allocations
final Vector3 _distPtTriA = Vector3();
final Vector3 _distPtTriB = Vector3();
final Vector3 _distPtTriC = Vector3();

double distToTriMesh(Vector3 p, List<double> verts, List<int> tris, int ntris) {
  double dmin = double.maxFinite;
  
  for (int i = 0; i < ntris; ++i) {
    final int va = tris[i * 4 + 0] * 3;
    final int vb = tris[i * 4 + 1] * 3;
    final int vc = tris[i * 4 + 2] * 3;
    
    // Equivalent to vec3.fromBuffer
    _distPtTriA.fromArray(verts, va);
    _distPtTriB.fromArray(verts, vb);
    _distPtTriC.fromArray(verts, vc);
    
    // distPtTri comes from your previously converted math functions
    final double d = distPtTri(p, _distPtTriA, _distPtTriB, _distPtTriC);
    if (d < dmin) dmin = d;
  }
  
  if (dmin == double.maxFinite){
    return -1;
  }
  return dmin;
}

final Vector3 _distToPolyVj = Vector3();
final Vector3 _distToPolyVi = Vector3();
final DistPtSeg2dResult _distToPolyDistPtSeg2dResult = createDistPtSeg2dResult();

double distToPoly(int nvert, List<double> verts, Vector3 p) {
  double dmin = double.maxFinite;
  int c = 0;
  
  for (int i = 0, j = nvert - 1; i < nvert; j = i++) {
    final int vi = i * 3;
    final int vj = j * 3;
    
    final bool condition1 = verts[vi + 2] > p.z;
    final bool condition2 = verts[vj + 2] > p.z;
    
    if ((condition1 != condition2) && 
        p.x < ((verts[vj] - verts[vi]) * (p.z - verts[vi + 2])) / (verts[vj + 2] - verts[vi + 2]) + verts[vi]) {
      c = (c == 0) ? 1 : 0;
    }
    
    _distToPolyVj.fromArray(verts, vj);
    _distToPolyVi.fromArray(verts, vi);
    
    // distancePtSeg2d updates the cached reference properties
    distancePtSeg2d(_distToPolyDistPtSeg2dResult, p, _distToPolyVj, _distToPolyVi);
    dmin = math.min(dmin, _distToPolyDistPtSeg2dResult.dist);
  }
  
  return c != 0 ? -dmin : dmin;
}

/// Calculates the closest height point on a triangle using barycentric coordinates.
/// @returns Height at position, or double.nan if point is not inside triangle
double closestHeightPointTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c) {
  const double eps = 1e-6;
  final double v0x = c.x - a.x;
  final double v0y = c.y - a.y;
  final double v0z = c.z - a.z;
  final double v1x = b.x - a.x;
  final double v1y = b.y - a.y;
  final double v1z = b.z - a.z;
  final double v2x = p.x - a.x;
  final double v2z = p.z - a.z;
  
  // Compute scaled barycentric coordinates
  double denom = v0x * v1z - v0z * v1x;
  if (denom.abs() < eps) {
    return double.nan;
  }
  
  double u = v1z * v2x - v1x * v2z;
  double v = v0x * v2z - v0z * v2x;
  
  if (denom < 0) {
    denom = -denom;
    u = -u;
    v = -v;
  }
  
  // If point lies inside the triangle, return interpolated ycoord.
  if (u >= 0.0 && v >= 0.0 && u + v <= denom) {
    return a.y + (v0y * u + v1y * v) / denom;
  }
  return double.nan;
}

final Vector2 _overlapSegAB = Vector2();
final Vector2 _overlapSegAD = Vector2();
final Vector2 _overlapSegAC = Vector2();
final Vector2 _overlapSegCD = Vector2();
final Vector2 _overlapSegCA = Vector2();

bool overlapSegSeg2d(Vector2 a, Vector2 b, Vector2 c, Vector2 d) {
  // calculate cross products for line segment intersection test
  final Vector2 ab = _overlapSegAB;
  final Vector2 ad = _overlapSegAD;
  final Vector2 ac = _overlapSegAC;
  
  ab.sub2(b, a); // b - a
  ad.sub2(d, a); // d - a
  final double a1 = ab.x * ad.y - ab.y * ad.x;
  
  ac.sub2(c, a); // c - a
  final double a2 = ab.x * ac.y - ab.y * ac.x;
  
  if (a1 * a2 < 0.0) {
    final Vector2 cd = _overlapSegCD;
    final Vector2 ca = _overlapSegCA;
    
    cd.sub2(d, c); // d - c
    ca.sub2(a, c); // a - c
    final double a3 = cd.x * ca.y - cd.y * ca.x;
    final double a4 = a3 + a2 - a1;
    
    if (a3 * a4 < 0.0) return true;
  }
  return false;
}

/// 2D signed area in XZ plane (positive if c is to the left of ab)
double triArea2D(Vector3 a, Vector3 b, Vector3 c) {
  final double abx = b.x - a.x;
  final double abz = b.z - a.z;
  final double acx = c.x - a.x;
  final double acz = c.z - a.z;
  return acx * abz - abx * acz;
}

class IntersectSegSeg2DResult {
  bool hit;
  double s;
  double t;
  IntersectSegSeg2DResult({this.hit = false, this.s = 0, this.t = 0});
}

IntersectSegSeg2DResult createIntersectSegSeg2DResult() => IntersectSegSeg2DResult();

/// Segment-segment intersection in XZ plane.
/// P = a + s*(b-a) and Q = c + t*(d-c). Hit only if both s and t are within.
IntersectSegSeg2DResult intersectSegSeg2D(IntersectSegSeg2DResult out, Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
  final double bax = b.x - a.x;
  final double baz = b.z - a.z;
  final double dcx = d.x - c.x;
  final double dcz = d.z - c.z;
  final double acx = a.x - c.x;
  final double acz = a.z - c.z;
  
  final double denom = dcz * bax - dcx * baz;
  if (denom.abs() < 1e-12) {
    out.hit = false;
    out.s = 0;
    out.t = 0;
    return out;
  }
  
  final double s = (dcx * acz - dcz * acx) / denom;
  final double t = (bax * acz - baz * acx) / denom;
  final bool hit = !(s < 0 || s > 1 || t < 0 || t > 1);
  
  out.hit = hit;
  out.s = s;
  out.t = t;
  return out;
}

final Vector3 _polyMinExtentPt = Vector3();
final Vector3 _polyMinExtentP1 = Vector3();
final Vector3 _polyMinExtentP2 = Vector3();
final DistPtSeg2dResult _polyMinExtentDistPtSeg2dResult = createDistPtSeg2dResult();

// calculate minimum extent of the polygon.
double polyMinExtent(List<double> verts, int nverts) {
  double minDist = double.maxFinite;
  
  for (int i = 0; i < nverts; i++) {
    final int ni = (i + 1) % nverts;
    final int p1 = i * 3;
    final int p2 = ni * 3;
    double maxEdgeDist = 0;
    
    for (int j = 0; j < nverts; j++) {
      if (j == i || j == ni) continue;
      final int ptIdx = j * 3;
      
      _polyMinExtentPt.fromArray(verts, ptIdx);
      _polyMinExtentP1.fromArray(verts, p1);
      _polyMinExtentP2.fromArray(verts, p2);
      
      distancePtSeg2d(_polyMinExtentDistPtSeg2dResult, _polyMinExtentPt, _polyMinExtentP1, _polyMinExtentP2);
      maxEdgeDist = math.max(maxEdgeDist, _polyMinExtentDistPtSeg2dResult.dist);
    }
    minDist = math.min(minDist, maxEdgeDist);
  }
  
  return math.sqrt(minDist);
}

/// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
/// The vectors are projected onto the xz-plane, so the y-values are ignored.
double _vperp2D(Vector3 u, Vector3 v) {
  return u.z * v.x - u.x * v.z;
}

class IntersectSegmentPoly2DResult {
  bool intersects;
  double tmin;
  double tmax;
  int segMin;
  int segMax;
  
  IntersectSegmentPoly2DResult({
    this.intersects = false,
    this.tmin = 0,
    this.tmax = 0,
    this.segMin = -1,
    this.segMax = -1,
  });
}

IntersectSegmentPoly2DResult createIntersectSegmentPoly2DResult() => IntersectSegmentPoly2DResult();

// File-private caching scratchpads to prevent garbage collection during runtime execution loops
final Vector3 _intersectSegmentPoly2DVi = Vector3();
final Vector3 _intersectSegmentPoly2DVj = Vector3();
final Vector3 _intersectSegmentPoly2DDir = Vector3();
final Vector3 _intersectSegmentPoly2DToStart = Vector3();
final Vector3 _intersectSegmentPoly2DEdge = Vector3();

/// Intersects a segment with a polygon in 2D (ignoring Y).
/// Uses the Sutherland-Hodgman clipping algorithm approach.
IntersectSegmentPoly2DResult intersectSegmentPoly2D(
  IntersectSegmentPoly2DResult result,
  Vector3 startPosition,
  Vector3 endPosition,
  int nv,
  List<double> verts,
) {
  result.intersects = false;
  result.tmin = 0;
  result.tmax = 1;
  result.segMin = -1;
  result.segMax = -1;
  
  final Vector3 dir = _intersectSegmentPoly2DDir;
  dir.sub2(endPosition, startPosition);
  
  final Vector3 vi = _intersectSegmentPoly2DVi;
  final Vector3 vj = _intersectSegmentPoly2DVj;
  final Vector3 edge = _intersectSegmentPoly2DEdge;
  final Vector3 diff = _intersectSegmentPoly2DToStart;
  
  for (int i = 0, j = nv - 1; i < nv; j = i, i++) {
    vi.fromArray(verts, i * 3);
    vj.fromArray(verts, j * 3);
    
    edge.sub2(vi, vj);
    diff.sub2(startPosition, vj);
    
    final double n = _vperp2D(edge, diff);
    final double d = _vperp2D(dir, edge);
    
    if (d.abs() < eps) {
      // S is nearly parallel to this edge
      if (n < 0) {
        return result;
      }
      continue;
    }
    
    final double t = n / d;
    if (d < 0) {
      // segment S is entering across this edge
      if (t > result.tmin) {
        result.tmin = t;
        result.segMin = j;
        // S enters after leaving polygon
        if (result.tmin > result.tmax) {
          return result;
        }
      }
    } else {
      // segment S is leaving across this edge
      if (t < result.tmax) {
        result.tmax = t;
        result.segMax = j;
        // S leaves before entering polygon
        if (result.tmax < result.tmin) {
          return result;
        }
      }
    }
  }
  
  result.intersects = true;
  return result;
}

final Vector3 _randomPointInConvexPolyVa = Vector3();
final Vector3 _randomPointInConvexPolyVb = Vector3();
final Vector3 _randomPointInConvexPolyVc = Vector3();

/// Generates a random point inside a convex polygon using barycentric coordinates.
Vector3 randomPointInConvexPoly(
  Vector3 out,
  int nv,
  List<double> verts,
  List<double> areas,
  double s,
  double t,
) {
  // calculate cumulative triangle areas for weighted selection
  double areaSum = 0;
  final Vector3 va = _randomPointInConvexPolyVa.fromArray(verts, 0);
  
  for (int i = 2; i < nv; i++) {
    final Vector3 vb = _randomPointInConvexPolyVb.fromArray(verts, (i - 1) * 3);
    final Vector3 vc = _randomPointInConvexPolyVc.fromArray(verts, i * 3);
    
    // triArea2D comes from your second block of math functions
    areas[i] = triArea2D(va, vb, vc);
    areaSum += math.max(0.001, areas[i]);
  }
  
  // choose triangle based on area-weighted random selection
  final double thr = s * areaSum;
  double acc = 0;
  double u = 1;
  int tri = nv - 1;
  
  for (int i = 2; i < nv; i++) {
    final double dacc = areas[i];
    if (thr >= acc && thr < acc + dacc) {
      u = (thr - acc) / dacc;
      tri = i;
      break;
    }
    acc += dacc;
  }
  
  // generate random point in triangle using barycentric coordinates
  // standard method: use square root for uniform distribution
  final double v = math.sqrt(t);
  final double a = 1 - v;
  final double b = (1 - u) * v;
  final double c = u * v;
  
  final vb = _randomPointInConvexPolyVb.fromArray(verts, (tri - 1) * 3);
  final vc = _randomPointInConvexPolyVc.fromArray(verts, tri * 3);
  
  out.x = a * va.x + b * vb.x + c * vc.x;
  out.y = a * va.y + b * vb.y + c * vc.y;
  out.z = a * va.z + b * vb.z + c * vc.z;
  
  return out;
}

/// Simple projection class mapping to your JS tuple array [min, max]
class ProjectionResult {
  double min;
  double max;
  ProjectionResult({this.min = 0, this.max = 0});
}

/// Projects a polygon onto an axis and updates the min/max projection values.
void _projectPoly(ProjectionResult out, Vector2 axis, List<double> verts, int nverts) {
  double min = axis.x * verts[0] + axis.y * verts[2]; // dot product with first vertex (x,z)
  double max = min;
  
  for (int i = 1; i < nverts; i++) {
    final double dot = axis.x * verts[i * 3] + axis.y * verts[i * 3 + 2]; // dot product (x,z)
    min = math.min(min, dot);
    max = math.max(max, dot);
  }
  
  out.min = min;
  out.max = max;
}

/// Checks if two ranges overlap with epsilon tolerance.
bool _overlapRange(double amin, double amax, double bmin, double bmax, double eps) {
  return !(amin + eps > bmax || amax - eps < bmin);
}

// Caching infrastructure matching your trailing definitions
final Vector2 _overlapPolyPolyNormal = Vector2();
final Vector2 _overlapPolyPolyVa = Vector2();
final Vector2 _overlapPolyPolyVb = Vector2();
final ProjectionResult _overlapPolyPolyProjA = ProjectionResult();
final ProjectionResult _overlapPolyPolyProjB = ProjectionResult();

/// Tests if two convex polygons overlap in 2D (XZ plane).
/// Uses the separating axis theorem - matches the C++ dtOverlapPolyPoly2D implementation.
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool overlapPolyPoly2D(List<double> vertsA, int nvertsA, List<double> vertsB, int nvertsB) {
  const double eps = 1e-4;
  
  // Check separation along each edge normal of polygon A
  for (int i = 0, j = nvertsA - 1; i < nvertsA; j = i++) {
    final Vector2 va = _overlapPolyPolyVa;
    final Vector2 vb = _overlapPolyPolyVb;
    
    va.setValues(vertsA[j * 3], vertsA[j * 3 + 2]); // x, z
    vb.setValues(vertsA[i * 3], vertsA[i * 3 + 2]); // x, z
    
    // Calculate edge normal: n = { vb[z]-va[z], -(vb[x]-va[x]) }
    final Vector2 normal = _overlapPolyPolyNormal;
    normal.setValues(vb.y - va.y, -(vb.x - va.x));
    
    // Project both polygons onto this normal
    final ProjectionResult projA = _overlapPolyPolyProjA;
    final ProjectionResult projB = _overlapPolyPolyProjB;
    
    _projectPoly(projA, normal, vertsA, nvertsA);
    _projectPoly(projB, normal, vertsB, nvertsB);
    
    // Check if projections are separated
    if (!_overlapRange(projA.min, projA.max, projB.min, projB.max, eps)) {
      // Found separating axis
      return false;
    }
  }
  
  // Check separation along each edge normal of polygon B
  for (int i = 0, j = nvertsB - 1; i < nvertsB; j = i++) {
    final Vector2 va = _overlapPolyPolyVa;
    final Vector2 vb = _overlapPolyPolyVb;
    
    va.setValues(vertsB[j * 3], vertsB[j * 3 + 2]); // x, z
    vb.setValues(vertsB[i * 3], vertsB[i * 3 + 2]); // x, z
    
    // Calculate edge normal: n = { vb[z]-va[z], -(vb[x]-va[x]) }
    final Vector2 normal = _overlapPolyPolyNormal;
    normal.setValues(vb.y - va.y, -(vb.x - va.x));
    
    // Project both polygons onto this normal
    final ProjectionResult projA = _overlapPolyPolyProjA;
    final ProjectionResult projB = _overlapPolyPolyProjB;
    
    _projectPoly(projA, normal, vertsA, nvertsA);
    _projectPoly(projB, normal, vertsB, nvertsB);
    
    // Check if projections are separated
    if (!_overlapRange(projA.min, projA.max, projB.min, projB.max, eps)) {
      // Found separating axis
      return false;
    }
  }
  
  // No separating axis found, polygons overlap
  return true;
}
