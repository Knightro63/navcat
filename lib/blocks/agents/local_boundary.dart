import 'package:three_js_math/three_js_math.dart'; 
import '../../navcat.dart'; 

const int maxLocalSegs = 8; 
const int maxLocalPolys = 16; 

class LocalBoundarySegment { 
  final List<double> s; 
  double d; 

  LocalBoundarySegment({ 
    required this.s, 
    this.d = 0, 
  }); 
} 

/** 
 * Local boundary data for avoiding collisions with nearby walls. 
 */ 
class LocalBoundary { 
  final Vector3 center; 
  List<LocalBoundarySegment> segments; 
  List<int> polys; 

  LocalBoundary({ 
    Vector3? center, 
    List<LocalBoundarySegment>? segments, 
    List<int>? polys, 
  })  : this.center = center ?? Vector3(), 
        this.segments = segments ?? [], 
        this.polys = polys ?? []; 

  factory LocalBoundary.create() { 
    return LocalBoundary(
      center: Vector3(double.infinity, double.infinity, double.infinity),
    ); 
  } 
} 

/** 
 * Resets the boundary data. 
 */ 
void resetLocalBoundary(LocalBoundary boundary) { 
  boundary.center.setValues(double.infinity, double.infinity, double.infinity); 
  boundary.segments.clear(); 
  boundary.polys.clear(); 
} 

/** 
 * Calculates distance squared from point to line segment in 2D (XZ plane). 
 */ 
double distancePtSegSqr2d(Vector3 pt, Vector3 segStart, Vector3 segEnd) { 
  final pqx = segEnd.x - segStart.x; 
  final pqz = segEnd.z - segStart.z; 
  final dx = pt.x - segStart.x; 
  final dz = pt.z - segStart.z; 
  final d = pqx * pqx + pqz * pqz; 
  
  double t = pqx * dx + pqz * dz; 
  if (d > 0) t /= d; 
  if (t < 0) t = 0; 
  else if (t > 1) t = 1; 

  final nearestX = segStart.x + t * pqx; 
  final nearestZ = segStart.z + t * pqz; 
  final distX = pt.x - nearestX; 
  final distZ = pt.z - nearestZ; 
  
  return distX * distX + distZ * distZ; 
} 

/** 
 * Adds a wall segment to the boundary, sorted by distance. 
 */ 
void addSegmentToBoundary(LocalBoundary boundary, double dist, List<double> s) { 
  // Find insertion point based on distance 
  int insertIdx = 0; 
  for (int i = 0; i < boundary.segments.length; i++) { 
    if (dist <= boundary.segments[i].d) { 
      insertIdx = i; 
      break; 
    } 
    insertIdx = i + 1; 
  } 

  // Don't exceed max segments 
  if (boundary.segments.length >= maxLocalSegs) { 
    // If we're trying to insert past the end, skip 
    if (insertIdx >= maxLocalSegs) return; 
    // Remove last segment to make room 
    boundary.segments.removeLast(); 
  } 

  // Create new segment 
  final segment = LocalBoundarySegment( 
    d: dist, 
    s: [s[0], s[1], s[2], s[3], s[4], s[5]], 
  ); 

  // Insert at the correct position 
  boundary.segments.insert(insertIdx, segment); 
} 

/** 
 * Updates the local boundary data around the given position. 
 */ 
void updateLocalBoundary( 
  LocalBoundary boundary, 
  int nodeRef, 
  Vector3 position, 
  double collisionQueryRange, 
  NavMesh navMesh, 
  QueryFilter filter, 
) { 
  if (!isValidNodeRef(navMesh, nodeRef)) { 
    resetLocalBoundary(boundary); 
    return; 
  } 

  boundary.center.setFrom(position); 

  // First query non-overlapping polygons 
  final neighbourhoodResult = findLocalNeighbourhood(
    navMesh, nodeRef, position, collisionQueryRange, filter,
  ); 
  
  if (!neighbourhoodResult.success) { 
    boundary.segments.clear(); 
    boundary.polys.clear(); 
    return; 
  } 

  // Store found polygons (limit to max using sublist instead of slice) 
  final limit = neighbourhoodResult.nodeRefs.length < maxLocalPolys 
      ? neighbourhoodResult.nodeRefs.length 
      : maxLocalPolys;
  boundary.polys = neighbourhoodResult.nodeRefs.sublist(0, limit); 

  // Clear existing segments 
  boundary.segments.clear(); 

  // Store all polygon wall segments 
  final collisionQueryRangeSqr = collisionQueryRange * collisionQueryRange; 
  
  for (final polyRef in boundary.polys) { 
    final wallSegmentsResult = getPolyWallSegments(navMesh, polyRef, filter, false); 
    if (!wallSegmentsResult.success) continue; 

    final segmentCount = wallSegmentsResult.segmentVerts.length ~/ 6; 
    for (int k = 0; k < segmentCount; ++k) { 
      final segStart = k * 6; 
      final s = wallSegmentsResult.segmentVerts.sublist(segStart, segStart + 6); 

      // Skip distant segments 
      final segmentStart = Vector3(
        s[0].toDouble(), s[1].toDouble(), s[2].toDouble(),
      ); 
      final segmentEnd = Vector3(
        s[3].toDouble(), s[4].toDouble(), s[5].toDouble(),
      ); 
      
      final distSqr = distancePtSegSqr2d(position, segmentStart, segmentEnd); 
      if (distSqr > collisionQueryRangeSqr) { 
        continue; 
      } 
      
      addSegmentToBoundary(boundary, distSqr, s); 
    } 
  } 
} 

/** 
 * Checks if the boundary data is still valid. 
 */ 
bool isLocalBoundaryValid(LocalBoundary boundary, NavMesh navMesh, QueryFilter filter) { 
  if (boundary.polys.isEmpty) { 
    return false; 
  } 

  // Check that all polygons still pass query filter 
  for (final polyRef in boundary.polys) { 
    if (!isValidNodeRef(navMesh, polyRef)) { 
      return false; 
    } 
    // Check filter if available 
    if (!filter.passFilter(polyRef, navMesh)) { 
      return false; 
    } 
  } 
  
  return true; 
}
