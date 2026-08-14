import 'package:three_js_math/three_js_math.dart';
import 'index.dart';

enum FindPathResultFlags {
  none(0),
  success(1 << 0),
  completePath(1 << 1),
  partialPath(1 << 2),
  maxPointsReached(1 << 3),
  invalidInput(1 << 4),
  findNodePathFailed(1 << 5),
  findStraightPathFailed(1 << 6);

  final int value;
  const FindPathResultFlags(this.value);

  // Helper utility method to perform quick flag checks
  bool isSetIn(int bitmask) => (bitmask & value) != 0;
}

class FindPathResult {
  /// whether the search completed successfully, with either a partial or complete path
  bool success;

  /// the status flags of the pathfinding operation
  late int flags;

  /// the path, consisting of polygon node and offmesh link node references
  List<StraightPathPoint>? path;

  /// the status flags of the straight pathfinding operation
  late int straightPathFlags;

  /// the start poly node ref
  int? startNodeRef;

  /// the start closest point
  late Vector3 startPosition;

  /// the end poly node ref
  int? endNodeRef;

  /// the end closest point
  late Vector3 endPosition;

  /// the node path result
  FindNodePathResult? nodePath;

  /// the status flags of the node pathfinding operation
  late int nodePathFlags;

  FindPathResult({
    this.success = false,
    int? flags,
    this.path = const [],
    int? straightPathFlags,
    this.startNodeRef,
    Vector3? startPosition,
    this.endNodeRef,
    Vector3? endPosition,
    this.nodePath,
    int? nodePathFlags,
  }){
    this.startPosition = startPosition ?? Vector3();
    this.endPosition = endPosition ?? Vector3();
    this.flags = flags ?? FindPathResultFlags.none.value;
    this.straightPathFlags = straightPathFlags ?? FindStraightPathResultFlags.none.value;
    this.nodePathFlags = nodePathFlags ?? FindNodePathResultFlags.none.value;
  }
}

final _findPathStartNearestPolyResult = createFindNearestPolyResult();
final _findPathEndNearestPolyResult = createFindNearestPolyResult();

/// Find a path between two positions on a NavMesh.
///
/// If the end node cannot be reached through the navigation graph,
/// the last node in the path will be the nearest the end node.
///
/// Internally:
/// - finds the closest poly for the start and end positions with @see findNearestPoly
/// - finds a nav mesh node path with @see findNodePath
/// - finds a straight path with @see findStraightPath
///
/// If you want more fine tuned behaviour you can call these methods directly.
/// For example, for agent movement you might want to find a node path once but regularly re-call @see findStraightPath
///
/// @param navMesh The navigation mesh.
/// @param start The starting position in world space.
/// @param end The ending position in world space.
/// @param queryFilter The query filter.
/// @returns The result of the pathfinding operation.
FindPathResult findPath(
  NavMesh navMesh,
  Vector3 start,
  Vector3 end,
  Vector3 halfExtents,
  QueryFilter queryFilter, [
  Map<String, dynamic>? options,
]) {
  // Corrected the parameters from JS array literals to proper Dart type targets
  final result = FindPathResult(
    success: false,
    flags: FindPathResultFlags.none.value | FindPathResultFlags.invalidInput.value,
    nodePathFlags: FindNodePathResultFlags.none.value,
    straightPathFlags: FindStraightPathResultFlags.none.value,
    path: [],
    startNodeRef: null,
    startPosition: Vector3(),
    endNodeRef: null,
    endPosition: Vector3(),
    nodePath: null,
  );

  /* find start nearest poly */
  final startNearestPolyResult = findNearestPoly(
    _findPathStartNearestPolyResult, 
    navMesh, 
    start, 
    halfExtents, 
    queryFilter,
  );
  if (!startNearestPolyResult.success){
    return result;
  }
  
  // Utilizes standard three_js_math .copy property updates
  result.startPosition.setFrom(startNearestPolyResult.position);
  result.startNodeRef = startNearestPolyResult.nodeRef;

  /* find end nearest poly */
  final endNearestPolyResult = findNearestPoly(
    _findPathEndNearestPolyResult, 
    navMesh, 
    end, 
    halfExtents, 
    queryFilter,
  );
  if (!endNearestPolyResult.success){
    return result;
  }
  
  result.endPosition.setFrom(endNearestPolyResult.position);
  result.endNodeRef = endNearestPolyResult.nodeRef;

  /* find node path corridor */
  final double? raycastDistance = options?['raycastDistance'] as double?;
  final nodePath = findNodePath(
    navMesh,
    result.startNodeRef!,
    result.endNodeRef!,
    result.startPosition,
    result.endPosition,
    queryFilter,
    raycastDistance != null ? {'raycastDistance': raycastDistance} : null,
  );
  
  result.nodePath = nodePath;
  result.nodePathFlags = nodePath.flags;

  if (!nodePath.success) {
    result.flags = FindPathResultFlags.findNodePathFailed.value;
    return result;
  }

  /* find straight path string-pulling funnel */
  final straightPath = findStraightPath(
    navMesh, 
    result.startPosition, 
    result.endPosition, 
    nodePath.path,
  );
  
  if (!straightPath.success) {
    result.flags = FindPathResultFlags.findStraightPathFailed.value;
    return result;
  }

  result.success = true;
  result.path = straightPath.path;
  result.straightPathFlags = straightPath.flags;

  int flags = FindPathResultFlags.success.value;

  if ((nodePath.flags & FindNodePathResultFlags.completePath.value) != 0 && 
      (straightPath.flags & FindStraightPathResultFlags.partialPath.value) == 0
  ) {
    flags |= FindPathResultFlags.completePath.value;
  } else {
    flags |= FindPathResultFlags.partialPath.value;
  }

  if ((straightPath.flags & FindStraightPathResultFlags.maxPointsReached.value) != 0) {
    flags |= FindPathResultFlags.maxPointsReached.value;
  }

  result.flags = flags;
  return result;
}
