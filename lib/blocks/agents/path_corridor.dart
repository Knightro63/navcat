import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';
import '../../navcat.dart';

class PathCorridor {
  final Vector3 position;
  final Vector3 target;
  List<int> path; // Using int for NodeRef

  PathCorridor({
    Vector3? position,
    Vector3? target,
    List<int>? path,
  })  : this.position = position ?? Vector3(0, 0, 0),
        this.target = target ?? Vector3(0, 0, 0),
        this.path = path ?? [];

  /// Resets the corridor data.
  void reset(int ref, Vector3 position) {
    this.position.setFrom(position);
    this.target.setFrom(position);
    this.path = [ref];
  }

  /// Sets the corridor path and target.
  void setPath(Vector3 target, List<int> path) {
    this.target.setFrom(target);
    this.path = path;
  }
}

List<int> mergeStartMoved(List<int> currentPath, List<int> visited) {
  if (visited.isEmpty) return currentPath;

  int furthestPath = -1;
  int furthestVisited = -1;

  // Find furthest common polygon
  for (int i = currentPath.length - 1; i >= 0; i--) {
    for (int j = visited.length - 1; j >= 0; j--) {
      if (currentPath[i] == visited[j]) {
        furthestPath = i;
        furthestVisited = j;
        break;
      }
    }
    if (furthestPath != -1) break;
  }

  // If no intersection found, just return current path
  if (furthestPath == -1 || furthestVisited == -1) {
    return currentPath;
  }

  // Concatenate paths
  final int req = visited.length - furthestVisited;
  final int orig = math.min(furthestPath + 1, currentPath.length);
  final int size = math.max(0, currentPath.length - orig);
  
  // Initialize sizing explicitly for index assignment
  final List<int> newPath = List<int>.filled(req + size, 0);

  // Store visited polygons (in reverse order)
  for (int i = 0; i < req; i++) {
    newPath[i] = visited[visited.length - 1 - i];
  }

  // Add remaining current path
  if (size > 0) {
    for (int i = 0; i < size; i++) {
      newPath[req + i] = currentPath[orig + i];
    }
  }

  return newPath;
}

List<int> mergeStartShortcut(List<int> currentPath, List<int> visited) {
  if (visited.isEmpty) return currentPath;

  int furthestPath = -1;
  int furthestVisited = -1;

  // Find furthest common polygon
  for (int i = currentPath.length - 1; i >= 0; i--) {
    for (int j = visited.length - 1; j >= 0; j--) {
      if (currentPath[i] == visited[j]) {
        furthestPath = i;
        furthestVisited = j;
        break;
      }
    }
    if (furthestPath != -1) break;
  }

  // If no intersection found, just return current path
  if (furthestPath == -1 || furthestVisited == -1) {
    return currentPath;
  }

  // Concatenate paths
  final int req = furthestVisited;
  if (req <= 0) {
    return currentPath;
  }

  final int orig = furthestPath;
  final int size = math.max(0, currentPath.length - orig);
  final List<int> newPath = List<int>.filled(req + size, 0);

  // Store visited polygons (not reversed)
  for (int i = 0; i < req; i++) {
    newPath[i] = visited[i];
  }

  // Add remaining current path
  if (size > 0) {
    for (int i = 0; i < size; i++) {
      newPath[req + i] = currentPath[orig + i];
    }
  }

  return newPath;
}

bool movePosition(PathCorridor corridor, Vector3 newPos, NavMesh navMesh, QueryFilter filter) {
  if (corridor.path.isEmpty) return false;

  final result = moveAlongSurface(navMesh, corridor.path[0], corridor.position, newPos, filter);
  if (result.success) {
    corridor.path = mergeStartMoved(corridor.path, result.visited);
    corridor.position.setFrom(result.position);
    return true;
  }
  return false;
}

const double minTargetDist = 0.01;

List<StraightPathPoint>? findCorners(
  PathCorridor corridor,
  NavMesh navMesh,
  int maxCorners,
) {
  if (corridor.path.isEmpty) return null; // Using null in Dart instead of false for object paths

  final straightPathResult = findStraightPath(navMesh, corridor.position, corridor.target, corridor.path, maxCorners);
  if (!straightPathResult.success || straightPathResult.path.isEmpty) {
    return null;
  }

  List<StraightPathPoint> corners = straightPathResult.path;

  // Prune points in the beginning of the path which are too close
  while (corners.isNotEmpty) {
    final firstCorner = corners[0];
    final distance = corridor.position.distanceTo(firstCorner.position);

    // If the first corner is far enough, we're done pruning
    if (firstCorner.type == NodeType.offMesh.value || distance > minTargetDist) {
      break;
    }
    // Remove the first corner as it's too close
    corners = corners.sublist(1);
  }

  // Prune points after an offmesh connection
  int firstOffMeshConnectionIndex = -1;
  for (int i = 0; i < corners.length; i++) {
    if (corners[i].type == NodeType.offMesh.value) {
      firstOffMeshConnectionIndex = i;
      break;
    }
  }

  if (firstOffMeshConnectionIndex != -1) {
    corners = corners.sublist(0, firstOffMeshConnectionIndex + 1);
  }

  return corners;
}

bool corridorIsValid(PathCorridor corridor, int maxLookAhead, NavMesh navMesh, QueryFilter filter) {
  final n = math.min(corridor.path.length, maxLookAhead);

  // Check nodes are still valid and pass query filter
  for (int i = 0; i < n; i++) {
    final nodeRef = corridor.path[i];
    if (!isValidNodeRef(navMesh, nodeRef) || !filter.passFilter(nodeRef, navMesh)) {
      return false;
    }
  }
  return true;
}

bool fixPathStart(PathCorridor corridor, int safeRef, Vector3 safePos) {
  corridor.position.setFrom(safePos);
  
  if (corridor.path.length < 3 && corridor.path.isNotEmpty) {
    final lastPoly = corridor.path[corridor.path.length - 1];
    
    // Explicitly rebuild/expand structural path to 3 items
    corridor.path = [safeRef, invalidNodeRef, lastPoly];
  } else {
    if (corridor.path.isEmpty) {
      corridor.path = [safeRef, invalidNodeRef];
    } else {
      corridor.path[0] = safeRef;
      corridor.path[1] = invalidNodeRef;
    }
  }
  return true;
}

/// Container for the successful result of moving over an off-mesh connection.
class MoveOffMeshResult {
  final Vector3 startPosition;
  final Vector3 endPosition;
  final int endNodeRef;
  final int prevNodeRef;
  final int offMeshNodeRef;

  MoveOffMeshResult({
    required this.startPosition,
    required this.endPosition,
    required this.endNodeRef,
    required this.prevNodeRef,
    required this.offMeshNodeRef,
  });
}

MoveOffMeshResult? moveOverOffMeshConnection(
  PathCorridor corridor, 
  int offMeshNodeRef, 
  NavMesh navMesh,
) {
  if (corridor.path.isEmpty) return null;

  // Advance the path up to and over the off-mesh connection.
  int? prevNodeRef;
  int nodeRef = corridor.path[0];
  int i = 0;

  while (i < corridor.path.length && nodeRef != offMeshNodeRef) {
    prevNodeRef = nodeRef;
    i++;
    if (i < corridor.path.length) {
      nodeRef = corridor.path[i];
    }
  }

  if (i == corridor.path.length) {
    // Could not find the off-mesh connection node
    return null;
  }

  // Prune path - remove the elements from 0 up to and including the off-mesh connection
  corridor.path = corridor.path.sublist(i + 1);

  if (prevNodeRef == null) {
    return null;
  }

  // Get the off-mesh connection details
  final nodeData = getNodeByRef(navMesh, offMeshNodeRef);
  final offMeshConnectionId = nodeData?.offMeshConnectionId;
  
  final offMeshConnection = navMesh.offMeshConnections[offMeshConnectionId];
  final offMeshConnectionAttachment = navMesh.offMeshConnectionAttachments[offMeshConnectionId];

  if (offMeshConnection == null || offMeshConnectionAttachment == null) {
    return null;
  }

  // Determine which end we're moving to
  final bool onStart = offMeshConnectionAttachment.startPolyNode == prevNodeRef;
  final Vector3 endPosition = onStart ? offMeshConnection.end : offMeshConnection.start;
  final int endNodeRef = onStart 
      ? offMeshConnectionAttachment.endPolyNode 
      : offMeshConnectionAttachment.startPolyNode;

  corridor.position.setFrom(endPosition);

  return MoveOffMeshResult(
    startPosition: onStart ? offMeshConnection.start : offMeshConnection.end,
    endPosition: endPosition,
    endNodeRef: endNodeRef,
    prevNodeRef: prevNodeRef,
    offMeshNodeRef: offMeshNodeRef,
  );
}

/**
 * Attempts to optimize the path using a local area search (partial replanning).
 */
bool optimizePathTopology(PathCorridor corridor, NavMesh navMesh, QueryFilter filter) {
  if (corridor.path.length < 3) {
    return false;
  }

  const int maxIter = 32;
  final query = createSlicedNodePathQuery();

  // Do a local area search from start to end
  initSlicedFindNodePath(
    navMesh,
    query,
    corridor.path.first,
    corridor.path.last,
    corridor.position,
    corridor.target,
    filter,
  );

  updateSlicedFindNodePath(navMesh, query, maxIter);
  
  final result = finalizeSlicedFindNodePathPartial(navMesh, query, corridor.path);

  if ((result.status & SlicedFindNodePathStatusFlags.success) != 0 && result.path.isNotEmpty) {
    // Merge the optimized path with the corridor using shortcut merge
    corridor.path = mergeStartShortcut(corridor.path, result.path);
    return true;
  }

  return false;
}

// Internal reusable vector allocation states to avoid constant GC pressure
final Vector3 _optimizePathVisibilityGoal = Vector3();
final _optimizePathVisibilityDelta = Vector3();

/**
 * Attempts to optimize the path if the specified point is visible from the current position.
 */
void optimizePathVisibility(
  PathCorridor corridor,
  Vector3 next,
  double pathOptimizationRange,
  NavMesh navMesh,
  QueryFilter filter,
) {
  if (corridor.path.isEmpty) {
    return;
  }

  // Clamp the ray to max distance
  final goal = _optimizePathVisibilityGoal.setFrom(next);
  final double dx = goal.x - corridor.position.x;
  final double dz = goal.z - corridor.position.z;
  double dist = math.sqrt(dx * dx + dz * dz);

  // If too close to the goal, do not try to optimize.
  if (dist < 0.01) {
    return;
  }

  // Overshoot a little. This helps to optimize open fields in tiled meshes.
  dist = math.min(dist + 0.01, pathOptimizationRange);

  // Adjust ray length using operator overloading vector mechanics
  final delta = _optimizePathVisibilityDelta.sub2(goal,corridor.position).scale((pathOptimizationRange / dist));
  goal.add2(corridor.position, delta);

  final result = raycast(navMesh, corridor.path[0], corridor.position, goal, filter);

  if (result.path.length > 1 && result.t > 0.99) {
    corridor.path = mergeStartShortcut(corridor.path, result.path);
  }
}