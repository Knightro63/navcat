import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';
import 'index.dart';

final Vector3 _findSmoothPathDelta = Vector3();
final Vector3 _findSmoothPathMoveTarget = Vector3();

final _findSmoothPathStartNearestPolyResult = createFindNearestPolyResult();
final _findSmoothPathEndNearestPolyResult = createFindNearestPolyResult();

enum FindSmoothPathResultFlags {
  none(0),
  success(1 << 0),
  completePath(1 << 1),
  partialPath(1 << 2),
  invalidPath(1 << 3),
  findNodePathFailed(1 << 4),
  findStraightPathFailed(1 << 5);

  final int value;
  const FindSmoothPathResultFlags(this.value);
}

enum SmoothPathPointFlags {
  start(0),
  end(1),
  offMesh(2);

  final int value;
  const SmoothPathPointFlags(this.value);
}

class SmoothPathPoint {
  final Vector3 position;
  final NodeType type;
  final int? nodeRef;
  final int flags;

  SmoothPathPoint({
    required this.position,
    required this.type,
    this.nodeRef,
    required this.flags,
  });
}

class FindSmoothPathResult {
  /// whether the search completed successfully
  bool success;

  /// the status flags of the smooth pathfinding operation
  FindSmoothPathResultFlags flags;

  /// the path points
  final List<SmoothPathPoint> path;

  /// the start poly node ref
  int? startNodeRef;

  /// the start closest point
  final Vector3 startPosition;

  /// the end poly node ref
  int? endNodeRef;

  /// the end closest point
  final Vector3 endPosition;

  /// the node path result
  FindNodePathResult? nodePath;

  FindSmoothPathResult({
    required this.success,
    required this.flags,
    required this.path,
    this.startNodeRef,
    required this.startPosition,
    this.endNodeRef,
    required this.endPosition,
    this.nodePath,
  });
}

/// Find a smooth path between two positions on a NavMesh.
///
/// This method computes a smooth path by iteratively moving along the navigation
/// mesh surface using the polygon path found between start and end positions.
/// The resulting path follows the surface more naturally than a straight path.
///
/// If the end node cannot be reached through the navigation graph,
/// the path will go as far as possible toward the target.
///
/// Internally:
/// - finds the closest poly for the start and end positions with @see findNearestPoly
/// - finds a nav mesh node path with @see findNodePath
/// - computes a smooth path by iteratively moving along the surface with @see moveAlongSurface
///
/// @param navMesh The navigation mesh.
/// @param start The starting position in world space.
/// @param end The ending position in world space.
/// @param halfExtents The half extents for nearest polygon queries.
/// @param queryFilter The query filter.
/// @param stepSize The step size for movement along the surface
/// @param slop The distance tolerance for reaching waypoints
/// @returns The result of the smooth pathfinding operation, with path points containing position, type, and nodeRef information.
///
FindSmoothPathResult findSmoothPath(
  NavMesh navMesh,
  Vector3 start,
  Vector3 end,
  Vector3 halfExtents,
  QueryFilter queryFilter,
  Map<String, dynamic> options,
) {
  final double stepSize = options['stepSize'] as double;
  final double slop = options['slop'] as double;
  final int maxPoints = options['maxPoints'] as int;
  //final double? raycastDistance = options['raycastDistance'] as double?;

  final result = FindSmoothPathResult(
    success: false,
    flags: FindSmoothPathResultFlags.values[ FindSmoothPathResultFlags.none.value | FindSmoothPathResultFlags.invalidPath.value],
    path: [],
    startNodeRef: null,
    startPosition: Vector3(),
    endNodeRef: null,
    endPosition: Vector3(),
    nodePath: null,
  );

  /* find start nearest poly */
  final startNearestPolyResult = findNearestPoly(
    _findSmoothPathStartNearestPolyResult,
    navMesh,
    start,
    halfExtents,
    queryFilter,
  );
  if (!startNearestPolyResult.success){ 
    return result;
  }

  result.startPosition.setFrom(startNearestPolyResult.position);
  result.startNodeRef = startNearestPolyResult.nodeRef;

  /* find end nearest poly */
  final endNearestPolyResult = findNearestPoly(_findSmoothPathEndNearestPolyResult, navMesh, end, halfExtents, queryFilter);
  if (!endNearestPolyResult.success){ 
    return result;
  }

  result.endPosition.setFrom(endNearestPolyResult.position);
  result.endNodeRef = endNearestPolyResult.nodeRef;

  /* find node path */
  final nodePath = findNodePath(
    navMesh,
    result.startNodeRef!,
    result.endNodeRef!,
    result.startPosition,
    result.endPosition,
    queryFilter,
  );

  result.nodePath = nodePath;

  if (!nodePath.success || nodePath.path.length == 0) {
    result.flags = FindSmoothPathResultFlags.findNodePathFailed;
    return result;
  }

  // iterate over the path to find a smooth path
  final iterPos = result.startPosition.clone();
  final targetPos = result.endPosition.clone();

  List<int> polys = [...nodePath.path];

  result.path.add(SmoothPathPoint(
      position: iterPos.clone(),
      type: NodeType.poly,
      nodeRef: result.startNodeRef,
      flags: SmoothPathPointFlags.start.value,
  ));

  while (polys.isNotEmpty && result.path.length < maxPoints) {
    // find location to steer towards
    final steerTarget = getSteerTarget(navMesh, iterPos, targetPos, slop, polys);

    if (!steerTarget.success) {
      break;
    }

    final isEndOfPath = steerTarget.steerPosFlags.value & StraightPathPointFlags.end.value;
    final isOffMeshConnection = steerTarget.steerPosFlags.value & StraightPathPointFlags.offMesh.value;

    // find movement delta
    final steerPos = steerTarget.steerPos;
    final Vector3 delta = _findSmoothPathDelta.sub2(steerPos, iterPos);
    double len = delta.length;

    // always move to the steer target, but limit each step to stepSize
    len = math.min(stepSize, len) / len;
    final Vector3 moveTarget = _findSmoothPathMoveTarget.setValues(iterPos.x, iterPos.y, iterPos.z).addScaled(delta, len);

    // move along surface
    final moveAlongSurfaceResult = moveAlongSurface(navMesh, polys[0], iterPos, moveTarget, queryFilter);
    if (!moveAlongSurfaceResult.success){
      break;
    }

    final resultPosition = moveAlongSurfaceResult.position;

    polys = mergeCorridorStartMoved(polys, moveAlongSurfaceResult.visited, 256);
    fixupShortcuts(polys, navMesh);

    iterPos.setFrom(resultPosition);

    // handle end of path and off-mesh links when close enough
    if (isEndOfPath != 0 && inRange(iterPos, steerTarget.steerPos, slop, 1.0)) {
      // reached end of path
      iterPos.setFrom(targetPos);

      if (result.path.length < maxPoints) {
        result.path.add(SmoothPathPoint(
          position: iterPos.clone(),
          type: NodeType.poly,
          nodeRef: result.endNodeRef,
          flags: SmoothPathPointFlags.end.value,
        ));
      }

      break;
    } 
    else if (isOffMeshConnection != 0 && inRange(iterPos, steerTarget.steerPos, slop, 1.0)) {
      // reached off-mesh connection
      final offMeshConRef = steerTarget.steerPosRef;

      // advance the path up to and over the off-mesh connection
      int? prevNodeRef;
      int nodeRef = polys[0];
      int npos = 0;

      while (npos < polys.length && nodeRef != offMeshConRef) {
        prevNodeRef = nodeRef;
        nodeRef = polys[npos];
        npos++;
      }

      // remove processed polys
      polys.removeAt(npos);

      // handle the off-mesh connection
      final offMeshNode = getNodeByRef(navMesh, offMeshConRef);
      final offMeshConnection = navMesh.offMeshConnections[offMeshNode?.offMeshConnectionId ?? 0];

      if (offMeshConnection != null && prevNodeRef != null) {
        // find the link from the previous poly to the off-mesh node to determine direction
        final prevNode = getNodeByRef(navMesh, prevNodeRef);
        int? linkEdge = 0; // default to START

        for (final linkIndex in prevNode?.links ?? []) {
          final link = navMesh.links[linkIndex];
          if (link?.toNodeRef == offMeshConRef) {
            linkEdge = link?.edge;
            break;
          }
        }

        // use the link edge to determine direction
        // edge 0 = entering from START side, edge 1 = entering from END side
        final enteringFromStart = linkEdge == 0;

        if (result.path.length < maxPoints) {
          result.path.add(SmoothPathPoint(
            position: iterPos.clone(),
            type: NodeType.offMesh,
            nodeRef: offMeshConRef,
            flags: SmoothPathPointFlags.offMesh.value,
          ));

          final endPosition = enteringFromStart ? offMeshConnection.end : offMeshConnection.start;

          iterPos.setFrom(endPosition);
        }
      }
    }

    // store results - add a point for each iteration to create smooth path
    if (result.path.length < maxPoints) {
      // determine the current ref from the current position
      final currentNodeRef = polys.length > 0 ? polys[0] : result.endNodeRef;

      result.path.add(SmoothPathPoint(
        position: iterPos.clone(),
        type: NodeType.poly,
        nodeRef: currentNodeRef,
        flags: 0,
      ));
    }
  }

    // compose flags
  int flags = FindSmoothPathResultFlags.success.value;
  final int nodePathFlags = nodePath.flags;
  
  if ((nodePathFlags & FindNodePathResultFlags.completePath.value) != 0) {
    flags |= FindSmoothPathResultFlags.completePath.value;
  } else if ((nodePathFlags & FindNodePathResultFlags.partialPath.value) != 0) {
    flags |= FindSmoothPathResultFlags.partialPath.value;
  }

  result.success = true;
  result.flags = FindSmoothPathResultFlags.values[flags];

  return result;
}

class GetSteerTargetResult {
  bool success;
  Vector3 steerPos;
  int steerPosRef;
  FindSmoothPathResultFlags steerPosFlags;

  GetSteerTargetResult({
    required this.success,
    required this.steerPos,
    required this.steerPosRef,
    required this.steerPosFlags,
  });
}

GetSteerTargetResult getSteerTarget(
  NavMesh navMesh,
  Vector3 start,
  Vector3 end,
  double minTargetDist,
  List<int> pathPolys,
) {
  final result = GetSteerTargetResult(
    success: false,
    steerPos: Vector3(),
    steerPosRef: invalidNodeRef,
    steerPosFlags: FindSmoothPathResultFlags.none,
  );

  const int maxStraightPathPoints = 3;
  final straightPath = findStraightPath(navMesh, start, end, pathPolys, maxStraightPathPoints, 0);

  if (!straightPath.success || straightPath.path.length == 0) {
    return result;
  }

  // find vertex far enough to steer to
  int ns = 0;
  while (ns < straightPath.path.length) {
    final point = straightPath.path[ns];

    // stop at off-mesh link
    if (point.type == NodeType.offMesh.value) {
      break;
    }

    // if this point is far enough from start, we can steer to it
    if (!inRange(point.position, start, minTargetDist, 1000.0)) {
      break;
    }

    ns++;
  }

  // failed to find good point to steer to
  if (ns >= straightPath.path.length) {
    return result;
  }

  final steerPoint = straightPath.path[ns];

  result.steerPos.setFrom(steerPoint.position);
  result.steerPosRef = steerPoint.nodeRef ?? invalidNodeRef;
  result.steerPosFlags = FindSmoothPathResultFlags.values[ steerPoint.flags];
  result.success = true;

  return result;
}

bool inRange(Vector3 a, Vector3 b, double r, double h) {
  final double dx = b.x - a.x;
  final double dy = b.y - a.y;
  final double dz = b.z - a.z;
  return (dx * dx + dz * dz) < (r * r) && dy.abs() < h;
}

List<int> mergeCorridorStartMoved(List<int> currentPath, List<int> visited, int maxPath) {
  if (visited.length == 0) return currentPath;

  int furthestPath = -1;
  int furthestVisited = -1;

  // find furthest common polygon
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

  // if no intersection found, just return current path
  if (furthestPath == -1 || furthestVisited == -1) {
    return currentPath;
  }

  // concatenate paths
  final int req = visited.length - furthestVisited;
  final orig = math.min(furthestPath + 1, currentPath.length);
  int size = math.max(0, currentPath.length - orig);

  if (req + size > maxPath) {
    size = maxPath - req;
  }

  final newPath = <int>[];

  // store visited polygons (in reverse order)
  for (int i = 0; i < math.min(req, maxPath); i++) {
    newPath[i] = visited[visited.length - 1 - i];
  }

  // add remaining current path
  if (size > 0) {
    for (int i = 0; i < size; i++) {
      newPath[req + i] = currentPath[orig + i];
    }
  }

  return newPath.sublist(0, req + size);
}

/// This function checks if the path has a small U-turn, that is,
/// a polygon further in the path is adjacent to the first polygon
/// in the path. If that happens, a shortcut is taken.
/// This can happen if the target (T) location is at tile boundary,
/// and we're approaching it parallel to the tile edge.
/// The choice at the vertex can be arbitrary,
///  +---+---+
///  |:::|:::|
///  +-S-+-T-+
///  |:::|   | <-- the step can end up in here, resulting U-turn path.
///  +---+---+
///
void fixupShortcuts(List<int> pathPolys, NavMesh navMesh) {
  if (pathPolys.length < 3) {
    return;
  }

  // Track shared adjacency links leading from our current head position
  const int maxNeis = 16;
  int nneis = 0;
  final List<int> neis = [];

  final firstNode = getNodeByRef(navMesh, pathPolys[0]);
  final firstNodeLinks = firstNode?.links;

  for (final linkIndex in firstNodeLinks ?? []) {
    final link = navMesh.links[linkIndex];
    if (link != null && nneis < maxNeis) {
      neis.add(link.toNodeRef);
      nneis++;
    }
  }

  // Look ahead along the corridor to see if any later step is already adjacent to our starting node
  const int maxLookAhead = 6;
  int cut = 0;
  final int lookAheadLimit = math.min(maxLookAhead, pathPolys.length);

  for (int i = lookAheadLimit - 1; i > 1 && cut == 0; i--) {
    for (int j = 0; j < nneis; j++) {
      if (pathPolys[i] == neis[j]) {
        cut = i;
        break;
      }
    }
  }

  // Extract redundant routing indices by invoking a splice deletion if a shortcut exists
  if (cut > 1) {
    pathPolys.removeRange(1, cut);
  }
}