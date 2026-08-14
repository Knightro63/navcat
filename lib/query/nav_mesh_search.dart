import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';
import 'index.dart';
import '../math/vector.dart';
import '../geometry.dart';

// A* Node execution tracking bits
const int nodeFlagOpen = 0x01;
const int nodeFlagClosed = 0x02;
const int nodeFlagParentDetached = 0x04;

class SearchNode {
  final Vector3 position;
  double cost;
  double total;
  int? parentNodeRef;
  int? parentState;
  int state;
  int flags;
  int nodeRef;

  SearchNode({
    required this.position,
    required this.cost,
    required this.total,
    this.parentNodeRef,
    this.parentState,
    required this.state,
    required this.flags,
    required this.nodeRef,
  });
}

class SearchQuery{
  SearchNode node;
  bool wasDetached;

  SearchQuery({
    required this.node,
    required this.wasDetached,
  });
}

// Map mapping to original SearchNodePool [nodeRef: NodeRef]: SearchNode[]
typedef SearchNodePool = Map<int, List<SearchNode>>;
typedef SearchNodeQueue = List<SearchNode>;

SearchNode? getSearchNode(SearchNodePool pool, int nodeRef, int state) {
  final List<SearchNode>? nodes = pool[nodeRef];
  if (nodes == null) return null;
  for (int i = 0; i < nodes.length; i++) {
    if (nodes[i].state == state) {
      return nodes[i];
    }
  }
  return null;
}

void addSearchNode(SearchNodePool pool, SearchNode node) {
  final List<SearchNode> nodes = pool.putIfAbsent(node.nodeRef, () => <SearchNode>[]);
  nodes.add(node);
}

void bubbleUpQueue(SearchNodeQueue queue, int i, SearchNode node) {
  int parent = ((i - 1) / 2).floor();
  while (i > 0 && queue[parent].total > node.total) {
    queue[i] = queue[parent];
    i = parent;
    parent = ((i - 1) / 2).floor();
  }
  queue[i] = node;
}

void trickleDownQueue(SearchNodeQueue queue, int i, SearchNode node) {
  final int count = queue.length;
  int child = 2 * i + 1;
  while (child < count) {
    if (child + 1 < count && queue[child + 1].total < queue[child].total) {
      child++;
    }
    if (node.total <= queue[child].total) {
      break;
    }
    queue[i] = queue[child];
    i = child;
    child = i * 2 + 1;
  }
  queue[i] = node;
}

void pushNodeToQueue(SearchNodeQueue queue, SearchNode node) {
  queue.add(node);
  bubbleUpQueue(queue, queue.length - 1, node);
}

SearchNode? popNodeFromQueue(SearchNodeQueue queue) {
  if (queue.isEmpty) return null;
  final SearchNode node = queue[0];
  final SearchNode lastNode = queue.removeLast();
  if (queue.isNotEmpty) {
    queue[0] = lastNode;
    trickleDownQueue(queue, 0, lastNode);
  }
  return node;
}

void reindexNodeInQueue(SearchNodeQueue queue, SearchNode node) {
  for (int i = 0; i < queue.length; i++) {
    if (queue[i].nodeRef == node.nodeRef && queue[i].state == node.state) {
      queue[i] = node;
      bubbleUpQueue(queue, i, node);
      return;
    }
  }
}

// Pre-allocated static layout buffers to bypass loop runtime allocation footprints
final Vector3 _getPortalPointsStart = Vector3();
final Vector3 _getPortalPointsEnd = Vector3();

/// Retrieves the left and right points of the portal edge between two adjacent polygons.
/// Or if one of the polygons is an off-mesh connection, returns the connection endpoint for both left and right.
bool getPortalPoints(
  NavMesh navMesh,
  int fromNodeRef,
  int toNodeRef,
  Vector3 outLeft,
  Vector3 outRight,
) {
  NavMeshLink? toLink;
  final NavMeshNode? fromNode = getNodeByRef(navMesh, fromNodeRef);
  
  for (final linkIndex in fromNode?.links ?? []) {
    final NavMeshLink? link = navMesh.links[linkIndex];
    if (link?.toNodeRef == toNodeRef) {
      toLink = link;
      break;
    }
  }
  
  if (toLink == null) return false;

  final int fromNodeType = getNodeRefType(fromNodeRef);
  final int toNodeType = getNodeRefType(toNodeRef);

  // handle from poly to poly
  if (fromNodeType == NodeType.poly.value && toNodeType == NodeType.poly.value) {
    final NavMeshNode? fromPolyData = getNodeByRef(navMesh, fromNodeRef);
    final int fromTileId = fromPolyData!.tileId;
    final int fromPolyIndex = fromPolyData.polyIndex;
    
    final NavMeshTile? fromTile = navMesh.tiles[fromTileId];
    final NavMeshPoly fromPoly = fromTile!.polys![fromPolyIndex];

    final int linkEdge = toLink.edge;
    final int v0Index = fromPoly.vertices[linkEdge].toInt();
    final int v1Index = fromPoly.vertices[(linkEdge + 1) % fromPoly.vertices.length].toInt();
    
    final int v0Offset = v0Index * 3;
    final int v1Offset = v1Index * 3;
    final List<double> tileVerts = List<double>.from(fromTile.vertices);

    // If the link resides at a tile boundary, clamp the vertices to the link width width constraints
    if (toLink.side != 0xff && (toLink.bmin != 0 || toLink.bmax != 255)) {
      const double s = 1.0 / 255.0;
      final double tmin = (toLink.bmin as int) * s;
      final double tmax = (toLink.bmax as int) * s;
      
      _getPortalPointsStart.setValues(tileVerts[v0Offset], tileVerts[v0Offset + 1], tileVerts[v0Offset + 2]);
      _getPortalPointsEnd.setValues(tileVerts[v1Offset], tileVerts[v1Offset + 1], tileVerts[v1Offset + 2]);
      
      outLeft.lerpVectors(_getPortalPointsStart, _getPortalPointsEnd, tmin);
      outRight.lerpVectors(_getPortalPointsStart, _getPortalPointsEnd, tmax);
    } else {
      // No boundary clamping needed - apply direct vertex data extraction mapping
      outLeft.setValues(tileVerts[v0Offset], tileVerts[v0Offset + 1], tileVerts[v0Offset + 2]);
      outRight.setValues(tileVerts[v1Offset], tileVerts[v1Offset + 1], tileVerts[v1Offset + 2]);
    }
    return true;
  }

  // handle from poly to offmesh connection
  if (fromNodeType == NodeType.poly.value && toNodeType == NodeType.offMesh.value) {
    final NavMeshNode toNode = getNodeByRef(navMesh, toNodeRef)!;
    final OffMeshConnection? offMeshConnection = navMesh.offMeshConnections[toNode.offMeshConnectionId];
    if (offMeshConnection == null) return false;
    
    final Vector3 position = (toLink.edge == 0) ? offMeshConnection.start : offMeshConnection.end;
    outLeft.setFrom(position);
    outRight.setFrom(position);
    return true;
  }

  // handle from offmesh connection to poly
  if (fromNodeType == NodeType.offMesh.value && toNodeType == NodeType.poly.value) {
    final OffMeshConnection? offMeshConnection = navMesh.offMeshConnections[fromNode!.offMeshConnectionId];
    if (offMeshConnection == null) return false;
    
    final Vector3 position = (toLink.edge == 0) ? offMeshConnection.start : offMeshConnection.end;
    outLeft.setFrom(position);
    outRight.setFrom(position);
    return true;
  }

  return false;
}

// Pre-allocated static registers to bypass lookahead memory re-allocation steps
final Vector3 _edgeMidPointPortalLeft = Vector3();
final Vector3 _edgeMidPointPortalRight = Vector3();

/// Calculates the center point of a shared edge between two adjacent nodes
bool getEdgeMidPoint(NavMesh navMesh, int fromNodeRef, int toNodeRef, Vector3 outMidPoint) {
  if (!getPortalPoints(navMesh, fromNodeRef, toNodeRef, _edgeMidPointPortalLeft, _edgeMidPointPortalRight)) {
    return false;
  }
  outMidPoint.x = (_edgeMidPointPortalLeft.x + _edgeMidPointPortalRight.x) * 0.5;
  outMidPoint.y = (_edgeMidPointPortalLeft.y + _edgeMidPointPortalRight.y) * 0.5;
  outMidPoint.z = (_edgeMidPointPortalLeft.z + _edgeMidPointPortalRight.z) * 0.5;
  return true;
}

/// Status flags tracking graph corridor search resolutions
enum FindNodePathResultFlags {
  none(0),
  success(1 << 0),
  completePath(1 << 1),
  partialPath(1 << 2),
  invalidPath(1 << 3);

  final int value;
  const FindNodePathResultFlags(this.value);
}

/// Strongly typed container enclosing full execution tracking instances
class FindNodePathResult {
  final bool success;
  final int flags;
  final List<int> path; // Contains sequential NodeRef entities
  final SearchNodePool nodes;
  final SearchNodeQueue openList;

  FindNodePathResult({
    required this.success,
    required this.flags,
    required this.path,
    required this.nodes,
    required this.openList,
  });
}

const double huristicScale = 0.999;

/// Find an optimized route corridor between two specific node references on the navigation graph
FindNodePathResult findNodePath(
  NavMesh navMesh,
  int startNodeRef,
  int endNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter,
  [Map<String, dynamic>? options]
) {
  final raycastDistance = options?['raycastDistance'] ?? 0;
  final SearchNodePool nodes = {};
  final SearchNodeQueue openList = [];
  
  // Validate spatial input definitions
  if (
    !isValidNodeRef(navMesh, startNodeRef) || 
    !isValidNodeRef(navMesh, endNodeRef) || 
    !startPosition.isFinite() || 
    !endPosition.isFinite()
  ) {
    return FindNodePathResult(
      flags: FindNodePathResultFlags.none.value | FindNodePathResultFlags.invalidPath.value,
      success: false,
      path: [],
      nodes: nodes,
      openList: openList,
    );
  }

  // Early out if start and target locations correspond to the exact same tracking slot
  if (startNodeRef == endNodeRef) {
    return FindNodePathResult(
      flags: FindNodePathResultFlags.success.value | FindNodePathResultFlags.completePath.value,
      success: true,
      path: [startNodeRef],
      nodes: nodes,
      openList: openList,
    );
  }

  // Prepare baseline search constraints
  final SearchNode startNode = SearchNode(
    cost: 0.0,
    total: startPosition.distanceTo(endPosition) * huristicScale,
    parentNodeRef: null,
    parentState: null,
    nodeRef: startNodeRef,
    state: 0,
    flags: nodeFlagOpen,
    position: Vector3(startPosition.x, startPosition.y, startPosition.z),
  );

  addSearchNode(nodes, startNode);
  pushNodeToQueue(openList, startNode);

  SearchNode lastBestNode = startNode;
  double lastBestNodeCost = startNode.total;

  while (openList.isNotEmpty) {
    final SearchNode bestSearchNode = popNodeFromQueue(openList)!;
    bestSearchNode.flags &= ~nodeFlagOpen;
    bestSearchNode.flags |= nodeFlagClosed;

    final int bestNodeRef = bestSearchNode.nodeRef;
    if (bestNodeRef == endNodeRef) {
      lastBestNode = bestSearchNode;
      break;
    }

    final NavMeshNode? bestNode = getNodeByRef(navMesh, bestNodeRef);
    final bestNodeIsPoly = getNodeRefType(bestNodeRef) == NodeType.poly.value;
    final int? parentNodeRef = bestSearchNode.parentNodeRef;

    for (final linkIndex in bestNode?.links ?? []) {
      final NavMeshLink? link = navMesh.links[linkIndex];
      final int neighbourNodeRef = link?.toNodeRef ?? 0;

      if (neighbourNodeRef == parentNodeRef){ 
        continue;
      }
      if (filter?.passFilter(neighbourNodeRef, navMesh) == false){ 
        continue;
      }

      int state = 0;
      if (link?.side != 0xff) {
        state = link!.side >> 1;
      }

      SearchNode? neighbourSearchNode = getSearchNode(nodes, neighbourNodeRef, state);
      if (neighbourSearchNode == null) {
        neighbourSearchNode = SearchNode(
          cost: 0.0,
          total: 0.0,
          parentNodeRef: null,
          parentState: null,
          nodeRef: neighbourNodeRef,
          state: state,
          flags: 0,
          position: Vector3(0.0, 0.0, 0.0),
        );
        addSearchNode(nodes, neighbourSearchNode);
        getEdgeMidPoint(navMesh, bestNodeRef, neighbourNodeRef, neighbourSearchNode.position);
      }

      // Calculate traversal cost metrics
      final double curCost = filter?.getCost(
        bestSearchNode.position, 
        neighbourSearchNode.position, 
        navMesh, 
        parentNodeRef, 
        bestNodeRef, 
        neighbourNodeRef
      ) ?? 0;
      
      double cost = bestSearchNode.cost + curCost;
      double heuristic = 0.0;

      bool foundShortcut = false;
      if (
        raycastDistance > 0 &&
        bestNodeIsPoly &&
        parentNodeRef != null &&
        getNodeRefType(parentNodeRef) == NodeType.poly.value &&
        bestSearchNode.parentState != null
      ) {
        // get grandparent node for potential raycast shortcut
        final grandparentNode = getSearchNode(nodes, parentNodeRef, bestSearchNode.parentState!);

        if (grandparentNode != null) {
          final rayLength = grandparentNode.position.distanceTo(neighbourSearchNode.position);

          if (rayLength < raycastDistance) {
            // attempt raycast from grandparent to current neighbor
            final rayResult = raycastWithCosts(
              navMesh,
              grandparentNode.nodeRef,
              grandparentNode.position,
              neighbourSearchNode.position,
              filter,
              grandparentNode.parentNodeRef ?? 0, // pass the great-grandparent for accurate cost calculations
            );

            // if the raycast didn't hit anything, we can take the shortcut
            if (rayResult.t >= 1.0) {
              foundShortcut = true;
              cost = grandparentNode.cost + rayResult.pathCost;
            }
          }
        }
      }

      if (!foundShortcut) {
        // normal cost calculation
        final curCost = filter?.getCost(
            bestSearchNode.position,
            neighbourSearchNode.position,
            navMesh,
            parentNodeRef,
            bestNodeRef,
            neighbourNodeRef,
        ) ?? 0;
        cost = bestSearchNode.cost + curCost;
      }

      if (neighbourNodeRef == endNodeRef) {
        final double endCost = filter?.getCost(
          neighbourSearchNode.position, 
          endPosition, 
          navMesh, 
          bestNodeRef, 
          neighbourNodeRef, 
          null
        ) ?? 0;
        cost += endCost;
        heuristic = 0.0;
      } 
      else {
        heuristic = neighbourSearchNode.position.distanceTo(endPosition) * huristicScale;
      }

      final double total = cost + heuristic;

      if ((neighbourSearchNode.flags & nodeFlagOpen) != 0 && total >= neighbourSearchNode.total){ 
        continue;
      }
      if ((neighbourSearchNode.flags & nodeFlagClosed) != 0 && total >= neighbourSearchNode.total){ 
        continue;
      }

      if (foundShortcut) {
        neighbourSearchNode.parentNodeRef = bestSearchNode.parentNodeRef;
        neighbourSearchNode.parentState = bestSearchNode.parentState;
      } 
      else {
        neighbourSearchNode.parentNodeRef = bestSearchNode.nodeRef;
        neighbourSearchNode.parentState = bestSearchNode.state;
      }

      neighbourSearchNode.nodeRef = neighbourNodeRef;
      neighbourSearchNode.flags &= ~nodeFlagClosed;
      neighbourSearchNode.cost = cost;
      neighbourSearchNode.total = total;

      if ((neighbourSearchNode.flags & nodeFlagOpen) != 0) {
        reindexNodeInQueue(openList, neighbourSearchNode);
      } else {
        neighbourSearchNode.flags |= nodeFlagOpen;
        pushNodeToQueue(openList, neighbourSearchNode);
      }

      if (heuristic < lastBestNodeCost) {
        lastBestNode = neighbourSearchNode;
        lastBestNodeCost = heuristic;
      }
    }
  }

  // Assemble sequential route corridor references list
  final List<int> path = [];
  SearchNode? currentNode = lastBestNode;
  
  while (currentNode != null) {
    path.add(currentNode.nodeRef);
    if (currentNode.parentNodeRef != null && currentNode.parentState != null) {
      currentNode = getSearchNode(nodes, currentNode.parentNodeRef!, currentNode.parentState!);
    } else {
      currentNode = null;
    }
  }
  
  final List<int> reversedPath = path.reversed.toList();

  if (lastBestNode.nodeRef != endNodeRef) {
    return FindNodePathResult(
      flags: FindNodePathResultFlags.partialPath.value,
      success: true,
      path: reversedPath,
      nodes: nodes,
      openList: openList,
    );
  }

  return FindNodePathResult(
    flags: FindNodePathResultFlags.success.value | FindNodePathResultFlags.completePath.value,
    success: true,
    path: reversedPath,
    nodes: nodes,
    openList: openList,
  );
}

// --- Incremental Search Engine Flags ---
abstract class SlicedFindNodePathStatusFlags {
  static const int notInitialized = 0;
  static const int inProgress = 1;
  static const int success = 2;
  static const int partialResult = 4;
  static const int failure = 8;
  static const int invalidParam = 16;
}

abstract class SlicedFindNodePathInitFlags {
  /// Enable any-angle pathfinding with raycast optimization
  static const int anyAngle = 1;
}

/// Thread/frame-safe search state context object tracking continuous execution slices
class SlicedNodePathQuery {
  int status;
  
  // Search parameters
  int startNodeRef;
  int endNodeRef;
  final Vector3 startPosition;
  final Vector3 endPosition;
  QueryFilter? filter;
  
  // Search state pools
  SearchNodePool nodes;
  SearchNodeQueue openList;
  SearchNode? lastBestNode;
  double lastBestNodeCost;
  
  // Raycast optimization configurations
  double? raycastLimitSqr;

  SlicedNodePathQuery({
    this.status = SlicedFindNodePathStatusFlags.notInitialized,
    this.startNodeRef = 0,
    this.endNodeRef = 0,
    Vector3? startPosition,
    Vector3? endPosition,
    this.filter,
    SearchNodePool? nodes,
    SearchNodeQueue? openList,
    this.lastBestNode,
    this.lastBestNodeCost = double.maxFinite,
    this.raycastLimitSqr,
  })  : startPosition = startPosition ?? Vector3(0.0, 0.0, 0.0),
        endPosition = endPosition ?? Vector3(0.0, 0.0, 0.0),
        nodes = nodes ?? {},
        openList = openList ?? [];
}

/// Creates a new sliced path query object with default initialization metrics
SlicedNodePathQuery createSlicedNodePathQuery() {
  return SlicedNodePathQuery(
    status: SlicedFindNodePathStatusFlags.notInitialized,
    startNodeRef: 0,
    endNodeRef: 0,
    startPosition: Vector3(0.0, 0.0, 0.0),
    endPosition: Vector3(0.0, 0.0, 0.0),
    filter: null, // Assign your DEFAULT_QUERY_FILTER reference instance here
    nodes: {},
    openList: [],
    lastBestNode: null,
    lastBestNodeCost: double.maxFinite,
    raycastLimitSqr: null,
  );
}

/// Initializes a sliced path query state engine instance to prepare for incremental loop ticks
int initSlicedFindNodePath(
  NavMesh navMesh,
  SlicedNodePathQuery query,
  int startNodeRef,
  int endNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter, [
  int flags = 0,
]) {
  // Set tracking parameters using modern .copy mapping styles
  query.startNodeRef = startNodeRef;
  query.endNodeRef = endNodeRef;
  query.startPosition.setFrom(startPosition);
  query.endPosition.setFrom(endPosition);
  query.filter = filter;
  
  // Hard-reset internal loop trackers
  query.status = SlicedFindNodePathStatusFlags.failure;
  query.nodes.clear();
  query.openList.clear();
  query.lastBestNode = null;
  query.lastBestNodeCost = double.maxFinite;

  // Validate structural spatial inputs before committing frame resources
  if (!isValidNodeRef(navMesh, startNodeRef) || 
      !isValidNodeRef(navMesh, endNodeRef) || 
      !startPosition.isFinite() || 
      !endPosition.isFinite()) {
    query.status = SlicedFindNodePathStatusFlags.failure | SlicedFindNodePathStatusFlags.invalidParam;
    return query.status;
  }

  // Handle any-angle raycast optimization state setups
  if ((flags & SlicedFindNodePathInitFlags.anyAngle) != 0) {
    query.raycastLimitSqr = 25.0; // Sensible default constraint configuration baseline
  } else {
    query.raycastLimitSqr = null;
  }

  // Construct the graph search starting master node
  final SearchNode startNode = SearchNode(
    cost: 0.0,
    total: startPosition.distanceTo(endPosition) * huristicScale,
    parentNodeRef: null,
    parentState: null,
    nodeRef: startNodeRef,
    state: 0,
    flags: nodeFlagOpen,
    position: Vector3(startPosition.x, startPosition.y, startPosition.z),
  );

  addSearchNode(query.nodes, startNode);
  query.lastBestNode = startNode;
  query.lastBestNodeCost = startNode.total;

  // Early out loop sequence if both refs align to the exact same tracking poly slot
  if (startNodeRef == endNodeRef) {
    query.status = SlicedFindNodePathStatusFlags.success;
    return query.status;
  }

  pushNodeToQueue(query.openList, startNode);
  query.status = SlicedFindNodePathStatusFlags.inProgress;
  
  return query.status;
}

/// Updates an in-progress sliced path query for a discrete number of steps.
/// 
/// Returns the total number of evaluation iterations performed during this tick frame.
int updateSlicedFindNodePath(NavMesh navMesh, SlicedNodePathQuery query, int maxIterations) {
  int itersDone = 0;
  
  // Check if query is in a valid progress state
  if ((query.status & SlicedFindNodePathStatusFlags.inProgress) == 0) {
    return itersDone;
  }
  
  // Validate node refs remain active and valid across asynchronous loop boundaries
  if (!isValidNodeRef(navMesh, query.startNodeRef) || !isValidNodeRef(navMesh, query.endNodeRef)) {
    query.status = SlicedFindNodePathStatusFlags.failure;
    return itersDone;
  }
  
  while (itersDone < maxIterations && query.openList.isNotEmpty) {
    itersDone++;
    
    // Remove best node from open list and close it
    final SearchNode bestSearchNode = popNodeFromQueue(query.openList)!;
    bestSearchNode.flags &= ~nodeFlagOpen;
    bestSearchNode.flags |= nodeFlagClosed;
    
    // Check if we've reached the goal node target
    if (bestSearchNode.nodeRef == query.endNodeRef) {
      query.lastBestNode = bestSearchNode;
      query.status = SlicedFindNodePathStatusFlags.success;
      return itersDone;
    }
    
    final int bestNodeRef = bestSearchNode.nodeRef;
    final NavMeshNode? bestNode = getNodeByRef(navMesh, bestNodeRef);
    final int? parentNodeRef = bestSearchNode.parentNodeRef;
    
    // Expand search space to adjacent neighbors
    for (final linkIndex in bestNode?.links ?? []) {
      final NavMeshLink? link = navMesh.links[linkIndex];
      final int neighbourNodeRef = link?.toNodeRef ?? 0;
      
      if (neighbourNodeRef == parentNodeRef) continue;
      if (query.filter?.passFilter(neighbourNodeRef, navMesh) == false) continue;
      
      int state = 0;
      if (link?.side != 0xff) {
        state = link!.side >> 1;
      }
      
      // Fetch or instantiate matching neighborhood search frame context
      SearchNode? neighbourSearchNode = getSearchNode(query.nodes, neighbourNodeRef, state);
      if (neighbourSearchNode == null) {
        neighbourSearchNode = SearchNode(
          cost: 0.0,
          total: 0.0,
          parentNodeRef: null,
          parentState: null,
          nodeRef: neighbourNodeRef,
          state: state,
          flags: 0,
          position: Vector3(0.0, 0.0, 0.0),
        );
        addSearchNode(query.nodes, neighbourSearchNode);
        getEdgeMidPoint(navMesh, bestNodeRef, neighbourNodeRef, neighbourSearchNode.position);
      }
      
      double cost = 0.0;
      double heuristic = 0.0;
      bool foundShortcut = false;
      
      // Evaluate raycast string-pulling shortcuts if a grandparent exists
      final double? raycastLimitSqr = query.raycastLimitSqr;
      if (raycastLimitSqr != null && bestSearchNode.parentNodeRef != null && bestSearchNode.parentState != null) {
        final SearchNode? grandparentNode = getSearchNode(query.nodes, bestSearchNode.parentNodeRef!, bestSearchNode.parentState!);
        
        if (grandparentNode != null) {
          final double rayLength = grandparentNode.position.distanceTo(neighbourSearchNode.position);
          if (rayLength < math.sqrt(raycastLimitSqr)) {
            // Attempt straight line query verification across polygon bounds
            final RaycastResult rayResult = raycastWithCosts(
              navMesh,
              grandparentNode.nodeRef,
              grandparentNode.position,
              neighbourSearchNode.position,
              query.filter,
              grandparentNode.parentNodeRef ?? 0,
            );
            
            if (rayResult.t >= 1.0) {
              foundShortcut = true;
              cost = grandparentNode.cost + rayResult.pathCost;
            }
          }
        }
      }
      
      // Standard incremental traversal cost evaluation fallback
      if (!foundShortcut) {
        final double curCost = query.filter?.getCost(
          bestSearchNode.position, neighbourSearchNode.position, navMesh,
          parentNodeRef, bestNodeRef, neighbourNodeRef,
        ) ?? 0;
        cost = bestSearchNode.cost + curCost;
      }
      
      // Append goal point calculation offset adjustments on the tail node step
      if (neighbourNodeRef == query.endNodeRef) {
        final double endCost = query.filter?.getCost(
          neighbourSearchNode.position, query.endPosition, navMesh,
          bestNodeRef, neighbourNodeRef, null,
        ) ?? 0;
        cost += endCost;
        heuristic = 0.0;
      } else {
        heuristic = neighbourSearchNode.position.distanceTo(query.endPosition) * huristicScale;
      }
      
      final double total = cost + heuristic;
      
      // Skip updates if calculated weight exceeds existing heap node limits
      if (((neighbourSearchNode.flags & nodeFlagOpen) != 0 && total >= neighbourSearchNode.total) ||
          ((neighbourSearchNode.flags & nodeFlagClosed) != 0 && total >= neighbourSearchNode.total)) {
        continue;
      }
      
      if (foundShortcut) {
        neighbourSearchNode.parentNodeRef = bestSearchNode.parentNodeRef;
        neighbourSearchNode.parentState = bestSearchNode.parentState;
        neighbourSearchNode.flags |= nodeFlagParentDetached;
      } else {
        neighbourSearchNode.parentNodeRef = bestSearchNode.nodeRef;
        neighbourSearchNode.parentState = bestSearchNode.state;
        neighbourSearchNode.flags &= ~nodeFlagParentDetached;
      }
      
      neighbourSearchNode.cost = cost;
      neighbourSearchNode.total = total;
      neighbourSearchNode.flags &= ~nodeFlagClosed;
      
      if ((neighbourSearchNode.flags & nodeFlagOpen) != 0) {
        reindexNodeInQueue(query.openList, neighbourSearchNode);
      } else {
        neighbourSearchNode.flags |= nodeFlagOpen;
        pushNodeToQueue(query.openList, neighbourSearchNode);
      }
      
      if (heuristic < query.lastBestNodeCost) {
        query.lastBestNodeCost = heuristic;
        query.lastBestNode = neighbourSearchNode;
      }
    }
  }
  
  // Flag complete if search open-set stack is completely exhausted
  if (query.openList.isEmpty) {
    query.status = SlicedFindNodePathStatusFlags.success | SlicedFindNodePathStatusFlags.partialResult;
  }
  
  return itersDone;
}

/// Container matching the structural layout fields of a sliced path resolution result
class SlicedFindNodePathResult {
  int status;
  List<int> path; // Contains sequential NodeRef entities
  int pathCount;

  SlicedFindNodePathResult({
    required this.status,
    required this.path,
    required this.pathCount,
  });
}

/// Finalizes and returns the complete results of a sliced path query.
SlicedFindNodePathResult finalizeSlicedFindNodePath(NavMesh navMesh, SlicedNodePathQuery query) {
  final List<int> defaultPath = [];
  
  if (query.lastBestNode == null) {
    query.status = SlicedFindNodePathStatusFlags.failure;
    return SlicedFindNodePathResult(status: query.status, path: defaultPath, pathCount: 0);
  }

  // Handle same start/end node case
  if (query.startNodeRef == query.endNodeRef) {
    defaultPath.add(query.startNodeRef);
    query.status = SlicedFindNodePathStatusFlags.notInitialized;
    return SlicedFindNodePathResult(status: SlicedFindNodePathStatusFlags.success, path: defaultPath, pathCount: 1);
  }

  // Check for partial result
  if (query.lastBestNode!.nodeRef != query.endNodeRef) {
    query.status |= SlicedFindNodePathStatusFlags.partialResult;
  }

  // Reverse the path by traversing parent chain using clean inline tuple records
  final List<SearchQuery> reversedPath = [];
  SearchNode? currentNode = query.lastBestNode;

  while (currentNode != null) {
    final bool wasDetached = (currentNode.flags & nodeFlagParentDetached) != 0;
    reversedPath.add(SearchQuery(node:currentNode, wasDetached: wasDetached));
    
    if (currentNode.parentNodeRef != null && currentNode.parentState != null) {
      currentNode = getSearchNode(query.nodes, currentNode.parentNodeRef!, currentNode.parentState!) ?? null;
    } else {
      currentNode = null;
    }
  }

  // Reverse to get forward progression order
  final List<SearchQuery> forwardPath = reversedPath.reversed.toList();
  final List<int> finalPath = [];

  if (query.raycastLimitSqr != null) {
    // Any-angle pathfinding was enabled, fill in raycast shortcuts across gaps
    for (int i = 0; i < forwardPath.length; i++) {
      final current = forwardPath[i];
      final next = (i + 1 < forwardPath.length) ? forwardPath[i + 1] : null;

      // If this node has a detached parent, raycast to discover missing intermediate boundary polygons
      if (next != null && current.wasDetached) {
        final RaycastResult rayResult = raycast(
          navMesh,
          current.node.nodeRef,
          current.node.position,
          next.node.position,
          query.filter,
        );

        for (final int polyRef in rayResult.path) {
          finalPath.add(polyRef);
        }

        // Remove edge duplicates if the trailing raycast poly boundary lands on the next target node
        if (finalPath.isNotEmpty && finalPath.last == next.node.nodeRef) {
          finalPath.removeLast();
        }
      } else {
        finalPath.add(current.node.nodeRef);
      }
    }
  } else {
    // No any-angle pathfinding shortcut used - extract direct adjacent node references
    for (int i = 0; i < forwardPath.length; i++) {
      finalPath.add(forwardPath[i].node.nodeRef);
    }
  }

  final int finalStatus = SlicedFindNodePathStatusFlags.success | (query.status & SlicedFindNodePathStatusFlags.partialResult);
  query.status = SlicedFindNodePathStatusFlags.notInitialized; // Reset tracking query engine frame status

  return SlicedFindNodePathResult(
    status: finalStatus,
    path: finalPath,
    pathCount: finalPath.length,
  );
}

/// Finalizes and returns the results of an incomplete sliced path query,
/// returning the path to the furthest polygon on the existing path that was visited during the search.
SlicedFindNodePathResult finalizeSlicedFindNodePathPartial(
  NavMesh navMesh,
  SlicedNodePathQuery query,
  List<int> existingPath,
) {
  final List<int> defaultPath = [];
  SearchNode? furthestNode;

  // Scan backward through existing route nodes to identify the furthest expanded search checkpoint
  for (int i = existingPath.length - 1; i >= 0; i--) {
    final int targetNodeRef = existingPath[i];
    final List<SearchNode>? nodes = query.nodes[targetNodeRef];
    
    if (nodes != null) {
      for (int j = 0; j < nodes.length; j++) {
        if (nodes[j].nodeRef == targetNodeRef) {
          furthestNode = nodes[j];
          break;
        }
      }
    }
    if (furthestNode != null) break;
  }

  if (furthestNode == null) {
    furthestNode = query.lastBestNode;
    query.status |= SlicedFindNodePathStatusFlags.partialResult;
  }

  if (furthestNode == null) {
    query.status = SlicedFindNodePathStatusFlags.failure;
    return SlicedFindNodePathResult(status: query.status, path: defaultPath, pathCount: 0);
  }

  // Handle same start/end node case
  if (query.startNodeRef == query.endNodeRef) {
    defaultPath.add(query.startNodeRef);
    query.status = SlicedFindNodePathStatusFlags.notInitialized;
    return SlicedFindNodePathResult(status: SlicedFindNodePathStatusFlags.success, path: defaultPath, pathCount: 1);
  }

  // Mark as partial result since we're winding down an incomplete search execution context
  query.status |= SlicedFindNodePathStatusFlags.partialResult;

  final List<SearchQuery> reversedPath = [];
  SearchNode? currentNode = furthestNode;

  while (currentNode != null) {
    final bool wasDetached = (currentNode.flags & nodeFlagParentDetached) != 0;
    reversedPath.add(SearchQuery(node: currentNode, wasDetached: wasDetached));
    
    if (currentNode.parentNodeRef != null && currentNode.parentState != null) {
      currentNode = getSearchNode(query.nodes, currentNode.parentNodeRef!, currentNode.parentState!) ?? null;
    } else {
      currentNode = null;
    }
  }

  final List<SearchQuery> forwardPath = reversedPath.reversed.toList();
  final List<int> finalPath = [];

  if (query.raycastLimitSqr != null) {
    // Fill gaps from shortcuts using raycast expansions
    for (int i = 0; i < forwardPath.length; i++) {
      final current = forwardPath[i];
      final next = (i + 1 < forwardPath.length) ? forwardPath[i + 1] : null;

      if (next != null && current.wasDetached) {
        final RaycastResult rayResult = raycast(
          navMesh,
          current.node.nodeRef,
          current.node.position,
          next.node.position,
          query.filter,
        );

        for (final int polyRef in rayResult.path) {
          finalPath.add(polyRef);
        }

        if (finalPath.isNotEmpty && finalPath.last == next.node.nodeRef) {
          finalPath.removeLast();
        }
      } else {
        finalPath.add(current.node.nodeRef);
      }
    }
  } else {
    // Raw adjacent copying
    for (int i = 0; i < forwardPath.length; i++) {
      finalPath.add(forwardPath[i].node.nodeRef);
    }
  }

  final int finalStatus = SlicedFindNodePathStatusFlags.success | SlicedFindNodePathStatusFlags.partialResult;
  query.status = SlicedFindNodePathStatusFlags.notInitialized;

  return SlicedFindNodePathResult(
    status: finalStatus,
    path: finalPath,
    pathCount: finalPath.length,
  );
}

// Low-allocation lookahead registers to eliminate continuous GC collection cycles
final List<double> _moveAlongSurfaceVertices = [];
final GetPolyHeightResult _moveAlongSurfacePolyHeightResult = createGetPolyHeightResult();
final Vector3 _moveAlongSurfaceWallEdgeVj = Vector3();
final Vector3 _moveAlongSurfaceWallEdgeVi = Vector3();
final Vector3 _moveAlongSurfaceLinkVj = Vector3();
final Vector3 _moveAlongSurfaceLinkVi = Vector3();
final DistancePtSegSqr2dResult _moveAlongSurfaceDistancePtSegSqr2dResult = createDistancePtSegSqr2dResult();
final Vector3 _moveAlongSurfaceSearchPos = Vector3();

class MoveAlongSurfaceResult {
  bool success;
  final Vector3 position;
  int nodeRef; // NodeRef target
  final List<int> visited; // Contains sequential NodeRef entities

  MoveAlongSurfaceResult({
    required this.success,
    required this.position,
    required this.nodeRef,
    required this.visited,
  });
}

/// Moves from start position towards end position along the navigation mesh surface.
MoveAlongSurfaceResult moveAlongSurface(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter,
) {
  final result = MoveAlongSurfaceResult(
    success: false,
    position: startPosition.clone(),
    nodeRef: startNodeRef,
    visited: [],
  );

  if (!isValidNodeRef(navMesh, startNodeRef) || !startPosition.isFinite() || !endPosition.isFinite()) {
    return result;
  }

  result.success = true;
  final SearchNodePool nodes = {};
  
  final SearchNode startNode = SearchNode(
    cost: 0.0,
    total: 0.0,
    parentNodeRef: null,
    parentState: null,
    nodeRef: startNodeRef,
    state: 0,
    flags: nodeFlagClosed,
    position: Vector3(startPosition.x, startPosition.y, startPosition.z),
  );
  addSearchNode(nodes, startNode);

  final Vector3 bestPos = startPosition.clone();
  double bestDist = double.maxFinite;
  SearchNode? bestNode = startNode;

  // Search geometry constraint limits calculation
  final Vector3 searchPos = _moveAlongSurfaceSearchPos.lerpVectors(startPosition, endPosition, 0.5);
  final double searchRadSqr = math.pow((startPosition.distanceTo(endPosition) / 2.0 + 0.001), 2) as double;

  // Breadth-first search queue
  final SearchNodeQueue queue = [startNode];

  while (queue.isNotEmpty) {
    final SearchNode curNode = queue.removeAt(0); // Equivalent to JS queue.shift()
    final int curRef = curNode.nodeRef;
    
    final GetTileAndPolyByRefResult tileAndPoly = getTileAndPolyByRef(navMesh,curRef);
    if (tileAndPoly.success != true) continue;
    
    final NavMeshTile tile = tileAndPoly.tile!;
    final NavMeshPoly poly = tileAndPoly.poly!;
    //final int polyIndex = tileAndPoly.polyIndex;
    final List<int> polyVertices = List<int>.from(poly.vertices);
    final int nv = polyVertices.length;
    
    // Dynamically expand flat double tracker array capacity safely if required
    if (_moveAlongSurfaceVertices.length < nv * 3) {
      _moveAlongSurfaceVertices.addAll(List<double>.filled((nv * 3) - _moveAlongSurfaceVertices.length, 0.0));
    }
    
    final List<double> tileVerts = List<double>.from(tile.vertices);
    for (int i = 0; i < nv; ++i) {
      final int start = polyVertices[i] * 3;
      _moveAlongSurfaceVertices[i * 3] = tileVerts[start];
      _moveAlongSurfaceVertices[i * 3 + 1] = tileVerts[start + 1];
      _moveAlongSurfaceVertices[i * 3 + 2] = tileVerts[start + 2];
    }

    // Target position confirmed inside current boundaries -> snap best selection and cease search
    if (pointInPoly(endPosition, _moveAlongSurfaceVertices, nv)) {
      bestNode = curNode;
      bestPos.setFrom(endPosition);
      break;
    }

    // Traverse bounding contours, isolating structural walls from shared adjacency portal nodes
    for (int i = 0, j = nv - 1; i < nv; j = i++) {
      final List<int> neis = [];
      final NavMeshNode? node = getNodeByRef(navMesh, curRef);
      
      for (final linkIndex in node?.links ?? []) {
        final NavMeshLink? link = navMesh.links[linkIndex];
        if (link == null) continue;
        final int neighbourRef = link.toNodeRef;

        if (link.edge == j) {
          if (filter?.passFilter(neighbourRef, navMesh) == false) continue;
          neis.add(neighbourRef);
        }
      }

      if (neis.isEmpty) {
        // Wall boundary detected -> verify distance criteria constraints
        final int jOffset = j * 3;
        final int iOffset = i * 3;
        final Vector3 vj = _moveAlongSurfaceWallEdgeVj.setValues(_moveAlongSurfaceVertices[jOffset], _moveAlongSurfaceVertices[jOffset + 1], _moveAlongSurfaceVertices[jOffset + 2]);
        final Vector3 vi = _moveAlongSurfaceWallEdgeVi.setValues(_moveAlongSurfaceVertices[iOffset], _moveAlongSurfaceVertices[iOffset + 1], _moveAlongSurfaceVertices[iOffset + 2]);
        
        final DistancePtSegSqr2dResult res = distancePtSegSqr2d(_moveAlongSurfaceDistancePtSegSqr2dResult, endPosition, vj, vi);
        final double distSqr = res.distSqr;
        
        if (distSqr < bestDist) {
          bestPos.lerpVectors(vj, vi, res.t);
          bestDist = distSqr;
          bestNode = curNode;
        }
      } else {
        for (final neighbourRef in neis) {
          SearchNode? neighbourNode = getSearchNode(nodes, neighbourRef, 0);
          if (neighbourNode == null) {
            neighbourNode = SearchNode(
              cost: 0.0,
              total: 0.0,
              parentNodeRef: null,
              parentState: null,
              nodeRef: neighbourRef,
              state: 0,
              flags: 0,
              position: Vector3(endPosition.x, endPosition.y, endPosition.z),
            );
            addSearchNode(nodes, neighbourNode);
          }

          if ((neighbourNode.flags & nodeFlagClosed) != 0) continue;

          final int jOffset = j * 3;
          final int iOffset = i * 3;
          final Vector3 vj = _moveAlongSurfaceLinkVj.setValues(_moveAlongSurfaceVertices[jOffset], _moveAlongSurfaceVertices[jOffset + 1], _moveAlongSurfaceVertices[jOffset + 2]);
          final Vector3 vi = _moveAlongSurfaceLinkVi.setValues(_moveAlongSurfaceVertices[iOffset], _moveAlongSurfaceVertices[iOffset + 1], _moveAlongSurfaceVertices[iOffset + 2]);
          
          final DistancePtSegSqr2dResult res = distancePtSegSqr2d(_moveAlongSurfaceDistancePtSegSqr2dResult, searchPos, vj, vi);
          if (res.distSqr > searchRadSqr) continue;

          neighbourNode.parentNodeRef = curNode.nodeRef;
          neighbourNode.parentState = curNode.state;
          neighbourNode.flags |= nodeFlagClosed;
          queue.add(neighbourNode);
        }
      }
    }
  }

  if (bestNode != null) {
    SearchNode? currentNode = bestNode;
    while (currentNode != null) {
      result.visited.add(currentNode.nodeRef);
      if (currentNode.parentNodeRef != null) {
        currentNode = getSearchNode(nodes, currentNode.parentNodeRef!, 0);
      } else {
        currentNode = null;
      }
    }
    result.visited.clear();
    result.visited.addAll(result.visited.reversed.toList());
    result.position.setFrom(bestPos);
    result.nodeRef = bestNode.nodeRef;

    // Apply surface planar height projection alignment adjustments
    final GetTileAndPolyByRefResult tileAndPoly = getTileAndPolyByRef(navMesh, result.nodeRef);
    if (tileAndPoly.success == true) {
      final GetPolyHeightResult polyHeightResult = getPolyHeight(
        _moveAlongSurfacePolyHeightResult,
        tileAndPoly.tile!,
        tileAndPoly.poly,
        tileAndPoly.polyIndex,
        result.position,
      );
      if (polyHeightResult.success == true) {
        result.position.y = polyHeightResult.height;
      }
    }
  }

  return result;
}

// Low-allocation lookahead registers to eliminate continuous GC collection cycles
final List<double> _raycastVertices = [];
final Vector3 _raycastDir = Vector3();
final Vector3 _raycastCurPos = Vector3();
final Vector3 _raycastLastPos = Vector3();
final Vector3 _raycastE1Vec = Vector3();
final Vector3 _raycastE0Vec = Vector3();
final Vector3 _raycastEDir = Vector3();
final Vector3 _raycastDiff = Vector3();
final Vector3 _raycastHitNormalVa = Vector3();
final Vector3 _raycastHitNormalVb = Vector3();

class RaycastResult {
  double t;
  final Vector3 hitNormal;
  int hitEdgeIndex;
  final List<int> path; // Contains sequential NodeRef entities
  double pathCost;

  RaycastResult({
    required this.t,
    required this.hitNormal,
    required this.hitEdgeIndex,
    required this.path,
    required this.pathCost,
  });
}

/// Internal base implementation of raycast that handles both cost calculation modes.
RaycastResult raycastBase(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter,
  bool calculateCosts,
  int prevRef,
) {
  final result = RaycastResult(
    t: 0.0,
    hitNormal: Vector3(0.0, 0.0, 0.0),
    hitEdgeIndex: -1,
    path: [],
    pathCost: 0.0,
  );

  if (!isValidNodeRef(navMesh, startNodeRef) || !startPosition.isFinite() || !endPosition.isFinite() || filter == null) {
    return result;
  }

  int? curRef = startNodeRef;
  int prevRefTracking = prevRef;
  final intersectResult = createIntersectSegmentPoly2DResult();

  if (calculateCosts) {
    _raycastDir.sub2(endPosition, startPosition);
    _raycastCurPos.setFrom(startPosition);
  }

  while (curRef != null) {
    final GetTileAndPolyByRefResult tileAndPolyResult = getTileAndPolyByRef(navMesh,curRef);
    if (tileAndPolyResult.success != true) break;

    final NavMeshTile? tile = tileAndPolyResult.tile;
    final NavMeshPoly? poly = tileAndPolyResult.poly;
    final int nv = poly?.vertices.length ?? 0;
    final List<int> polyVertices = List<int>.from(poly!.vertices);

    if (_raycastVertices.length < nv * 3) {
      _raycastVertices.addAll(List<double>.filled((nv * 3) - _raycastVertices.length, 0.0));
    }

    final List<double> tileVerts = List<double>.from(tile!.vertices);
    for (int i = 0; i < nv; i++) {
      final int start = polyVertices[i] * 3;
      _raycastVertices[i * 3] = tileVerts[start];
      _raycastVertices[i * 3 + 1] = tileVerts[start + 1];
      _raycastVertices[i * 3 + 2] = tileVerts[start + 2];
    }

    intersectSegmentPoly2D(intersectResult, startPosition, endPosition, nv, _raycastVertices);
    if (intersectResult.intersects != true) return result;

    result.hitEdgeIndex = intersectResult.segMax;
    if (intersectResult.tmax > result.t) {
      result.t = intersectResult.tmax;
    }

    result.path.add(curRef);

    if (intersectResult.segMax == -1) {
      result.t = double.maxFinite; // Maps from JS Number.MAX_VALUE
      return result;
    }

    int? nextRef;
    final NavMeshNode? curNode = getNodeByRef(navMesh, curRef);
    final int targetSegMax = intersectResult.segMax;

    for (final linkIndex in curNode?.links ?? []) {
      final NavMeshLink? link = navMesh.links[linkIndex];
      if (link?.edge != targetSegMax) continue;
      if (getNodeRefType(link!.toNodeRef) == NodeType.offMesh.value) continue;

      final GetTileAndPolyByRefResult nextTileAndPolyResult = getTileAndPolyByRef(navMesh,link.toNodeRef);
      if (nextTileAndPolyResult.success != true) continue;
      if (filter.passFilter(link.toNodeRef, navMesh) == false) continue;

      if (link.side == 0xff) {
        nextRef = link.toNodeRef;
        break;
      }

      if (link.bmin == 0 && link.bmax == 255) {
        nextRef = link.toNodeRef;
        break;
      }

      final int v0 = polyVertices[link.edge];
      final int v1 = polyVertices[(link.edge + 1) % nv];
      final List<double> left = [tileVerts[v0 * 3], tileVerts[v0 * 3 + 1], tileVerts[v0 * 3 + 2]];
      final List<double> right = [tileVerts[v1 * 3], tileVerts[v1 * 3 + 1], tileVerts[v1 * 3 + 2]];

      const double s = 1.0 / 255.0;
      if (link.side == 0 || link.side == 4) {
        double lmin = left[2] + (right[2] - left[2]) * ((link.bmin as int) * s);
        double lmax = left[2] + (right[2] - left[2]) * ((link.bmax as int) * s);
        if (lmin > lmax) { final double t = lmin; lmin = lmax; lmax = t; }

        final double z = startPosition.z + (endPosition.z - startPosition.z) * intersectResult.tmax;
        if (z >= lmin && z <= lmax) { nextRef = link.toNodeRef; break; }
      } else if (link.side == 2 || link.side == 6) {
        double lmin = left[0] + (right[0] - left[0]) * ((link.bmin as int) * s);
        double lmax = left[0] + (right[0] - left[0]) * ((link.bmax as int) * s);
        if (lmin > lmax) { final double t = lmin; lmin = lmax; lmax = t; }

        final double x = startPosition.x + (endPosition.x - startPosition.x) * intersectResult.tmax;
        if (x >= lmin && x <= lmax) { nextRef = link.toNodeRef; break; }
      }
    }

    if (nextRef == null) {
      if (targetSegMax >= 0) {
        final int a = targetSegMax;
        final int b = (targetSegMax + 1 < nv) ? targetSegMax + 1 : 0;
        
        _raycastHitNormalVa.setValues(_raycastVertices[a * 3], _raycastVertices[a * 3 + 1], _raycastVertices[a * 3 + 2]);
        _raycastHitNormalVb.setValues(_raycastVertices[b * 3], _raycastVertices[b * 3 + 1], _raycastVertices[b * 3 + 2]);
        
        final double dx = _raycastHitNormalVb.x - _raycastHitNormalVa.x;
        final double dz = _raycastHitNormalVb.z - _raycastHitNormalVa.z;
        
        result.hitNormal.setValues(dz, 0.0, -dx).normalize();
      }
      return result;
    }

    if (calculateCosts) {
      _raycastLastPos.setFrom(_raycastCurPos);
      _raycastCurPos.setValues(startPosition.x, startPosition.y, startPosition.z).addScaled(_raycastDir, result.t);

      final int e0 = polyVertices[targetSegMax];
      final int e1 = polyVertices[(targetSegMax + 1) % nv];
      
      _raycastE1Vec.setValues(tileVerts[e1 * 3], tileVerts[e1 * 3 + 1], tileVerts[e1 * 3 + 2]);
      _raycastE0Vec.setValues(tileVerts[e0 * 3], tileVerts[e0 * 3 + 1], tileVerts[e0 * 3 + 2]);
      
      _raycastEDir.sub2(_raycastE1Vec, _raycastE0Vec);
      _raycastDiff.sub2(_raycastCurPos, _raycastE0Vec);

      final double sFactor = (_raycastEDir.x * _raycastEDir.x > _raycastEDir.z * _raycastEDir.z) 
          ? _raycastDiff.x / _raycastEDir.x 
          : _raycastDiff.z / _raycastEDir.z;
          
      _raycastCurPos.y = _raycastE0Vec.y + _raycastEDir.y * sFactor;
      result.pathCost += filter.getCost(_raycastLastPos, _raycastCurPos, navMesh, prevRefTracking, curRef, nextRef);
    }

    prevRefTracking = curRef;
    curRef = nextRef;
  }
  return result;
}

/// Casts a 'walkability' ray along the surface of the navigation mesh from 
/// the start position toward the end position (without cost calculation).
RaycastResult raycast(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter,
) {
  return raycastBase(navMesh, startNodeRef, startPosition, endPosition, filter, false, 0);
}

/// Casts a 'walkability' ray along the surface of the navigation mesh from 
/// the start position toward the end position, calculating accumulated path costs.
RaycastResult raycastWithCosts(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 startPosition,
  Vector3 endPosition,
  QueryFilter? filter,
  int prevRef,
) {
  return raycastBase(navMesh, startNodeRef, startPosition, endPosition, filter, true, prevRef);
}

class FindRandomPointResult {
  bool success;
  int nodeRef; // NodeRef target
  final Vector3 position;

  FindRandomPointResult({
    required this.success,
    required this.nodeRef,
    required this.position,
  });
}

// Low allocation layout scratchpads to bypass loop runtime allocation footprints
final List<double> _findRandomPointVertices = [];
final Vector3 _findRandomPointVa = Vector3();
final Vector3 _findRandomPointVb = Vector3();
final Vector3 _findRandomPointVc = Vector3();

/// Finds a random point on the navigation mesh using a stochastic reservoir area selection method.
FindRandomPointResult findRandomPoint(NavMesh navMesh, QueryFilter? filter, double Function() rand) {
  final result = FindRandomPointResult(
    success: false,
    nodeRef: 0,
    position: Vector3(0.0, 0.0, 0.0),
  );

  // Randomly pick one tile using uniform reservoir sampling weighting
  NavMeshTile? selectedTile;
  double tileSum = 0.0;
  
  // Handling standard dynamic map parsing matching the tiles database structure
  final tiles = navMesh.tiles.values.toList();
  
  for (final tile in tiles) {
    if (tile == null || tile.polys == null) continue;
    const double area = 1.0;
    tileSum += area;
    
    final double u = rand();
    if (u * tileSum <= area) {
      selectedTile = tile;
    }
  }

  if (selectedTile == null) return result;

  // Randomly pick one polygon weighted precisely by its geometric area profile
  NavMeshPoly? selectedPoly;
  int? selectedPolyRef;
  double areaSum = 0.0;
  
  final selectedTilePolys = selectedTile.polys;
  final List<double> tileVerts = selectedTile.vertices;

  for (int i = 0; i < (selectedTilePolys?.length ?? 0); i++) {
    final poly = selectedTilePolys![i];
    final NavMeshNode node = getNodeByTileAndPoly(navMesh, selectedTile, i)!;
    
    if (filter?.passFilter(node.ref, navMesh) == false) continue;

    // Calculate area of the polygon using fan triangulation loops
    double polyArea = 0.0;
    final List<int> polyVertices = List<int>.from(poly.vertices as Iterable);
    
    for (int j = 2; j < polyVertices.length; j++) {
      final int v0Idx = polyVertices[0] * 3;
      final int vj1Idx = polyVertices[j - 1] * 3;
      final int vjIdx = polyVertices[j] * 3;
      
      _findRandomPointVa.setValues(tileVerts[v0Idx], tileVerts[v0Idx + 1], tileVerts[v0Idx + 2]);
      _findRandomPointVb.setValues(tileVerts[vj1Idx], tileVerts[vj1Idx + 1], tileVerts[vj1Idx + 2]);
      _findRandomPointVc.setValues(tileVerts[vjIdx], tileVerts[vjIdx + 1], tileVerts[vjIdx + 2]);
      
      // triArea2D comes from your previous geometric mathematics segments
      polyArea += triArea2D(_findRandomPointVa, _findRandomPointVb, _findRandomPointVc);
    }

    // Select random polygon using continuous reservoir aggregation metrics
    areaSum += polyArea;
    final double u = rand();
    if (u * areaSum <= polyArea) {
      selectedPoly = poly;
      selectedPolyRef = node.ref;
    }
  }

  if (selectedPoly == null || selectedPolyRef == null) return result;

  // Extract polygon coordinates out into flat array matrices
  final List<int> selectedPolyVertices = List<int>.from(selectedPoly.vertices as Iterable);
  final int nv = selectedPolyVertices.length;
  
  if (_findRandomPointVertices.length < nv * 3) {
    _findRandomPointVertices.addAll(List<double>.filled((nv * 3) - _findRandomPointVertices.length, 0.0));
  }

  for (int j = 0; j < nv; j++) {
    final int start = selectedPolyVertices[j] * 3;
    _findRandomPointVertices[j * 3] = tileVerts[start];
    _findRandomPointVertices[j * 3 + 1] = tileVerts[start + 1];
    _findRandomPointVertices[j * 3 + 2] = tileVerts[start + 2];
  }

  final double s = rand();
  final double t = rand();
  final List<double> areas = List<double>.filled(nv, 0.0);
  final Vector3 pt = Vector3(0.0, 0.0, 0.0);
  
  // Distributed calculation mapping straight to standard three_js_math registers
  randomPointInConvexPoly(pt, nv, _findRandomPointVertices, areas, s, t);

  // Project point back down onto mesh surfaces to correct planar tracking heights
  final GetClosestPointOnPolyResult closestPointResult = createGetClosestPointOnPolyResult();
  getClosestPointOnPoly(closestPointResult, navMesh, selectedPolyRef, pt);

  if (closestPointResult.success == true) {
    result.position.setFrom(closestPointResult.position);
  } else {
    result.position.setFrom(pt);
  }

  result.nodeRef = selectedPolyRef;
  result.success = true;
  return result;
}

// Caching infrastructure tracking the trailing circle sample references
final List<double> _findRandomPointAroundCircleVertices = [];
final Vector3 _findRandomPointAroundCircleV0 = Vector3();
final Vector3 _findRandomPointAroundCircleV1 = Vector3();
final Vector3 _findRandomPointAroundCircleV2 = Vector3();
final DistancePtSegSqr2dResult _findRandomPointAroundCircleDistancePtSegSqr2dResult = createDistancePtSegSqr2dResult();

class FindRandomPointAroundCircleResult {
  bool success;
  int nodeRef; // NodeRef target
  final Vector3 position;
  FindRandomPointAroundCircleResult({required this.success, required this.nodeRef, required this.position});
}

/// Finds a random point around a center position on the navigation mesh.
/// Exploits reachable polygons using a Dijkstra-like search up to maxRadius,
/// then applies reservoir sampling to select an area-weighted point.
FindRandomPointAroundCircleResult findRandomPointAroundCircle(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 position,
  double maxRadius,
  QueryFilter? filter,
  double Function() rand,
) {
  final result = FindRandomPointAroundCircleResult(
    success: false,
    nodeRef: 0,
    position: Vector3(0.0, 0.0, 0.0),
  );

  // Validate structural spatial inputs before committing frame resources
  if (!isValidNodeRef(navMesh, startNodeRef) || !position.isFinite() || maxRadius < 0 || !maxRadius.isFinite) {
    return result;
  }

  final GetTileAndPolyByRefResult startTileAndPoly = getTileAndPolyByRef(navMesh,startNodeRef);
  if (startTileAndPoly.success != true) return result;

  // Verify whether the entry search target passes the exclusion filter
  if (filter?.passFilter(startNodeRef, navMesh) == false) return result;

  final SearchNodePool nodes = {};
  final SearchNodeQueue openList = [];

  final SearchNode startNode = SearchNode(
    cost: 0.0,
    total: 0.0,
    parentNodeRef: null,
    parentState: null,
    nodeRef: startNodeRef,
    state: 0,
    flags: nodeFlagOpen,
    position: Vector3(position.x, position.y, position.z),
  );

  addSearchNode(nodes, startNode);
  pushNodeToQueue(openList, startNode);

  final double radiusSqr = maxRadius * maxRadius;
  double areaSum = 0.0;
  
  NavMeshTile? randomTile;
  NavMeshPoly? randomPoly;
  int? randomPolyRef;

  final Vector3 va = _findRandomPointAroundCircleV0; // reuses static caching registers
  final Vector3 vb = _findRandomPointAroundCircleV1;

  while (openList.isNotEmpty) {
    final SearchNode bestNode = popNodeFromQueue(openList)!;
    bestNode.flags &= ~nodeFlagOpen;
    bestNode.flags |= nodeFlagClosed;

    final int bestRef = bestNode.nodeRef;
    final GetTileAndPolyByRefResult bestTileAndPoly = getTileAndPolyByRef(navMesh,bestRef);
    if (bestTileAndPoly.success != true) continue;

    final NavMeshTile? bestTile = bestTileAndPoly.tile;
    final NavMeshPoly? bestPoly = bestTileAndPoly.poly;
    final List<int> bestPolyVertices = List<int>.from(bestPoly!.vertices);
    final List<double> tileVerts = List<double>.from(bestTile!.vertices);

    double polyArea = 0.0;
    for (int j = 2; j < bestPolyVertices.length; j++) {
      final int v0Idx = bestPolyVertices[0] * 3;
      final int vj1Idx = bestPolyVertices[j - 1] * 3;
      final int vjIdx = bestPolyVertices[j] * 3;

      _findRandomPointAroundCircleV0.setValues(tileVerts[v0Idx], tileVerts[v0Idx + 1], tileVerts[v0Idx + 2]);
      _findRandomPointAroundCircleV1.setValues(tileVerts[vj1Idx], tileVerts[vj1Idx + 1], tileVerts[vj1Idx + 2]);
      _findRandomPointAroundCircleV2.setValues(tileVerts[vjIdx], tileVerts[vjIdx + 1], tileVerts[vjIdx + 2]);

      polyArea += triArea2D(_findRandomPointAroundCircleV0, _findRandomPointAroundCircleV1, _findRandomPointAroundCircleV2);
    }

    // Reservoir sampling pass weighted by submesh triangulation areas
    areaSum += polyArea;
    final double u = rand();
    if (u * areaSum <= polyArea) {
      randomTile = bestTile;
      randomPoly = bestPoly;
      randomPolyRef = bestRef;
    }

    final int? parentRef = bestNode.parentNodeRef;
    final NavMeshNode? node = getNodeByRef(navMesh, bestRef);

    // Expand search frontier across adjacent link nodes
    for (final linkIndex in node?.links ?? []) {
      final NavMeshLink? link = navMesh.links[linkIndex];
      if (link == null) continue;
      
      final int? neighbourRef = link.toNodeRef;
      if (neighbourRef == null || neighbourRef == parentRef) continue;

      final GetTileAndPolyByRefResult neighbourTileAndPoly = getTileAndPolyByRef(navMesh,neighbourRef);
      if (neighbourTileAndPoly.success != true) continue;
      if (filter?.passFilter(neighbourRef, navMesh) == false) continue;

      // Ensure that the target expansion link boundary intersects the search radius
      if (!getPortalPoints(navMesh, bestRef, neighbourRef, va, vb)) continue;

      final DistancePtSegSqr2dResult res = distancePtSegSqr2d(_findRandomPointAroundCircleDistancePtSegSqr2dResult, position, va, vb);
      if (res.distSqr > radiusSqr) continue;

      SearchNode? neighbourNode = getSearchNode(nodes, neighbourRef, 0);
      if (neighbourNode == null) {
        neighbourNode = SearchNode(
          cost: 0.0,
          total: 0.0,
          parentNodeRef: null,
          parentState: null,
          nodeRef: neighbourRef,
          state: 0,
          flags: 0,
          position: Vector3(0.0, 0.0, 0.0),
        );
        addSearchNode(nodes, neighbourNode);
      }

      if ((neighbourNode.flags & nodeFlagClosed) != 0) continue;

      // Assign position coordinates to edge midpoint on first discovery step
      if (neighbourNode.flags == 0) {
        neighbourNode.position.lerpVectors(va, vb, 0.5);
      }

      final double total = bestNode.total + bestNode.position.distanceTo(neighbourNode.position);

      if ((neighbourNode.flags & nodeFlagOpen) != 0 && total >= neighbourNode.total) continue;

      neighbourNode.parentNodeRef = bestRef;
      neighbourNode.parentState = 0;
      neighbourNode.flags &= ~nodeFlagClosed;
      neighbourNode.total = total;

      if ((neighbourNode.flags & nodeFlagOpen) != 0) {
        reindexNodeInQueue(openList, neighbourNode);
      } else {
        neighbourNode.flags = nodeFlagOpen;
        pushNodeToQueue(openList, neighbourNode);
      }
    }
  }

  if (randomPoly == null || randomTile == null || randomPolyRef == null) return result;

  final List<int> randomPolyVertices = List<int>.from(randomPoly.vertices as Iterable);
  final int nv = randomPolyVertices.length;
  final List<double> randomTileVerts = List<double>.from(randomTile.vertices as Iterable);

  if (_findRandomPointAroundCircleVertices.length < nv * 3) {
    _findRandomPointAroundCircleVertices.addAll(List<double>.filled((nv * 3) - _findRandomPointAroundCircleVertices.length, 0.0));
  }

  for (int j = 0; j < nv; j++) {
    final int start = randomPolyVertices[j] * 3;
    _findRandomPointAroundCircleVertices[j * 3] = randomTileVerts[start];
    _findRandomPointAroundCircleVertices[j * 3 + 1] = randomTileVerts[start + 1];
    _findRandomPointAroundCircleVertices[j * 3 + 2] = randomTileVerts[start + 2];
  }

  final double s = rand();
  final double t = rand();
  final List<double> areas = List<double>.filled(nv, 0.0);
  final Vector3 pt = Vector3(0.0, 0.0, 0.0);

  randomPointInConvexPoly(pt, nv, _findRandomPointAroundCircleVertices, areas, s, t);

  // Re-project the coordinate down to guarantee it aligns exactly to the mesh surface
  final GetClosestPointOnPolyResult closestPointResult = createGetClosestPointOnPolyResult();
  getClosestPointOnPoly(closestPointResult, navMesh, randomPolyRef, pt);

  if (closestPointResult.success == true) {
    result.position.setFrom(closestPointResult.position);
  } else {
    result.position.setFrom(pt);
  }

  result.nodeRef = randomPolyRef;
  result.success = true;
  return result;
}
