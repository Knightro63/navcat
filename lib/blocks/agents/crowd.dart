import 'dart:math' as math;
import 'package:navcat/blocks/agents/local_boundary.dart';
import 'package:navcat/blocks/agents/obstacle_avoidance.dart';
import 'package:navcat/blocks/agents/path_corridor.dart';
import 'package:three_js_math/three_js_math.dart';
import '../../navcat.dart'; // Adjust path depending on your directory tree

enum AgentState {
  invalid,
  walking,
  offMesh,
}

enum AgentTargetState {
  none,
  failed,
  valid,
  requesting,
  waitingForQueue,
  waitingForPath,
  velocity,
}

class CrowdUpdateFlags {
  static const int anticipateTurns = 1;
  static const int obstacleAvoidance = 2;
  static const int separation = 4;
  static const int optimizeVis = 8;
  static const int optimizeTopo = 16;
}

class AgentParams {
  final double radius;
  final double height;
  final double maxAcceleration;
  final double maxSpeed;
  final double collisionQueryRange;
  final double pathOptimizationRange;
  final double separationWeight;
  final int updateFlags;
  final QueryFilter queryFilter;
  final ObstacleAvoidanceParams obstacleAvoidance;
  final bool autoTraverseOffMeshConnections;
  final bool debugObstacleAvoidance;

  AgentParams({
    required this.radius,
    required this.height,
    required this.maxAcceleration,
    required this.maxSpeed,
    required this.collisionQueryRange,
    double? pathOptimizationRange,
    required this.separationWeight,
    required this.updateFlags,
    required this.queryFilter,
    ObstacleAvoidanceParams? obstacleAvoidance,
    this.autoTraverseOffMeshConnections = true,
    this.debugObstacleAvoidance = false,
  })  : this.pathOptimizationRange = pathOptimizationRange ?? (radius * 30.0),
        this.obstacleAvoidance = obstacleAvoidance ?? ObstacleAvoidanceParams();
}

class AgentNeighbor {
  final String agentId;
  final double dist;

  AgentNeighbor({
    required this.agentId,
    required this.dist,
  });
}

class OffMeshAnimation {
  double t;
  final Vector3 startPosition;
  final Vector3 endPosition;
  int nodeRef;
  double duration;

  OffMeshAnimation({
    required this.t,
    required this.startPosition,
    required this.endPosition,
    required this.nodeRef,
    required this.duration,
  });
}

class Agent {
  double radius;
  double height;
  double maxAcceleration;
  double maxSpeed;
  double collisionQueryRange;
  double pathOptimizationRange;
  double separationWeight;
  int updateFlags;
  QueryFilter queryFilter;
  ObstacleAvoidanceParams obstacleAvoidance;
  bool autoTraverseOffMeshConnections;
  AgentState state;
  PathCorridor corridor;
  LocalBoundary boundary;
  SlicedNodePathQuery slicedQuery;
  ObstacleAvoidanceQuery obstacleAvoidanceQuery;
  ObstacleAvoidanceDebugData? obstacleAvoidanceDebugData;
  List<AgentNeighbor> neis;
  List<StraightPathPoint> corners;
  final Vector3 position;
  double desiredSpeed;
  final Vector3 desiredVelocity;
  final Vector3 newVelocity;
  final Vector3 velocity;
  final Vector3 displacement;
  AgentTargetState targetState;
  int? targetRef; // Nullable int for NodeRef | null
  final Vector3 targetPosition;
  bool targetReplan;
  double targetPathfindingTime;
  bool targetPathIsPartial;
  double topologyOptTime;
  OffMeshAnimation? offMeshAnimation;

  Agent({
    required this.radius,
    required this.height,
    required this.maxAcceleration,
    required this.maxSpeed,
    required this.collisionQueryRange,
    required this.pathOptimizationRange,
    required this.separationWeight,
    required this.updateFlags,
    required this.queryFilter,
    required this.obstacleAvoidance,
    required this.autoTraverseOffMeshConnections,
    required this.state,
    required this.corridor,
    required this.boundary,
    required this.slicedQuery,
    required this.obstacleAvoidanceQuery,
    this.obstacleAvoidanceDebugData,
    required this.neis,
    required this.corners,
    Vector3? position,
    this.desiredSpeed = 0.0,
    Vector3? desiredVelocity,
    Vector3? newVelocity,
    Vector3? velocity,
    Vector3? displacement,
    required this.targetState,
    this.targetRef,
    Vector3? targetPosition,
    this.targetReplan = false,
    this.targetPathfindingTime = 0.0,
    this.targetPathIsPartial = false,
    this.topologyOptTime = 0.0,
    this.offMeshAnimation,
  })  : this.position = position ?? Vector3(),
        this.desiredVelocity = desiredVelocity ?? Vector3(),
        this.newVelocity = newVelocity ?? Vector3(),
        this.velocity = velocity ?? Vector3(),
        this.displacement = displacement ?? Vector3(),
        this.targetPosition = targetPosition ?? Vector3();
}

class Crowd {
  final Map<String, Agent> agents;
  int agentIdCounter;
  double maxAgentRadius;
  final Vector3 agentPlacementHalfExtents;
  int maxIterationsPerUpdate;
  int maxIterationsPerAgent;
  int quickSearchIterations;

  Crowd({
    required this.agents,
    required this.agentIdCounter,
    required this.maxAgentRadius,
    Vector3? agentPlacementHalfExtents,
    this.maxIterationsPerUpdate = 600,
    this.maxIterationsPerAgent = 200,
    this.quickSearchIterations = 20,
  }) : this.agentPlacementHalfExtents = agentPlacementHalfExtents ?? Vector3(maxAgentRadius, maxAgentRadius, maxAgentRadius);

  /// Factory helper matching your Javascript initialization method
  factory Crowd.create(double maxAgentRadius) {
    return Crowd(
      agents: {},
      agentIdCounter: 0,
      maxAgentRadius: maxAgentRadius,
    );
  }
}

/**
 * Adds an agent to the crowd.
 * Returns the ID of the added agent.
 */
String addAgent(Crowd crowd, NavMesh navMesh, Vector3 position, AgentParams agentParams) {
  final String agentId = (crowd.agentIdCounter++).toString();
  
  final agent = Agent(
    radius: agentParams.radius,
    height: agentParams.height,
    maxAcceleration: agentParams.maxAcceleration,
    maxSpeed: agentParams.maxSpeed,
    collisionQueryRange: agentParams.collisionQueryRange,
    pathOptimizationRange: agentParams.pathOptimizationRange,
    separationWeight: agentParams.separationWeight,
    updateFlags: agentParams.updateFlags,
    queryFilter: agentParams.queryFilter,
    obstacleAvoidance: agentParams.obstacleAvoidance,
    autoTraverseOffMeshConnections: agentParams.autoTraverseOffMeshConnections,
    state: AgentState.walking,
    corridor: PathCorridor(),
    slicedQuery: createSlicedNodePathQuery(),
    boundary: LocalBoundary.create(),
    obstacleAvoidanceQuery: ObstacleAvoidanceQuery(maxCircles: 32, maxSegments: 32),
    obstacleAvoidanceDebugData: agentParams.debugObstacleAvoidance ? ObstacleAvoidanceDebugData() : null,
    neis: [],
    corners: [],
    position: position.clone(),
    desiredSpeed: 0.0,
    desiredVelocity: Vector3(0, 0, 0),
    newVelocity: Vector3(0, 0, 0),
    velocity: Vector3(0, 0, 0),
    displacement: Vector3(0, 0, 0),
    targetState: AgentTargetState.none,
    targetRef: null,
    targetPosition: Vector3(0, 0, 0),
    targetReplan: false,
    targetPathfindingTime: 0.0,
    targetPathIsPartial: false,
    topologyOptTime: 0.0,
    offMeshAnimation: null,
  );

  crowd.agents[agentId] = agent;

  // Find nearest position on navmesh and place the agent there
  final nearestPolyResult = createFindNearestPolyResult();
  findNearestPoly(nearestPolyResult, navMesh, position, crowd.agentPlacementHalfExtents, agent.queryFilter);

  Vector3 nearestPos = position;
  int nearestRef = invalidNodeRef;

  if (nearestPolyResult.success) {
    nearestPos = nearestPolyResult.position;
    nearestRef = nearestPolyResult.nodeRef;
  }

  // Reset corridor with the found (or invalid) reference
  agent.position.setFrom(nearestPos);
  agent.corridor.reset(nearestRef, nearestPos);

  // Set agent state based on whether we found a valid polygon
  if (nearestRef != invalidNodeRef) {
    agent.state = AgentState.walking;
  } else {
    agent.state = AgentState.invalid;
  }

  return agentId;
}

/**
 * Removes an agent from the crowd.
 * Returns true if the agent was removed, false otherwise.
 */
bool removeAgent(Crowd crowd, String agentId) {
  if (crowd.agents.containsKey(agentId)) {
    crowd.agents.remove(agentId);
    return true;
  }
  return false;
}

/**
 * Requests a move target for an agent.
 * Returns true if the move target was set, false otherwise.
 */
bool requestMoveTarget(Crowd crowd, String agentId, int targetRef, Vector3 targetPos) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;

  agent.targetRef = targetRef;
  agent.targetPosition.setFrom(targetPos);

  // If the agent already has a corridor path, this is a replan
  agent.targetReplan = false;
  agent.targetState = AgentTargetState.requesting;
  agent.targetPathIsPartial = false;
  agent.targetPathfindingTime = 0.0; // Reset timer for new request
  
  return true;
}

bool requestMoveTargetReplan(Crowd crowd, String agentId, int? targetRef, Vector3 targetPos) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;

  agent.targetRef = targetRef;
  agent.targetPosition.setFrom(targetPos);
  agent.targetReplan = true;
  agent.targetState = AgentTargetState.requesting;
  
  return true;
}

/**
 * Request a move velocity for an agent.
 * Returns true if the move velocity was set, false otherwise.
 */
bool requestMoveVelocity(Crowd crowd, String agentId, Vector3 velocity) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;

  agent.targetPosition.setFrom(velocity);
  agent.targetState = AgentTargetState.velocity;
  agent.targetReplan = false;
  agent.targetRef = invalidNodeRef;
  
  return true;
}

/**
 * Reset the move target for an agent.
 * Returns true if the move target was reset, false otherwise.
 */
bool resetMoveTarget(Crowd crowd, String agentId) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;

  agent.targetRef = null;
  agent.targetPosition.setValues(0, 0, 0);
  agent.desiredVelocity.setValues(0, 0, 0);
  agent.targetReplan = false;
  agent.targetState = AgentTargetState.none;
  agent.targetPathIsPartial = false;
  
  return true;
}

const int CHECK_LOOKAHEAD = 10;
const double TARGET_REPLAN_DELAY_SECONDS = 1.0;

// Globally cached instantiation to prevent performance/allocation spikes in loops
final _checkPathValidityNearestPolyResult = createFindNearestPolyResult();

void checkPathValidity(Crowd crowd, NavMesh navMesh, double deltaTime) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    
    if (agent.state != AgentState.walking) continue;

    agent.targetPathfindingTime += deltaTime;
    bool replan = false;

    // First check that the current location is valid
    final int agentNodeRef = agent.corridor.path.first;
    
    if (!isValidNodeRef(navMesh, agentNodeRef)) {
      final nearestPolyResult = findNearestPoly(
        _checkPathValidityNearestPolyResult,
        navMesh,
        agent.position,
        crowd.agentPlacementHalfExtents,
        agent.queryFilter,
      );

      if (!nearestPolyResult.success) {
        agent.state = AgentState.invalid;
        agent.corridor.reset(invalidNodeRef, agent.position);
        resetLocalBoundary(agent.boundary);
        continue;
      }

      fixPathStart(agent.corridor, nearestPolyResult.nodeRef, nearestPolyResult.position);
      resetLocalBoundary(agent.boundary);
      agent.position.setFrom(nearestPolyResult.position);
      replan = true;
    }

    // If the agent doesn't have a move target, or is controlled by velocity, skip target recovery
    if (agent.targetState == AgentTargetState.none || agent.targetState == AgentTargetState.velocity) {
      continue;
    }

    // Try to recover move request position
    if (agent.targetState != AgentTargetState.none && agent.targetState != AgentTargetState.failed) {
      final targetRef = agent.targetRef;
      
      if (targetRef == null || 
          !isValidNodeRef(navMesh, targetRef) || 
          !agent.queryFilter.passFilter(targetRef, navMesh)) {
        
        // Current target is not valid, try to reposition
        final nearestPolyResult = findNearestPoly(
          _checkPathValidityNearestPolyResult,
          navMesh,
          agent.targetPosition,
          crowd.agentPlacementHalfExtents,
          agent.queryFilter,
        );

        if (!nearestPolyResult.success) {
          // Could not find location in navmesh, set agent state to invalid
          agent.targetState = AgentTargetState.none;
          agent.targetRef = null;
          agent.corridor.reset(invalidNodeRef, agent.position);
        } else {
          // Target poly became invalid, update to nearest valid poly
          agent.targetRef = nearestPolyResult.nodeRef;
          agent.targetPosition.setFrom(nearestPolyResult.position);
          replan = true;
        }
      }
    }

    // If nearby corridor is not valid, replan
    final bool corridorValid = corridorIsValid(agent.corridor, CHECK_LOOKAHEAD, navMesh, agent.queryFilter);
    if (!corridorValid) {
      replan = true;
    }

    // If the end of the path is near and it is not the requested location, replan
    if (agent.targetState == AgentTargetState.valid) {
      if (agent.targetPathfindingTime > TARGET_REPLAN_DELAY_SECONDS &&
          agent.corridor.path.length < CHECK_LOOKAHEAD &&
          agent.corridor.path.last != agent.targetRef) {
        replan = true;
      }
    }

    // Try to replan path to goal
    if (replan && agent.targetState != AgentTargetState.none) {
      requestMoveTargetReplan(crowd, agentId, agent.targetRef, agent.targetPosition);
    }
  }
}

void updateMoveRequests(Crowd crowd, NavMesh navMesh, double deltaTime) {
  // First, update pathfinding time for all agents in waitingForPath state
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.targetState == AgentTargetState.waitingForPath) {
      agent.targetPathfindingTime += deltaTime;
    }
  }

  // Collect all agents that need pathfinding processing
  final List<String> pathfindingAgents = [];

  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    
    if (agent.state == AgentState.invalid ||
        agent.targetState == AgentTargetState.none ||
        agent.targetState == AgentTargetState.velocity ||
        agent.targetRef == null) {
      continue;
    }

    if (agent.targetState == AgentTargetState.requesting) {
      // Init the pathfinding query and state
      initSlicedFindNodePath(
        navMesh,
        agent.slicedQuery,
        agent.corridor.path.first,
        agent.targetRef!,
        agent.position,
        agent.targetPosition,
        agent.queryFilter,
      );

      // Quick search 
      updateSlicedFindNodePath(navMesh, agent.slicedQuery, crowd.quickSearchIterations);

      // Finalize the partial path from quick search
      final partialResult = agent.targetReplan
          ? finalizeSlicedFindNodePathPartial(navMesh, agent.slicedQuery, agent.corridor.path)
          : finalizeSlicedFindNodePath(navMesh, agent.slicedQuery);

      final List<int> reqPath = partialResult.path;
      final int reqPathCount = reqPath.length;
      Vector3 reqPos = agent.targetPosition.clone();

      // If we got a path from the quick search
      if (reqPathCount > 0) {
        // Check if this is a partial path (didn't reach target)
        if (reqPath.last != agent.targetRef) {
          // Partial path - constrain target position inside the last polygon
          final closestPointResult = createGetClosestPointOnPolyResult();
          final closestPoint = getClosestPointOnPoly(
            closestPointResult,
            navMesh,
            reqPath.last,
            agent.targetPosition,
          );

          if (closestPoint.success) {
            reqPos = closestPoint.position;
          } else {
            // Failed to constrain position, fall back to current position
            reqPos = agent.position.clone();
            reqPath[0] = agent.corridor.path.first;
            reqPath.length = 1;
          }
        }
      } else {
        // Could not find any path, start the request from current location
        reqPos = agent.position.clone();
        reqPath[0] = agent.corridor.path.first;
        reqPath.length = 1;
      }

      // Immediately set the corridor with the partial path
      agent.corridor.setPath(reqPos, reqPath);
      resetLocalBoundary(agent.boundary);
      agent.targetPathIsPartial = false;

      // Check if we reached the target with the quick search
      if (reqPathCount > 0 && reqPath.last == agent.targetRef) {
        // Reached target - we're done!
        agent.targetState = AgentTargetState.valid;
        agent.targetPathfindingTime = 0.0;
      } else {
        // Partial path - queue for full pathfinding
        agent.targetState = AgentTargetState.waitingForQueue;
      }
    }
  }

  // Process agents in waitingForQueue - transition all waiting agents to full pathfinding
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    
    if (agent.targetState == AgentTargetState.waitingForQueue) {
      // Initialize the full pathfinding query once when entering waitingForPath
      // Start from the last polygon in the corridor (where partial path ended)
      final List<int> corridorPath = agent.corridor.path;
      final int startRef = corridorPath.last;

      initSlicedFindNodePath(
        navMesh,
        agent.slicedQuery,
        startRef,
        agent.targetRef!,
        agent.corridor.target,
        agent.targetPosition,
        agent.queryFilter,
      );

      agent.targetState = AgentTargetState.waitingForPath;
      pathfindingAgents.add(agentId);
    } else if (agent.targetState == AgentTargetState.waitingForPath) {
      pathfindingAgents.add(agentId);
    }
  }

  // Sort agents by targetPathfindingTime (longest waiting gets priority)
  pathfindingAgents.sort((a, b) {
    final double timeA = crowd.agents[a]!.targetPathfindingTime;
    final double timeB = crowd.agents[b]!.targetPathfindingTime;
    return timeB.compareTo(timeA);
  });

  // Distribute global iteration budget across prioritized agents
  int remainingIterations = crowd.maxIterationsPerUpdate;

  for (final agentId in pathfindingAgents) {
    final agent = crowd.agents[agentId]!;

    if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.inProgress) != 0 && remainingIterations > 0) {
      // Allocate iterations for this agent (minimum 1, maximum remaining)
      final int iterationsForAgent = math.min(crowd.maxIterationsPerAgent, remainingIterations);
      final int iterationsPerformed = updateSlicedFindNodePath(navMesh, agent.slicedQuery, iterationsForAgent);
      remainingIterations -= iterationsPerformed;
    }

    if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.failure) != 0) {
      // Pathfinding failed
      agent.targetState = AgentTargetState.failed;
      agent.targetPathfindingTime = 0.0;
      agent.targetPathIsPartial = false;
    } else if ((agent.slicedQuery.status & SlicedFindNodePathStatusFlags.success) != 0) {
      // Pathfinding succeeded - now we need to merge the result with the current corridor
      // Check if this is a partial path (best effort)
      agent.targetPathIsPartial = (agent.slicedQuery.status & SlicedFindNodePathStatusFlags.partialResult) != 0;
      
      final result = finalizeSlicedFindNodePath(navMesh, agent.slicedQuery);
      final List<int> newPath = result.path;
      final List<int> currentPath = agent.corridor.path;
      final int currentPathCount = currentPath.length;
      
      bool valid = true;
      Vector3 targetPos = agent.targetPosition.clone();

      // Ensure connect verification
      if (currentPathCount > 0 && newPath.isNotEmpty) {
        if (currentPath[currentPathCount - 1] != newPath[0]) {
          valid = false;
        }
      }

      if (valid) {
        List<int> mergedPath = [];

        // Merge the current corridor path with the new pathfinding result
        if (currentPathCount > 1) {
          // Copy current path excluding last placeholder sequence element
          mergedPath = currentPath.sublist(0, currentPathCount - 1);
          // Append the new path findings
          mergedPath.addAll(newPath);

          // Remove trackbacks loop setup (A -> B -> A sequences)
          for (int j = 0; j < mergedPath.length; j++) {
            if (j - 1 >= 0 && j + 1 < mergedPath.length) {
              if (mergedPath[j - 1] == mergedPath[j + 1]) {
                // Found a trackback: remove the elements using specific range boundaries
                mergedPath.removeRange(j - 1, j + 1);
                j -= 2;
              }
            }
          }
        } else {
          // Current path is just the start polygon, use the new path directly
          mergedPath = List<int>.from(newPath);
        }

        // Check if this is a partial path - constrain target position if needed
        if (mergedPath.isNotEmpty && mergedPath.last != agent.targetRef) {
          final int lastPoly = mergedPath.last;
          final closestPointResult = createGetClosestPointOnPolyResult();
          final closestPoint = getClosestPointOnPoly(closestPointResult, navMesh, lastPoly, agent.targetPosition);
          
          if (closestPoint.success) {
            targetPos = closestPoint.position;
          } else {
            valid = false;
          }
        }

        if (valid) {
          agent.corridor.setPath(targetPos, mergedPath);
          resetLocalBoundary(agent.boundary);
          agent.targetState = AgentTargetState.valid;
          agent.targetPathfindingTime = 0.0; // Reset on success
        } else {
          // Retry if target reference is valid
          if (agent.targetRef != null) {
            agent.targetState = AgentTargetState.requesting;
          } else {
            agent.targetState = AgentTargetState.failed;
            agent.targetPathfindingTime = 0.0; // Reset on failure
          }
        }
      } else {
        // Paths don't connect because the agent has moved too far from startup zone
        if (agent.targetRef != null) {
          agent.targetState = AgentTargetState.requesting;
        } else {
          agent.targetState = AgentTargetState.failed;
          agent.targetPathfindingTime = 0.0; // Reset on failure
        }
      }
    }
  }
}

void updateNeighbours(Crowd crowd) {
  // Uniform grid spatial partitioning, rebuilt each frame
  double minX = double.infinity, minZ = double.infinity;
  double maxX = double.negativeInfinity, maxZ = double.negativeInfinity;
  double maxQueryRange = 0.0;
  
  final List<String> agentIds = [];

  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    agent.neis.clear();
    
    if (agent.state != AgentState.walking) continue;
    
    agentIds.add(agentId);
    final x = agent.position.x;
    final z = agent.position.z;
    
    minX = math.min(minX, x);
    maxX = math.max(maxX, x);
    minZ = math.min(minZ, z);
    maxZ = math.max(maxZ, z);
    maxQueryRange = math.max(maxQueryRange, agent.collisionQueryRange);
  }

  if (agentIds.isEmpty) return;

  // Grid cell size = max query range (each agent checks its cell + surrounding 8 cells)
  final double cellSize = maxQueryRange;
  if (cellSize < 0.01) return; // Safety check

  final int gridWidth = ((maxX - minX) / cellSize).ceil() + 1;
  final int gridHeight = ((maxZ - minZ) / cellSize).ceil() + 1;

  // Build grid - flat array with direct indexing (faster than Map)
  final int gridSize = gridWidth * gridHeight;
  final List<List<String>?> grid = List<List<String>?>.filled(gridSize, null);

  // Insert agents into grid
  for (final agentId in agentIds) {
    final agent = crowd.agents[agentId]!;
    final int ix = ((agent.position.x - minX) / cellSize).floor();
    final int iz = ((agent.position.z - minZ) / cellSize).floor();
    final int key = (iz * gridWidth) + ix;
    
    // Bounds safety verification guard step
    if (key < 0 || key >= gridSize) continue;

    List<String>? cell = grid[key];
    if (cell == null) {
      cell = [];
      grid[key] = cell;
    }
    cell.add(agentId);
  }

  // Query neighbors using grid
  for (final agentId in agentIds) {
    final agent = crowd.agents[agentId]!;
    final double queryRangeSqr = agent.collisionQueryRange * agent.collisionQueryRange;
    final int ix = ((agent.position.x - minX) / cellSize).floor();
    final int iz = ((agent.position.z - minZ) / cellSize).floor();

    // Check 3x3 grid around agent (including own cell)
    for (int dz = -1; dz <= 1; dz++) {
      for (int dx = -1; dx <= 1; dx++) {
        final int checkX = ix + dx;
        final int checkZ = iz + dz;

        if (checkX < 0 || checkX >= gridWidth || checkZ < 0 || checkZ >= gridHeight) {
          continue;
        }

        final int cellKey = (checkZ * gridWidth) + checkX;
        final List<String>? cell = grid[cellKey];
        if (cell == null) continue;

        for (final otherAgentId in cell) {
          if (otherAgentId == agentId) continue;
          
          final other = crowd.agents[otherAgentId]!;
          final double distSqr = agent.position.distanceToSquared(other.position);

          if (distSqr < queryRangeSqr) {
            agent.neis.add(AgentNeighbor(
              agentId: otherAgentId, 
              dist: distSqr,
            ));
          }
        }
      }
    }
  }
}

void updateLocalBoundaries(Crowd crowd, NavMesh navMesh) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    
    if (agent.state != AgentState.walking || agent.corridor.path.isEmpty) {
      continue;
    }

    // Update boundary if agent has moved significantly or if boundary is invalid
    final double updateThreshold = agent.collisionQueryRange * 0.25;
    final double movedDistance = agent.position.distanceTo(agent.boundary.center);

    if (movedDistance > updateThreshold || 
        !isLocalBoundaryValid(agent.boundary, navMesh, agent.queryFilter)) {
          
      updateLocalBoundary(
        agent.boundary,
        agent.corridor.path.first,
        agent.position,
        agent.collisionQueryRange,
        navMesh,
        agent.queryFilter,
      );
    }
  }
}

void updateCorners(Crowd crowd, NavMesh navMesh) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;

    if (agent.state != AgentState.walking || 
        agent.targetState == AgentTargetState.none || 
        agent.targetState == AgentTargetState.velocity) {
      agent.desiredVelocity.setValues(0, 0, 0);
      continue;
    }

    if (agent.state != AgentState.walking || agent.targetState != AgentTargetState.valid) {
      agent.desiredVelocity.setValues(0, 0, 0);
      continue;
    }

    // Get corridor corners for steering (returning null inside Dart setup variants instead of false)
    final List<StraightPathPoint>? corners = findCorners(agent.corridor, navMesh, 3);
    if (corners == null) {
      agent.desiredVelocity.setValues(0, 0, 0);
      continue;
    }

    agent.corners = corners;

    // Check to see if the corner after the next corner is directly visible, and short cut to there
    if ((agent.updateFlags & CrowdUpdateFlags.optimizeVis) != 0 && agent.corners.isNotEmpty) {
      final int targetIndex = math.min(1, agent.corners.length - 1);
      final Vector3 target = agent.corners[targetIndex].position;
      
      optimizePathVisibility(
        agent.corridor, 
        target, 
        agent.pathOptimizationRange, 
        navMesh, 
        agent.queryFilter,
      );
    }
  }
}

/**
 * Calculates squared distance between two points in 2D (XZ plane).
 */
double dist2dSqr(Vector3 a, Vector3 b) {
  final double dx = b.x - a.x;
  final double dz = b.z - a.z;
  return (dx * dx) + (dz * dz);
}

bool agentIsOverOffMeshConnection(Agent agent, double radius) {
  if (agent.corners.isEmpty) return false;
  
  final lastCorner = agent.corners.last;
  if (lastCorner.type != NodeType.offMesh.value) return false;
  
  final double dist = dist2dSqr(agent.position, lastCorner.position);
  return dist < (radius * radius);
}

void updateOffMeshConnectionTriggers(Crowd crowd, NavMesh navMesh) {
  // Trigger off mesh connections depending on next corners
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    
    if (agent.state != AgentState.walking ||
        agent.targetState == AgentTargetState.none ||
        agent.targetState == AgentTargetState.velocity) {
      continue;
    }

    final double triggerRadius = agent.radius * 2.25;
    if (agentIsOverOffMeshConnection(agent, triggerRadius)) {
      final int offMeshConnectionNode = agent.corners.last.nodeRef ?? 0;
      if (offMeshConnectionNode == 0) continue; // Assuming 0 represents an invalid or unassigned node reference

      final result = moveOverOffMeshConnection(agent.corridor, offMeshConnectionNode, navMesh);
      if (result == null) continue;

      agent.state = AgentState.offMesh;

      // If autoTraverseOffMeshConnections is true, set up automatic animation
      // Otherwise, still populate the data but the user must call completeOffMeshConnection manually
      agent.offMeshAnimation = OffMeshAnimation(
        t: 0.0,
        duration: agent.autoTraverseOffMeshConnections ? 0.5 : -1.0,
        startPosition: agent.position.clone(),
        endPosition: result.endPosition.clone(),
        nodeRef: result.offMeshNodeRef,
      );
    }
  }
}

/**
 * Manually completes an off-mesh connection for an agent.
 * This should be called after custom off-mesh animation is complete.
 */
bool completeOffMeshConnection(Crowd crowd, String agentId) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;
  if (agent.state != AgentState.offMesh) return false;
  if (agent.offMeshAnimation == null) return false;

  agent.position.setFrom(agent.offMeshAnimation!.endPosition);

  // Update velocity - set to zero during off-mesh connection
  agent.velocity.setValues(0, 0, 0);
  agent.desiredVelocity.setValues(0, 0, 0);

  // Finish animation
  agent.offMeshAnimation = null;

  // Prepare agent for walking
  agent.state = AgentState.walking;
  return true;
}

// Reusable scratch vectors to maintain clean allocation bounds inside the loop
final Vector3 _calcStraightSteerDirectionDirection = Vector3();

/**
 * Calculate straight steering direction (no anticipation).
 * Steers directly toward the first corner.
 */
void calcStraightSteerDirection(Agent agent, List<StraightPathPoint> corners) {
  if (corners.isEmpty) {
    agent.desiredVelocity.setValues(0, 0, 0);
    return;
  }

  final direction = _calcStraightSteerDirectionDirection.sub2(corners[0].position,agent.position);
    
  direction.y = 0.0; // Keep movement on XZ plane
  direction.normalize();

  final double speed = agent.maxSpeed;
  agent.desiredVelocity.addScaled(direction, speed);
}

final Vector3 _calcSmoothSteerDirectionDir0 = Vector3();
final Vector3 _calcSmoothSteerDirectionDir1 = Vector3();
final Vector3 _calcSmoothSteerDirectionDirection = Vector3();

/**
 * Calculate smooth steering direction (with anticipation).
 * Blends between first and second corner for smoother turns.
 */
void calcSmoothSteerDirection(Agent agent, List<StraightPathPoint> corners) {
  if (corners.isEmpty) {
    agent.desiredVelocity.setValues(0, 0, 0);
    return;
  }

  final int ip0 = 0;
  final int ip1 = math.min(1, corners.length - 1);

  final Vector3 p0 = corners[ip0].position;
  final Vector3 p1 = corners[ip1].position;

  final dir0 = _calcSmoothSteerDirectionDir0.sub2(p0,agent.position);
  final dir1 = _calcSmoothSteerDirectionDir1.sub2(p1,agent.position);

  dir0.y = 0.0;
  dir1.y = 0.0;

  final double len0 = dir0.length;
  final double len1 = dir1.length;

  if (len1 > 0.001) {
    dir1.scale(1.0 / len1);
  }

  final direction = _calcSmoothSteerDirectionDirection;
  direction.x = dir0.x - (dir1.x * len0 * 0.5);
  direction.y = 0.0;
  direction.z = dir0.z - (dir1.z * len0 * 0.5);
  
  direction.normalize();

  final double speed = agent.maxSpeed;
  agent.desiredVelocity.addScaled(direction, speed);
}

final Vector2 _getDistanceToGoalStart = Vector2();
final Vector2 _getDistanceToGoalEnd = Vector2();

double getDistanceToGoal(Agent agent, double range) {
  if (agent.corners.isEmpty) return range;

  final endPosition = agent.corners.last;
  final bool isEndOfPath = (endPosition.flags & StraightPathPointFlags.end.value) != 0;
  
  if (!isEndOfPath) return range;

  _getDistanceToGoalStart.setValues(endPosition.position.x, endPosition.position.z);
  _getDistanceToGoalEnd.setValues(agent.position.x, agent.position.z);

  final double dist = _getDistanceToGoalStart.distanceTo(_getDistanceToGoalEnd);
  return math.min(range, dist);
}

// Global cached initialization state buffers for subsequent navigation routines
final Vector3 _updateSteeringSeparationDisp = Vector3();
final Vector3 _updateSteeringSeparationDiff = Vector3();

void updateSteering(Crowd crowd) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;

    if (agent.targetState == AgentTargetState.velocity) {
      agent.desiredVelocity.setFrom(agent.targetPosition);
      continue;
    }

    final bool anticipateTurns = (agent.updateFlags & CrowdUpdateFlags.anticipateTurns) != 0;

    // Calculate steering direction
    if (anticipateTurns) {
      calcSmoothSteerDirection(agent, agent.corners);
    } else {
      calcStraightSteerDirection(agent, agent.corners);
    }

    // Calculate speed scale, handles slowdown at the end of the path
    final double slowDownRadius = agent.radius * 2.0;
    final double speedScale = getDistanceToGoal(agent, slowDownRadius) / slowDownRadius;

    agent.desiredSpeed = agent.maxSpeed;
    agent.desiredVelocity.scale(speedScale);

    // Separation
    if ((agent.updateFlags & CrowdUpdateFlags.separation) != 0) {
      final double separationDist = agent.collisionQueryRange;
      final double invSeparationDist = 1.0 / separationDist;
      final double separationWeight = agent.separationWeight;
      
      double w = 0.0;
      final disp = _updateSteeringSeparationDisp..setValues(0, 0, 0);

      for (int j = 0; j < agent.neis.length; j++) {
        final String neiId = agent.neis[j].agentId;
        final Agent? nei = crowd.agents[neiId];
        if (nei == null) continue;

        // diff = agent.position - nei.position
        final diff = _updateSteeringSeparationDiff.sub2(agent.position,nei.position);
        diff.y = 0.0; // Ignore Y axis

        final double distSqr = diff.length2;
        if (distSqr < 0.00001) continue;
        if (distSqr > (separationDist * separationDist)) continue;

        final double dist = math.sqrt(distSqr);
        final double normalizedDist = dist * invSeparationDist;
        final double weight = separationWeight * (1.0 - (normalizedDist * normalizedDist));

        // disp += diff * (weight / dist)
        disp.addScaled(diff,weight / dist);
        w += 1.0;
      }

      if (w > 0.0001) {
        // Adjust desired velocity: dvel += disp * (1.0 / w)
        agent.desiredVelocity.addScaled(disp, (1.0 / w));

        // Clamp desired velocity to desired speed
        final double speedSqr = agent.desiredVelocity.length2;
        final double desiredSqr = agent.desiredSpeed * agent.desiredSpeed;

        if (speedSqr > desiredSqr && speedSqr > 0.0) {
          agent.desiredVelocity.scale(math.sqrt(desiredSqr / speedSqr));
        }
      }
    }
  }
}

void updateVelocityPlanning(Crowd crowd) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.state != AgentState.walking) continue;

    if ((agent.updateFlags & CrowdUpdateFlags.obstacleAvoidance) != 0) {
      // Reset obstacle query
      agent.obstacleAvoidanceQuery.reset();

      // Add neighboring agents as circular obstacles
      for (final neighbor in agent.neis) {
        final Agent? neighborAgent = crowd.agents[neighbor.agentId];
        if (neighborAgent == null) continue;

        agent.obstacleAvoidanceQuery.addCircleObstacle(
          neighborAgent.position,
          neighborAgent.radius,
          neighborAgent.velocity,
          neighborAgent.desiredVelocity,
        );
      }

      // Add boundary segments as obstacles
      for (final segment in agent.boundary.segments) {
        final List<double> s = segment.s;
        final p1 = Vector3(s[0].toDouble(), s[1].toDouble(), s[2].toDouble());
        final p2 = Vector3(s[3].toDouble(), s[4].toDouble(), s[5].toDouble());

        // Only add segments that are in front of the agent
        final double triArea = (agent.position.x - p1.x) * (p2.z - p1.z) - 
                               (agent.position.z - p1.z) * (p2.x - p1.x);
        if (triArea < 0.0) {
          continue;
        }

        agent.obstacleAvoidanceQuery.addSegmentObstacle(p1, p2);
      }

      // Sample safe velocity using adaptive sampling
      sampleVelocityAdaptive(
        agent.obstacleAvoidanceQuery,
        agent.position,
        agent.radius,
        agent.maxSpeed,
        agent.velocity,
        agent.desiredVelocity,
        agent.obstacleAvoidance,
        agent.newVelocity,
        agent.obstacleAvoidanceDebugData,
      );
    } else {
      // Not using obstacle avoidance, set newVelocity to desiredVelocity
      agent.newVelocity.setFrom(agent.desiredVelocity);
    }
  }
}

final Vector3 _integrateDv = Vector3();

void integrate(Crowd crowd, double deltaTime) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.state != AgentState.walking) continue;

    // Fake dynamic constraint - limit acceleration
    final double maxDelta = agent.maxAcceleration * deltaTime;
    final dv = _integrateDv.sub2(agent.newVelocity,agent.velocity);
    final double ds = dv.length;

    if (ds > maxDelta) {
      dv.scale(maxDelta / ds);
    }

    agent.velocity.add(dv);

    // Integrate position
    if (agent.velocity.length > 0.0001) {
      agent.position.addScaled(agent.velocity,deltaTime);
    } else {
      agent.velocity.setValues(0, 0, 0);
    }
  }
}

final Vector3 _handleCollisionsDiff = Vector3();

void handleCollisions(Crowd crowd) {
  const double COLLISION_RESOLVE_FACTOR = 0.7;
  final List<String> agentIds = crowd.agents.keys.toList();
  final List<Agent> agents = crowd.agents.values.toList();

  for (int iter = 0; iter < 4; iter++) {
    // First pass: calculate displacement for each agent
    for (int i = 0; i < agents.length; i++) {
      final agent = agents[i];
      if (agent.state != AgentState.walking) continue;

      agent.displacement.setValues(0, 0, 0);
      double w = 0.0;

      for (int j = 0; j < agent.neis.length; j++) {
        final String neiAgentId = agent.neis[j].agentId;
        final Agent? nei = crowd.agents[neiAgentId];
        if (nei == null) continue;

        final diff = _handleCollisionsDiff.sub2(agent.position,nei.position);
        diff.y = 0.0; // Ignore Y axis

        final double distSqr = diff.length2;
        final double combinedRadius = agent.radius + nei.radius;

        if (distSqr > (combinedRadius * combinedRadius)) continue;

        final double dist = math.sqrt(distSqr);
        double pen = combinedRadius - dist;

        if (dist < 0.0001) {
          // Agents on top of each other, try to choose diverging separation directions
          final int idx0 = i;
          final int idx1 = agentIds.indexOf(neiAgentId);
          if (idx0 > idx1) {
            diff.setValues(-agent.desiredVelocity.z, 0.0, agent.desiredVelocity.x);
          } else {
            diff.setValues(agent.desiredVelocity.z, 0.0, -agent.desiredVelocity.x);
          }
          pen = 0.01;
        } else {
          pen = (1.0 / dist) * (pen * 0.5) * COLLISION_RESOLVE_FACTOR;
        }

        agent.displacement.addScaled(diff,pen);
        w += 1.0;
      }

      if (w > 0.0001) {
        agent.displacement.scale(1.0 / w);
      }
    }

    // Second pass: apply displacement to all agents
    for (int i = 0; i < agents.length; i++) {
      final agent = agents[i];
      if (agent.state != AgentState.walking) continue;

      agent.position.add(agent.displacement);
    }
  }
}

void updateCorridors(Crowd crowd, NavMesh navMesh) {
  // Update corridors for each agent
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.state != AgentState.walking) continue;

    // Move along navmesh
    movePosition(agent.corridor, agent.position, navMesh, agent.queryFilter);

    // Get valid constrained position back
    agent.position.setFrom(agent.corridor.position);

    // If not using path, truncate the corridor to one poly
    if (agent.targetState == AgentTargetState.none || 
        agent.targetState == AgentTargetState.velocity) {
      agent.corridor.reset(agent.corridor.path.first, agent.position);
    }
  }
}

void offMeshConnectionUpdate(Crowd crowd, double deltaTime) {
  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.offMeshAnimation == null) continue;

    // Only auto-update if autoTraverseOffMeshConnections is enabled
    // Otherwise, the user is responsible for animation and calling completeOffMeshConnection
    if (!agent.autoTraverseOffMeshConnections) continue;

    final anim = agent.offMeshAnimation!;

    // Progress animation time
    anim.t += deltaTime;
    if (anim.t >= anim.duration) {
      // Finish animation
      agent.offMeshAnimation = null;
      // Prepare agent for walking
      agent.state = AgentState.walking;
      continue;
    }

    // Update position via native Vector3 lerp mechanism
    final double progress = anim.t / anim.duration;
    agent.position.setFrom(anim.startPosition).lerp(anim.endPosition, progress);

    // Update velocity - set to zero during off-mesh connection
    agent.velocity.setValues(0, 0, 0);
    agent.desiredVelocity.setValues(0, 0, 0);
  }
}

/// Helper container representing queue structures inside topology optimizations
class TopologyQueueItem {
  final String agentId;
  final double time;

  TopologyQueueItem({required this.agentId, required this.time});
}

void updateTopologyOptimization(Crowd crowd, NavMesh navMesh, double deltaTime) {
  const double OPT_TIME_THR = 0.5; // Seconds
  const int OPT_MAX_AGENTS = 1;
  List<TopologyQueueItem> queue = [];

  for (final agentId in crowd.agents.keys) {
    final agent = crowd.agents[agentId]!;
    if (agent.state != AgentState.walking) continue;
    if (agent.targetState == AgentTargetState.none || 
        agent.targetState == AgentTargetState.velocity) continue;
    if ((agent.updateFlags & CrowdUpdateFlags.optimizeTopo) == 0) continue;

    agent.topologyOptTime += deltaTime;
    if (agent.topologyOptTime >= OPT_TIME_THR) {
      // Insert into queue based on greatest time (longest waiting gets priority)
      if (queue.isEmpty) {
        queue.add(TopologyQueueItem(agentId: agentId, time: agent.topologyOptTime));
      } else if (agent.topologyOptTime <= queue.last.time) {
        if (queue.length < OPT_MAX_AGENTS) {
          queue.add(TopologyQueueItem(agentId: agentId, time: agent.topologyOptTime));
        }
      } else {
        // Find insertion point (sorted by topologyOptTime descending)
        int insertIdx = 0;
        for (int i = 0; i < queue.length; i++) {
          if (agent.topologyOptTime >= queue[i].time) {
            insertIdx = i;
            break;
          }
        }
        queue.insert(insertIdx, TopologyQueueItem(agentId: agentId, time: agent.topologyOptTime));
        
        // Trim to max size cleanly using sublist bounds mapping
        if (queue.length > OPT_MAX_AGENTS) {
          queue = queue.sublist(0, OPT_MAX_AGENTS);
        }
      }
    }
  }

  for (final item in queue) {
    final agent = crowd.agents[item.agentId]!;
    optimizePathTopology(agent.corridor, navMesh, agent.queryFilter);
    agent.topologyOptTime = 0.0;
  }
}

/**
 * Update the crowd simulation main tick loop wrapper.
 */
void update(Crowd crowd, NavMesh navMesh, double deltaTime) {
  // Check whether agent paths are still valid
  checkPathValidity(crowd, navMesh, deltaTime);

  // Optimize path topology for agents periodically
  updateTopologyOptimization(crowd, navMesh, deltaTime);

  // Handle move requests since last update
  updateMoveRequests(crowd, navMesh, deltaTime);

  // Update neighbour agents for each agent
  updateNeighbours(crowd);

  // Update local boundary for each agent
  updateLocalBoundaries(crowd, navMesh);

  // Update desired velocity based on steering to corners or velocity target
  updateCorners(crowd, navMesh);

  // Trigger off mesh connections depending on next corners
  updateOffMeshConnectionTriggers(crowd, navMesh);

  // Calculate steering
  updateSteering(crowd);

  // Obstacle avoidance with other agents and local boundary
  updateVelocityPlanning(crowd);

  // Integrate forces and velocity changes into position steps
  integrate(crowd, deltaTime);

  // Handle agent x agent collisions displacement sweeps
  handleCollisions(crowd);

  // Update corridors constraints
  updateCorridors(crowd, navMesh);

  // Off mesh connection agent animations ticking
  offMeshConnectionUpdate(crowd, deltaTime);
}

/**
 * Check if an agent is at or near the end of their corridor.
 * Works for both complete and partial paths.
 */
bool isAgentAtTarget(Crowd crowd, String agentId, double? threshold) {
  final agent = crowd.agents[agentId];
  if (agent == null) return false;

  // Must have a valid target
  if (agent.targetState != AgentTargetState.valid) return false;

  // Check if we have corners and the last corner is marked as END
  if (agent.corners.isEmpty) return false;

  final endPosition = agent.corners.last;
  final bool isEndOfPath = (endPosition.flags & StraightPathPointFlags.end.value) != 0;
  if (!isEndOfPath) return false;

  // Check distance to the end point using custom or default radius threshold
  final double arrivalThreshold = threshold ?? agent.radius;
  final double dist = agent.position.distanceTo(endPosition.position);

  return dist <= arrivalThreshold;
}
