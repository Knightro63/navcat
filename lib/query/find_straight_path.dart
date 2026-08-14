import 'package:navcat/math/vector.dart';
import 'package:three_js_math/three_js_math.dart';
import 'index.dart';
import '../geometry.dart';

enum FindStraightPathOptions {
  allCrossings(1),
  areaCrossings(2);

  final int value;
  const FindStraightPathOptions(this.value);
}

enum StraightPathPointFlags {
  start(0),
  end(1),
  offMesh(2);

  final int value;
  const StraightPathPointFlags(this.value);
}

class StraightPathPoint {
  Vector3 position;
  int type;
  int? nodeRef;
  /** @see StraightPathPointFlags */
  int flags;

  StraightPathPoint({
    required this.position,
    required this.type,
    required this.nodeRef,
    required this.flags,
  });
}

enum FindStraightPathResultFlags {
  none(0),
  success(1 << 0),
  partialPath(1 << 2),
  maxPointsReached(1 << 3),
  invalidInput(1 << 4);

  final int value;
  const FindStraightPathResultFlags(this.value);

  /// Helper to check if this specific flag is set inside a combined bitmask
  bool isSetIn(int bitmask) => (bitmask & value) != 0;
}

class FindStraightPathResult {
  int flags;
  bool success;
  List<StraightPathPoint> path;

  FindStraightPathResult({
    required this.flags,
    required this.success,
    required this.path,
  });
}

enum AppendVertexStatus {
  none(0),
  success(1 << 0),
  maxPointsReached(1 << 1),
  inProgress(1 << 2);

  final int value;
  const AppendVertexStatus(this.value);
}

AppendVertexStatus appendVertex(
  Vector3 position,
  int? nodeRef,
  int flags,
  List<StraightPathPoint> outPoints,
  int nodeType,
  int? maxPoints,
) {
  if (outPoints.isNotEmpty && outPoints[outPoints.length - 1].position.equals(position)) {
    final prevType = outPoints[outPoints.length - 1].type;

    // only update if both points are regular polygon nodes
    // off-mesh connections should always be distinct waypoints
    if (prevType == NodeType.poly.value && nodeType == NodeType.poly.value) {
      // the vertices are equal, update
      outPoints[outPoints.length - 1].nodeRef = nodeRef;
      outPoints[outPoints.length - 1].type = nodeType;

      return AppendVertexStatus.inProgress;
    }

    // for off-mesh connections or mixed types, fall through to append a new point
  }

  // append new vertex
  outPoints.add(StraightPathPoint(
    position: Vector3(position[0], position[1], position[2]),
    type: nodeType,
    nodeRef: nodeRef,
    flags: flags,
  ));

  // if there is no space to append more vertices, return
  if (maxPoints != null && outPoints.length >= maxPoints) {
    return AppendVertexStatus.values[AppendVertexStatus.success.value | AppendVertexStatus.maxPointsReached.value];
  }

  // if reached end of path, return
  if (flags & StraightPathPointFlags.end.value != 0) {
    return AppendVertexStatus.success;
  }

  // else, continue appending points
  return AppendVertexStatus.inProgress;
}

final _intersectSegSeg2DResult = createIntersectSegSeg2DResult();

final _appendPortalsPoint = Vector3.zero();
final _appendPortalsLeft = Vector3.zero();
final _appendPortalsRight = Vector3.zero();

AppendVertexStatus appendPortals(
    NavMesh navMesh,
    int startIdx,
    int endIdx,
    Vector3 endPosition,
    List<int> path,
    List<StraightPathPoint> outPoints,
    int options,
    int? maxPoints,
) {
  final left = _appendPortalsLeft;
  final right = _appendPortalsRight;

  final startPos = outPoints[outPoints.length - 1].position;

  for (int i = startIdx; i < endIdx; i++) {
    final from = path[i];
    final to = path[i + 1];

    // skip intersection if only area crossings requested and areas equal.
    if (options & FindStraightPathOptions.areaCrossings.value != 0) {
      final a = getNodeByRef(navMesh, from);
      final b = getNodeByRef(navMesh, to);

      if (a?.area == b?.area) continue;
    }

    // calculate portal
    if (!getPortalPoints(navMesh, from, to, left, right)) {
      break;
    }

    // append intersection
    final intersectResult = intersectSegSeg2D(_intersectSegSeg2DResult, startPos, endPosition, left, right);

    if (!intersectResult.hit) continue;

    final point = _appendPortalsPoint.lerpVectors( left, right, intersectResult.t);

    final toType = getNodeRefType(to);

    final stat = appendVertex(point, to, 0, outPoints, toType, maxPoints);

    if (stat != AppendVertexStatus.inProgress) {
      return stat;
    }
  }

  return AppendVertexStatus.inProgress;
}

final _findStraightPathPortalApex = Vector3.zero();
final _findStraightPathPortalLeft = Vector3.zero();
final _findStraightPathPortalRight = Vector3.zero();
final _findStraightPathLeftPortalPoint = Vector3.zero();
final _findStraightPathRightPortalPoint = Vector3.zero();
final _findStraightPath_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();

FindStraightPathResult makeFindStraightPathResult(int flags, List<StraightPathPoint> path) {
  return FindStraightPathResult(
    flags: flags,
    success: (flags & FindStraightPathResultFlags.success.value) != 0,
    path: path,
  );
}

///
/// This method peforms what is often called 'string pulling'.
///
/// The start position is clamped to the first polygon node in the path, and the
/// end position is clamped to the last. So the start and end positions should
/// normally be within or very near the first and last polygons respectively.
///
/// @param navMesh The navigation mesh to use for the search.
/// @param start The start position in world space.
/// @param end The end position in world space.
/// @param pathNodeRefs The list of polygon node references that form the path, generally obtained from `findNodePath`
/// @param maxPoints The maximum number of points to return in the straight path. If null, no limit is applied.
/// @param straightPathOptions @see FindStraightPathOptions
/// @returns The straight path
///
FindStraightPathResult findStraightPath(
    NavMesh navMesh,
    Vector3 start,
    Vector3 end,
    List<int> pathNodeRefs,[
    int? maxPoints,
    int? straightPathOptions,]
) {
    final path = <StraightPathPoint>[];

    if (!start.isFinite() || !end.isFinite() || pathNodeRefs.isEmpty) {
      return makeFindStraightPathResult(FindStraightPathResultFlags.none.value | FindStraightPathResultFlags.invalidInput.value, path);
    }

    // clamp start & end to poly boundaries
    final closestStartPos = Vector3.zero();
    if (!getClosestPointOnPolyBoundary(closestStartPos, navMesh, pathNodeRefs[0], start)) {
      return makeFindStraightPathResult(FindStraightPathResultFlags.none.value | FindStraightPathResultFlags.invalidInput.value, path);
    }

    final closestEndPos = Vector3.zero();
    if (!getClosestPointOnPolyBoundary(closestEndPos, navMesh, pathNodeRefs[pathNodeRefs.length - 1], end)) {
      return makeFindStraightPathResult(FindStraightPathResultFlags.none.value | FindStraightPathResultFlags.invalidInput.value, path);
    }

    // add start point
    final startAppendStatus = appendVertex(
        closestStartPos,
        pathNodeRefs[0],
        StraightPathPointFlags.start.value,
        path,
        getNodeRefType(pathNodeRefs[0]),
        maxPoints,
    );

    if (startAppendStatus != AppendVertexStatus.inProgress) {
        // if we hit max points on the first vertex, it's a degenerate case
        final maxPointsReached = (startAppendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
        int flags = FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value;
        if (maxPointsReached) flags |= FindStraightPathResultFlags.maxPointsReached.value;
        return makeFindStraightPathResult(flags, path);
    }

    final portalApex = _findStraightPathPortalApex;
    final portalLeft = _findStraightPathPortalLeft;
    final portalRight = _findStraightPathPortalRight;
    final left = _findStraightPathLeftPortalPoint;
    final right = _findStraightPathRightPortalPoint;

    final pathSize = pathNodeRefs.length;

    if (pathSize > 1) {
        portalApex.setFrom(closestStartPos);
        portalLeft.setFrom(portalApex);
        portalRight.setFrom(portalApex);

        int apexIndex = 0;
        int leftIndex = 0;
        int rightIndex = 0;

        int? leftNodeRef = pathNodeRefs[0];
        int? rightNodeRef = pathNodeRefs[0];
        int leftNodeType = NodeType.poly.value;
        int rightNodeType = NodeType.poly.value;

        for (int i = 0; i < pathSize; ++i) {
            int toType = NodeType.poly.value;

            if (i + 1 < pathSize) {
                final toRef = pathNodeRefs[i + 1];
                toType = getNodeRefType(toRef);

                // next portal
                if (!getPortalPoints(navMesh, pathNodeRefs[i], toRef, left, right)) {
                    // failed to get portal points, clamp end to current poly and return partial
                    final endClamp = Vector3.zero();

                    // this should only happen when the first polygon is invalid.
                    if (!getClosestPointOnPolyBoundary(endClamp, navMesh, pathNodeRefs[i], end))
                        return makeFindStraightPathResult(
                          FindStraightPathResultFlags.none.value | FindStraightPathResultFlags.invalidInput.value,
                          path,
                        );

                    // append portals along the current straight path segment.
                    if ((straightPathOptions ?? 0) & (FindStraightPathOptions.areaCrossings.value | FindStraightPathOptions.allCrossings.value) != 0) {
                        // ignore status return value as we're just about to return
                        appendPortals(navMesh, apexIndex, i, endClamp, pathNodeRefs, path, straightPathOptions!, maxPoints);
                    }

                    final nodeType = getNodeRefType(pathNodeRefs[i]);

                    // ignore status return value as we're just about to return
                    appendVertex(endClamp, pathNodeRefs[i], 0, path, nodeType, maxPoints);

                    return makeFindStraightPathResult(
                      FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value,
                      path,
                    );
                }

                if (i == 0 && toType == NodeType.poly.value) {
                    // if starting really close to the portal, advance
                    final result = distancePtSegSqr2d(_findStraightPath_distancePtSegSqr2dResult, portalApex, left, right);
                    if (result.distSqr < 1e-6) continue;
                }

                // handle off-mesh connections explicitly
                // off-mesh connections should not be subject to string-pulling optimization
                if (toType == NodeType.offMesh.value) {
                    // get the off-mesh connection data
                    final node = getNodeByRef(navMesh, toRef);
                    final offMeshConnection = navMesh.offMeshConnections[node!.offMeshConnectionId]!;
                    final offMeshConnectionAttachment = navMesh.offMeshConnectionAttachments[node.offMeshConnectionId]!;

                    // find the link from the previous poly to the off-mesh node to determine direction
                    final prevPolyRef = pathNodeRefs[i];
                    final prevNode = getNodeByRef(navMesh, prevPolyRef);
                    int? linkEdge = 0; // default to START

                    for (final linkIndex in prevNode?.links ?? []) {
                      final link = navMesh.links[linkIndex];
                      if (link?.toNodeRef == toRef) {
                        linkEdge = link?.edge;
                        break;
                      }
                    }

                    // use the link edge to determine direction
                    // edge 0 = entering from START side, edge 1 = entering from END side
                    final enteringFromStart = linkEdge == 0;

                    // determine start and end based on which side of the connection we're entering from
                    final offMeshStart = enteringFromStart ? offMeshConnection.start : offMeshConnection.end;
                    final offMeshEnd = enteringFromStart ? offMeshConnection.end : offMeshConnection.start;

                    // get the target polygon we'll land on after the off-mesh connection
                    final toPolyRef = enteringFromStart
                        ? offMeshConnectionAttachment.endPolyNode
                        : offMeshConnectionAttachment.startPolyNode;

                    // append any pending portals along the current straight path segment
                    // this ensures we add intermediate waypoints between the current apex and the off-mesh connection start
                    if (((straightPathOptions ?? 0) & (FindStraightPathOptions.areaCrossings.value | FindStraightPathOptions.allCrossings.value)) > 0) {
                        final appendPortalsStatus = appendPortals(
                            navMesh,
                            apexIndex,
                            i,
                            offMeshStart,
                            pathNodeRefs,
                            path,
                            straightPathOptions ?? 0,
                            maxPoints,
                        );
                        if (appendPortalsStatus != AppendVertexStatus.inProgress) {
                            final maxPointsReached = (appendPortalsStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                            int flags = FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value;
                            if (maxPointsReached) flags |= FindStraightPathResultFlags.maxPointsReached.value;
                            return makeFindStraightPathResult(flags, path);
                        }
                    }

                    // check if we need to do string-pulling for the last portal
                    final lastPointAdded = path[path.length - 1].position;
                    if (!lastPointAdded.equals(portalRight) && !lastPointAdded.equals(portalLeft)) {
                        final rightArea = triArea2D(lastPointAdded, portalRight, offMeshStart);
                        final leftArea = triArea2D(lastPointAdded, portalLeft, offMeshStart);
                        AppendVertexStatus? appendStatus;

                        if (rightArea <= 0 && leftArea < 0) {
                            // offMeshStart is to the right of portalRight and portalLeft => add portalLeft
                            appendStatus = appendVertex(
                                portalLeft,
                                leftNodeRef,
                                0,
                                path,
                                leftNodeRef != null ? leftNodeType : NodeType.poly.value,
                                maxPoints,
                            );
                        } else if (leftArea >= 0 && rightArea > 0) {
                            // offMeshStart is to the left of portalRight and portalLeft => add portalRight
                            appendStatus = appendVertex(
                                portalRight,
                                rightNodeRef,
                                0,
                                path,
                                rightNodeRef != null ? rightNodeType : NodeType.poly.value,
                                maxPoints,
                            );
                        }

                        if (appendStatus != null && appendStatus != AppendVertexStatus.inProgress) {
                            final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                            int resultFlags = FindStraightPathResultFlags.success.value;
                            if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;
                            return makeFindStraightPathResult(resultFlags, path);
                        }
                    }

                    // append the off-mesh connection start point (with OFFMESH flag)
                    final appendStartStatus = appendVertex(
                        offMeshStart,
                        toRef,
                        StraightPathPointFlags.offMesh.value,
                        path,
                        NodeType.offMesh.value,
                        maxPoints,
                    );

                    if (appendStartStatus != AppendVertexStatus.inProgress) {
                        final maxPointsReached = (appendStartStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                        int resultFlags = FindStraightPathResultFlags.success.value;
                        if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;
                        return makeFindStraightPathResult(resultFlags, path);
                    }

                    // append the off-mesh connection end point (landing point on target polygon)
                    final appendEndStatus = appendVertex(offMeshEnd, toPolyRef, 0, path, NodeType.poly.value, maxPoints);

                    if (appendEndStatus != AppendVertexStatus.inProgress) {
                        final maxPointsReached = (appendEndStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                        int resultFlags = FindStraightPathResultFlags.success.value;
                        if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;
                        return makeFindStraightPathResult(resultFlags, path);
                    }

                    // reset the funnel, we should start from the ground poly at the end of the off-mesh connection
                    portalApex.setFrom(offMeshEnd);
                    portalLeft.setFrom(offMeshEnd);
                    portalRight.setFrom(offMeshEnd);
                    // set apex to the landing polygon (i+1) rather than the off-mesh node (i)
                    // this prevents infinite loops: if the funnel restarts via `i = apexIndex`,
                    // we want to restart from the landing polygon, not re-enter the off-mesh handler
                    apexIndex = i + 1;
                    leftIndex = i + 1;
                    rightIndex = i + 1;
                    leftNodeRef = toPolyRef;
                    rightNodeRef = toPolyRef;
                    leftNodeType = NodeType.poly.value;
                    rightNodeType = NodeType.poly.value;

                    // skip normal funnel processing for this off-mesh connection
                    continue;
                }
            } else {
                // end of path
                left.setFrom(closestEndPos);
                right.setFrom(closestEndPos);
                toType = NodeType.poly.value;
            }

            // right vertex
            if (triArea2D(portalApex, portalRight, right) <= 0.0) {
                if (portalApex.equals(portalRight) || triArea2D(portalApex, portalLeft, right) > 0.0) {
                    portalRight.setFrom(right);
                    rightNodeRef = i + 1 < pathSize ? pathNodeRefs[i + 1] : null;
                    rightNodeType = toType;
                    rightIndex = i;
                } else {
                    // append portals along current straight segment
                    if ((straightPathOptions ?? 0) & (FindStraightPathOptions.areaCrossings.value | FindStraightPathOptions.allCrossings.value ) != 0) {
                        final appendStatus = appendPortals(
                            navMesh,
                            apexIndex,
                            leftIndex,
                            portalLeft,
                            pathNodeRefs,
                            path,
                            straightPathOptions ?? 0,
                            maxPoints,
                        );
                        if (appendStatus != AppendVertexStatus.inProgress) {
                            final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                            int flags = FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value;
                            if (maxPointsReached) flags |= FindStraightPathResultFlags.maxPointsReached.value;
                            return makeFindStraightPathResult(flags, path);
                        }
                    }

                    portalApex.setFrom(portalLeft);
                    apexIndex = leftIndex;

                    int pointFlags = 0;
                    if (leftNodeRef == null) {
                      pointFlags = StraightPathPointFlags.end.value;
                    }
                    // note: leftNodeType can never be OFFMESH here because off-mesh connections are handled explicitly above

                    // append or update vertex
                    final appendStatus = appendVertex(
                        portalApex,
                        leftNodeRef,
                        pointFlags,
                        path,
                        leftNodeRef != null ? leftNodeType : NodeType.poly.value,
                        maxPoints,
                    );

                    if (appendStatus != AppendVertexStatus.inProgress) {
                        final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;

                        int resultFlags = FindStraightPathResultFlags.success.value;
                        if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;

                        return makeFindStraightPathResult(resultFlags, path);
                    }

                    portalLeft.setFrom(portalApex);
                    portalRight.setFrom(portalApex);
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;

                    // restart
                    i = apexIndex;

                    continue;
                }
            }

            // left vertex
            if (triArea2D(portalApex, portalLeft, left) >= 0.0) {
                if (portalApex.equals(portalLeft) || triArea2D(portalApex, portalRight, left) < 0.0) {
                    portalLeft.setFrom(left);
                    leftNodeRef = i + 1 < pathSize ? pathNodeRefs[i + 1] : null;
                    leftNodeType = toType;
                    leftIndex = i;
                } else {
                    // append portals along current straight segment
                    if ((straightPathOptions ?? 0) & (FindStraightPathOptions.areaCrossings.value | FindStraightPathOptions.allCrossings.value) != 0) {
                        final appendStatus = appendPortals(
                            navMesh,
                            apexIndex,
                            rightIndex,
                            portalRight,
                            pathNodeRefs,
                            path,
                            straightPathOptions ?? 0,
                            maxPoints,
                        );

                        if (appendStatus != AppendVertexStatus.inProgress) {
                            final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;

                            int flags = FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value;
                            if (maxPointsReached) flags |= FindStraightPathResultFlags.maxPointsReached.value;

                            return makeFindStraightPathResult(flags, path);
                        }
                    }

                    portalApex.setFrom(portalRight);
                    apexIndex = rightIndex;

                    int pointFlags = 0;
                    if (rightNodeRef == null) {
                        pointFlags = StraightPathPointFlags.end.value;
                    }
                    // note: rightNodeType can never be OFFMESH here because off-mesh connections are handled explicitly above

                    // add/update vertex
                    final appendStatus = appendVertex(
                        portalApex,
                        rightNodeRef,
                        pointFlags,
                        path,
                        rightNodeRef != null ? rightNodeType : NodeType.poly.value,
                        maxPoints,
                    );

                    if (appendStatus != AppendVertexStatus.inProgress) {
                        final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;

                        int resultFlags = FindStraightPathResultFlags.success.value;
                        if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;

                        return makeFindStraightPathResult(resultFlags, path);
                    }

                    portalLeft.setFrom(portalApex);
                    portalRight.setFrom(portalApex);
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;

                    // restart
                    i = apexIndex;

                    continue;
                }
            }
        }

        // append portals along the current straight path segment
        if ((straightPathOptions ?? 0) & (FindStraightPathOptions.areaCrossings.value | FindStraightPathOptions.allCrossings.value) != 0) {
            final appendStatus = appendPortals(
                navMesh,
                apexIndex,
                pathSize - 1,
                closestEndPos,
                pathNodeRefs,
                path,
                straightPathOptions ?? 0,
                maxPoints,
            );
            if (appendStatus != AppendVertexStatus.inProgress) {
                final maxPointsReached = (appendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;
                int flags = FindStraightPathResultFlags.success.value | FindStraightPathResultFlags.partialPath.value;
                if (maxPointsReached) flags |= FindStraightPathResultFlags.maxPointsReached.value;
                return makeFindStraightPathResult(flags, path);
            }
        }
    }

    // append end point
    // attach the last poly ref if available for the end point for easier identification
    final endRef = pathNodeRefs.isNotEmpty ? pathNodeRefs[pathNodeRefs.length - 1] : null;
    final endAppendStatus = appendVertex(closestEndPos, endRef, StraightPathPointFlags.end.value, path, NodeType.poly.value, maxPoints);
    final maxPointsReached = (endAppendStatus.value & AppendVertexStatus.maxPointsReached.value) != 0;

    int resultFlags = FindStraightPathResultFlags.success.value;
    if (maxPointsReached) resultFlags |= FindStraightPathResultFlags.maxPointsReached.value;

    return makeFindStraightPathResult(resultFlags, path);
}
