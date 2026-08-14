import 'package:three_js_math/three_js_math.dart';
import 'package:navcat/navcat.dart';
import 'package:navcat/math/vector.dart';

class SegmentInterval {
  int? nodeRef;
  double tmin;
  double tmax;

  SegmentInterval({
    this.nodeRef,
    this.tmin = 0,
    this.tmax = 0,
  });
}

// helper to insert an interval into a sorted array
void insertInterval(List<SegmentInterval> intervals, double tmin, double tmax, int? nodeRef) {
  // Find insertion point
  int idx = 0;
  while (idx < intervals.length && tmax > intervals[idx].tmin) {
    idx++;
  }

  // Insert at the found position
  intervals.insert(idx, SegmentInterval(nodeRef: nodeRef, tmin: tmin, tmax: tmax));
}

class FindLocalNeighbourhoodResult {
  bool success;
  /// node references for polygons in the local neighbourhood
  List<int> nodeRefs;
  /// search nodes
  SearchNodePool searchNodes;

  FindLocalNeighbourhoodResult({
    this.success = false,
    this.nodeRefs = const [],
    required this.searchNodes,
  });
}

final _findLocalNeighbourhoodPolyVerticesA = <double>[];
final _findLocalNeighbourhoodPolyVerticesB = <double>[];
final _findLocalNeighbourhood_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();

/// Finds all polygons within a radius of a center position, avoiding overlapping polygons.
///
/// This method is optimized for a small search radius and small number of result polygons.
/// Candidate polygons are found by searching the navigation graph beginning at the start polygon.
///
/// The value of the center point is used as the start point for cost calculations.
/// It is not projected onto the surface of the mesh, so its y-value will affect the costs.
///
/// Intersection tests occur in 2D. All polygons and the search circle are projected onto
/// the xz-plane. So the y-value of the center point does not affect intersection tests.
///
/// @param navMesh The navigation mesh
/// @param startNodeRef The reference ID of the starting polygon
/// @param position The center position of the search circle
/// @param radius The search radius
/// @param filter The query filter to apply
/// @returns The result containing found polygons and their parents
///
FindLocalNeighbourhoodResult findLocalNeighbourhood(
  NavMesh navMesh,
  int startNodeRef,
  Vector3 position,
  double radius,
  QueryFilter? filter,
) {
  // search state - use SearchNodePool for this algorithm
  final nodes = <int, List<SearchNode>>{};
  final stack = <SearchNode>[];

  final result = FindLocalNeighbourhoodResult(
    success: false,
    nodeRefs: [],
    searchNodes: nodes,
  );

  // validate input
  if (!isValidNodeRef(navMesh, startNodeRef) || !position.isFinite() || radius < 0 || !radius.isFinite || filter == null) {
    return result;
  }

  // initialize start node
  final startNode = SearchNode(
    cost: 0,
    total: 0,
    parentNodeRef: null,
    parentState: null,
    nodeRef: startNodeRef,
    state: 0,
    flags: nodeFlagClosed,
    position: position.clone(),
  );
  addSearchNode(nodes, startNode);
  stack.add(startNode);

  final radiusSqr = radius * radius;

  // add start polygon to results
  result.nodeRefs.add(startNodeRef);

  // temporary arrays for polygon vertices
  final polyVerticesA = _findLocalNeighbourhoodPolyVerticesA;
  final polyVerticesB = _findLocalNeighbourhoodPolyVerticesB;

  while (stack.isNotEmpty) {
    // pop front (breadth-first search)
    final curNode = stack.removeAt(0);
    final curRef = curNode.nodeRef;

    // get current poly and tile
    final curTileAndPoly = getTileAndPolyByRef(navMesh,curRef);
    if (!curTileAndPoly.success) continue;

    // iterate through all links
    final node = getNodeByRef(navMesh, curRef);

    for (final linkIndex in node?.links ?? []) {
      final link = navMesh.links[linkIndex];
      if (link == null) continue;

      final neighbourRef = link.toNodeRef;

      // skip if already visited
      final existingNode = getSearchNode(nodes, neighbourRef, 0);
      if (existingNode != null && (existingNode.flags & nodeFlagClosed) > 0) continue;

      // get neighbour poly and tile
      final neighbourTileAndPoly = getTileAndPolyByRef(navMesh,neighbourRef);
      if (!neighbourTileAndPoly.success) continue;
      final neighbourTile = neighbourTileAndPoly.tile;
      final neighbourPoly = neighbourTileAndPoly.poly;

      // skip off-mesh connections
      if (getNodeRefType(neighbourRef) == NodeType.offMesh.value) continue;

      // apply filter
      if (!filter.passFilter(neighbourRef, navMesh)) continue;

      // find edge and calc distance to the edge
      final va = Vector3();
      final vb = Vector3();
      if (!getPortalPoints(navMesh, curRef, neighbourRef, va, vb)) continue;

      // if the circle is not touching the next polygon, skip it
      distancePtSegSqr2d(_findLocalNeighbourhood_distancePtSegSqr2dResult, position, va, vb);
      if (_findLocalNeighbourhood_distancePtSegSqr2dResult.distSqr > radiusSqr) continue;

      // mark node visited before overlap test
      final neighbourNode = SearchNode(
        cost: 0,
        total: 0,
        parentNodeRef: curRef,
        parentState: 0,
        nodeRef: neighbourRef,
        state: 0,
        flags: nodeFlagClosed,
        position: position.clone(),
      );
      addSearchNode(nodes, neighbourNode);

      // check that the polygon does not collide with existing polygons
      // collect vertices of the neighbour poly
      final npa = neighbourPoly?.vertices.length ?? 0;
      for (int k = 0; k < npa; ++k) {
        final start = neighbourPoly!.vertices[k] * 3;
        polyVerticesA[k * 3] = neighbourTile!.vertices[start];
        polyVerticesA[k * 3 + 1] = neighbourTile.vertices[start + 1];
        polyVerticesA[k * 3 + 2] = neighbourTile.vertices[start + 2];
      }

      bool overlap = false;
      for (int j = 0; j < result.nodeRefs.length; ++j) {
        final pastRef = result.nodeRefs[j];

        // connected polys do not overlap
        bool connected = false;
        final curNode = getNodeByRef(navMesh, curRef);
        for (final pastLinkIndex in curNode?.links ?? []) {
          if (navMesh.links[pastLinkIndex]?.toNodeRef == pastRef) {
            connected = true;
            break;
          }
        }
        if (connected) continue;

        // potentially overlapping - get vertices and test overlap
        final pastTileAndPoly = getTileAndPolyByRef(navMesh,pastRef);
        if (!pastTileAndPoly.success) continue;
        final pastPoly = pastTileAndPoly.poly;
        final pastTile = pastTileAndPoly.tile;

        final npb = pastPoly?.vertices.length ?? 0;
        for (int k = 0; k < npb; ++k) {
          final start = pastPoly!.vertices[k] * 3;
          polyVerticesB[k * 3] = pastTile!.vertices[start];
          polyVerticesB[k * 3 + 1] = pastTile.vertices[start + 1];
          polyVerticesB[k * 3 + 2] = pastTile.vertices[start + 2];
        }

        if (overlapPolyPoly2D(polyVerticesA, npa, polyVerticesB, npb)) {
          overlap = true;
          break;
        }
      }

      if (overlap) continue;

      // this poly is fine, store and advance to the poly
      result.nodeRefs.add(neighbourRef);

      // add to stack for further exploration
      stack.add(neighbourNode);
    }
  }

  result.success = true;
  return result;
}

class PolyWallSegmentsResult {
  bool success;
  /// segment vertices [x1,y1,z1,x2,y2,z2,x1,y1,z1,x2,y2,z2,...]
  List<double> segmentVerts;
  /// polygon references for each segment (null for wall segments)
  List<int?> segmentRefs;

  PolyWallSegmentsResult({
    this.success = false,
    this.segmentVerts = const [],
    this.segmentRefs = const [],
  });
}

/// Returns the wall segments of a polygon, optionally including portal segments.
///
/// If segmentRefs is requested, then all polygon segments will be returned.
/// Otherwise only the wall segments are returned.
///
/// A segment that is normally a portal will be included in the result set as a
/// wall if the filter results in the neighbor polygon becoming impassable.
///
/// @param navMesh The navigation mesh
/// @param nodeRef The reference ID of the polygon
/// @param filter The query filter to apply
/// @param includePortals Whether to include portal segments in the result
PolyWallSegmentsResult getPolyWallSegments(NavMesh navMesh, int nodeRef, QueryFilter? filter, bool includePortals) {
  final result = PolyWallSegmentsResult(
    success: false,
    segmentVerts: [],
    segmentRefs: [],
  );

  // validate input
  final tileAndPoly = getTileAndPolyByRef(navMesh,nodeRef);
  if (!tileAndPoly.success || filter == null) {
    return result;
  }

  final tile = tileAndPoly.tile;
  final poly = tileAndPoly.poly;
  final segmentVerts = result.segmentVerts;
  final segmentRefs = result.segmentRefs;

  // process each edge of the polygon
  for (int i = 0, j = (poly?.vertices.length ?? 0) - 1; i < (poly?.vertices.length ?? 0); j = i++) {
    final intervals = <SegmentInterval>[];

    // check if this edge has external links (tile boundary)
    if (((poly?.neis[j] ?? 0) & polyNeisFlagExtLink) > 0) {
      // tile border - find all links for this edge
      final node = getNodeByRef(navMesh, nodeRef);

      for (final linkIndex in node?.links ?? []) {
        final link = navMesh.links[linkIndex];
        if (link == null|| link.edge != j) continue;

        if (link.toNodeRef > 0) {
          final neighbourTileAndPoly = getTileAndPolyByRef(navMesh,link.toNodeRef);
          if (neighbourTileAndPoly.success) {
            if (filter.passFilter(link.toNodeRef, navMesh)) {
              insertInterval(intervals, link.bmin, link.bmax, link.toNodeRef);
            }
          }
        }
      }
    } 
    else {
      // internal edge
      int? neiRef;
      if (poly?.neis[j] != null) {
        final idx = poly!.neis[j] - 1;
        neiRef = getNodeByTileAndPoly(navMesh, tile!, idx)!.ref;

        // check if neighbor passes filter
        final neighbourTileAndPoly = getTileAndPolyByRef(navMesh,neiRef);
        if (neighbourTileAndPoly.success) {
          if (!filter.passFilter(neiRef, navMesh)) {
            neiRef = null;
          }
        }
      }

      // If the edge leads to another polygon and portals are not stored, skip.
      if (neiRef != null && !includePortals) {
        continue;
      }

      // add the full edge as a segment
      final vj = Vector3().fromArray(tile!.vertices, poly!.vertices[j] * 3);//vec3.fromBuffer(Vector3(), tile.vertices, poly.vertices[j] * 3);
      final vi = Vector3().fromArray(tile.vertices, poly.vertices[i] * 3);

      segmentVerts.addAll([vj[0], vj[1], vj[2], vi[0], vi[1], vi[2]]);
      segmentRefs.add(neiRef);
      continue;
    }

    // add sentinels for interval processing
    insertInterval(intervals, -1, 0, null);
    insertInterval(intervals, 255, 256, null);

    // store segments based on intervals
    final vj = Vector3().fromArray(tile!.vertices, poly!.vertices[j] * 3);
    final vi = Vector3().fromArray(tile.vertices, poly.vertices[i] * 3);

    for (int k = 1; k < intervals.length; ++k) {
      // portal segment
      if (includePortals && intervals[k].nodeRef != null) {
        final tmin = intervals[k].tmin / 255.0;
        final tmax = intervals[k].tmax / 255.0;

        final segStart = Vector3();
        final segEnd = Vector3();
        segStart.lerpVectors(vj, vi, tmin);
        segEnd.lerpVectors(vj, vi, tmax);

        segmentVerts.addAll([segStart[0], segStart[1], segStart[2], segEnd[0], segEnd[1], segEnd[2]]);
        segmentRefs.add(intervals[k].nodeRef);
      }

      // wall segment
      final imin = intervals[k - 1].tmax;
      final imax = intervals[k].tmin;
      if (imin != imax) {
        final tmin = imin / 255.0;
        final tmax = imax / 255.0;

        final segStart = Vector3();
        final segEnd = Vector3();
        segStart.lerpVectors(vj, vi, tmin);
        segEnd.lerpVectors( vj, vi, tmax);

        segmentVerts.addAll([segStart[0], segStart[1], segStart[2], segEnd[0], segEnd[1], segEnd[2]]);
        segmentRefs.add(null);
      }
    }
  }

  result.success = true;
  return result;
}
