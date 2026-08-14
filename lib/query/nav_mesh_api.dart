import 'package:navcat/navcat.dart';
import 'package:navcat/index_pool.dart';
import 'package:navcat/math/vector.dart';
import 'package:three_js_math/three_js_math.dart';
import 'dart:math' as math;

/// Creates a new empty navigation mesh.
/// @returns The created navigation mesh
///
NavMesh createNavMesh() {
  return NavMesh(
    origin: Vector3(),
    tileWidth: 0,
    tileHeight: 0,
    links: {},
    nodes: {},
    tiles: {},
    tilePositionToTileId: {},
    tileColumnToTileIds: {},
    offMeshConnections: {},
    offMeshConnectionAttachments: {},
    tilePositionToSequenceCounter: {},
    offMeshConnectionSequenceCounter: 0,
    nodeIndexPool: createIndexPool(),
    tileIndexPool: createIndexPool(),
    offMeshConnectionIndexPool: createIndexPool(),
    linkIndexPool: createIndexPool(),
  );
}

/// Gets a navigation mesh node by its reference.
/// Note that navmesh nodes are pooled and may be reused on removing then adding tiles, so do not store node objects.
/// @param navMesh the navigation mesh
/// @param nodeRef the node reference
/// @returns the navigation mesh node
///
NavMeshNode? getNodeByRef(NavMesh navMesh, int nodeRef) {
  final nodeIndex = getNodeRefIndex(nodeRef);
  final node = navMesh.nodes[nodeIndex];
  return node;
}

/// Gets a navigation mesh node by its tile and polygon index.
/// @param navMesh the navigation mesh
/// @param tile the navigation mesh tile
/// @param polyIndex the polygon index
/// @returns the navigation mesh node
///
NavMeshNode? getNodeByTileAndPoly(NavMesh navMesh, NavMeshTile tile, int polyIndex) {
    final navMeshNodeIndex = tile.polyNodes[polyIndex];
    final navMeshNode = navMesh.nodes[navMeshNodeIndex];

    return navMeshNode;
}

/// Checks if a navigation mesh node reference is valid.
/// @param navMesh the navigation mesh
/// @param nodeRef the node reference
/// @returns true if the node reference is valid, false otherwise
///
bool isValidNodeRef(NavMesh navMesh, int nodeRef) {
  if (nodeRef == invalidNodeRef) {
    return false;
  }

  final nodeType = getNodeRefType(nodeRef);
  if (nodeType == NodeType.poly.value) {
    final node = getNodeByRef(navMesh, nodeRef);

    if (node == null) {
      return false;
    }

    final tile = navMesh.tiles[node.tileId];

    if (tile == null) {
      return false;
    }

    final sequence = getNodeRefSequence(nodeRef);

    if (tile.sequence != sequence) {
      return false;
    }

    if (node.polyIndex < 0 || node.polyIndex >= (tile.polys?.length ?? 0)) {
      return false;
    }

    //final poly = tile.polys[node.polyIndex];
    // if (poly == null) {
    //   return false;
    // }

    return true;
  }

  if (nodeType == NodeType.offMesh.value) {
    final node = getNodeByRef(navMesh, nodeRef);

    if (node == null) {
      return false;
    }

    final offMeshConnection = navMesh.offMeshConnections[node.offMeshConnectionId];

    if (offMeshConnection == null) {
      return false;
    }

    final sequence = getNodeRefSequence(nodeRef);

    if (offMeshConnection.sequence != sequence) {
      return false;
    }

    if (!isOffMeshConnectionConnected(navMesh, offMeshConnection.id)) {
      return false;
    }

    return true;
  }
  return false;
}

/// Gets the tile at the given x, y, and layer position.
/// @param navMesh the navigation mesh
/// @param x the x position
/// @param y the y position
/// @param layer the layer
/// @returns the navigation mesh tile
///
NavMeshTile? getTileAt(NavMesh navMesh, int x, int y, int layer) {
  final tileHash = getTilePositionHash(x, y, layer);
  final tileId = navMesh.tilePositionToTileId[tileHash];
  return navMesh.tiles[tileId];
}

/// Gets all tiles at the given x and y position.
/// @param navMesh the navigation mesh
/// @param x the x position
/// @param y the y position
/// @returns the navigation mesh tiles
///
List<NavMeshTile> getTilesAt(NavMesh navMesh, int x, int y) {
  final tileColumnHash = getTileColumnHash(x, y);
  
  final tileIds = navMesh.tileColumnToTileIds[tileColumnHash];

  if (tileIds == null) return [];

  final tiles = <NavMeshTile>[];

  for (final tileId in tileIds) {
    tiles.add(navMesh.tiles[tileId]!);
  }

  return tiles;
} 

List<NavMeshTile> getNeighbourTilesAt(NavMesh navMesh, int x, int y, int side) {
  int nx = x;
  int ny = y;

  switch (side) {
    case 0:
      nx++;
      break;
    case 1:
      nx++;
      ny++;
      break;
    case 2:
      ny++;
      break;
    case 3:
      nx--;
      ny++;
      break;
    case 4:
      nx--;
      break;
    case 5:
      nx--;
      ny--;
      break;
    case 6:
      ny--;
      break;
    case 7:
      nx++;
      ny--;
      break;
  }

  return getTilesAt(navMesh, nx, ny);
}

String getTilePositionHash(int x, int y, int layer) {
  return '${x},${y},${layer}';
}

String getTileColumnHash(int x, int y) {
  return '${x},${y}';
}

/**
 * Returns the tile x and y position in the nav mesh from a world space position.
 * @param outTilePosition the output tile position
 * @param navMesh the navigation mesh
 * @param worldPosition the world space position
 */
Vector2 worldToTilePosition(Vector2 outTilePosition, NavMesh navMesh, Vector3 worldPosition) {
  outTilePosition[0] = (worldPosition[0] - navMesh.origin[0]) / navMesh.tileWidth;
  outTilePosition[1] = (worldPosition[2] - navMesh.origin[2]) / navMesh.tileHeight;
  return outTilePosition;
}

class GetTileAndPolyByRefResult {
  bool success;
  NavMeshTile? tile;
  NavMeshPoly? poly;
  int polyIndex;

  GetTileAndPolyByRefResult({
    required this.success,
    required this.tile,
    required this.poly,
    required this.polyIndex,
  });
}

/// Gets the tile and polygon from a polygon reference
/// @param navMesh The navigation mesh
/// @param ref The polygon reference
/// @returns Object containing tile and poly, or null if not found
///
GetTileAndPolyByRefResult getTileAndPolyByRef(NavMesh navMesh, int ref) {
  final result = GetTileAndPolyByRefResult(
    success: false,
    tile: null,
    poly: null,
    polyIndex: -1,
  );

  final nodeType = getNodeRefType(ref);

  if (nodeType != NodeType.poly.value) return result;

  final node = getNodeByRef(navMesh, ref);
  final tileId = node?.tileId;
  final polyIndex = node?.polyIndex;

  final tile = navMesh.tiles[tileId];

  if (tile == null) {
    return result;
  }

  if (polyIndex! >= (tile.polys?.length ?? 0)) {
    return result;
  }

  result.poly = tile.polys?[polyIndex];
  result.tile = tile;
  result.polyIndex = polyIndex;
  result.success = true;

  return result;
}

final _getPolyHeightA = Vector3();
final _getPolyHeightB = Vector3();
final _getPolyHeightC = Vector3();
final _getPolyHeightTriangle = [_getPolyHeightA, _getPolyHeightB, _getPolyHeightC];
final Map<int,double> _getPolyHeightVertices = {};

class GetPolyHeightResult {
  bool success;
  double height;

  GetPolyHeightResult({
    required this.success,
    required this.height,
  });
}

GetPolyHeightResult createGetPolyHeightResult() => GetPolyHeightResult(
  success: false,
  height: 0,
);

final _getDetailMeshHeightClosest = Vector3();

double getDetailMeshHeight(NavMeshTile tile, NavMeshPoly? poly, int polyIndex, Vector3 pos) {
  final detailMesh = tile.detailMeshes[polyIndex];

  // point is inside polygon, find height at the location
  if (detailMesh != null) {
    for (int j = 0; j < detailMesh.trianglesCount; ++j) {
      final t = (detailMesh.trianglesBase + j) * 4;
      final detailTriangles = tile.detailTriangles;

      // get triangle vertices
      final List<Vector3> v = _getPolyHeightTriangle;
      for (int k = 0; k < 3; ++k) {
        final vertIndex = detailTriangles[t + k];
        if (vertIndex < poly!.vertices.length) {
          // use polygon vertex
          final polyVertIndex = poly.vertices[vertIndex] * 3;
          v[k][0] = tile.vertices[polyVertIndex + 0];
          v[k][1] = tile.vertices[polyVertIndex + 1];
          v[k][2] = tile.vertices[polyVertIndex + 2];
        } else {
          // use detail vertices
          final detailVertIndex = (detailMesh.verticesBase + (vertIndex - poly.vertices.length)) * 3;
          v[k][0] = tile.detailVertices[detailVertIndex + 0];
          v[k][1] = tile.detailVertices[detailVertIndex + 1];
          v[k][2] = tile.detailVertices[detailVertIndex + 2];
        }
      }

      final height = closestHeightPointTriangle(pos, v[0], v[1], v[2]);

      if (!height.isNaN) {
        return height;
      }
    }
  }

  getClosestPointOnDetailEdges(_getDetailMeshHeightClosest, tile, poly, polyIndex, pos, false);
  return _getDetailMeshHeightClosest[1];
}

/// Gets the height of a polygon at a given point using detail mesh if available.
/// @param result The result object to populate
/// @param tile The tile containing the polygon
/// @param poly The polygon
/// @param polyIndex The index of the polygon in the tile
/// @param pos The position to get height for
/// @returns The result object with success flag and height
///
GetPolyHeightResult getPolyHeight(GetPolyHeightResult result, NavMeshTile tile, NavMeshPoly? poly, int polyIndex, Vector3 pos) {
  result.success = false;
  result.height = 0;

  // build polygon vertices array
  final nv = poly?.vertices.length ?? 0;
  final vertices = _getPolyHeightVertices;
  for (int i = 0; i < nv; ++i) {
    final start = poly!.vertices[i] * 3;
    vertices[i * 3] = tile.vertices[start];
    vertices[i * 3 + 1] = tile.vertices[start + 1];
    vertices[i * 3 + 2] = tile.vertices[start + 2];
  }
  final d = vertices.values.toList();
  // check if point is inside polygon
  if (!pointInPoly(pos, d, nv)) {
    return result;
  }

  result.height = getDetailMeshHeight(tile, poly, polyIndex, pos);
  result.success = true;

  return result;
}

/// Get flags for edge in detail triangle.
/// @param[in]	triFlags		The flags for the triangle (last component of detail vertices above).
/// @param[in]	edgeIndex		The index of the first vertex of the edge. For instance, if 0,
///								returns flags for edge AB.
/// @returns The edge flags
///
int getDetailTriEdgeFlags(int triFlags, int edgeIndex) {
  return (triFlags >> (edgeIndex * 2)) & 0x3;
}

final _closestPointOnDetailEdgesTriangleVertices = [Vector3(), Vector3(), Vector3()];
final _closestPointOnDetailEdgesPmin = Vector3();
final _closestPointOnDetailEdgesPmax = Vector3();
final _closestPointOnDetailEdgesDistancePtSegSqr2dResult = createDistancePtSegSqr2dResult();

/// Gets the closest point on detail mesh edges to a given point
/// @param tile The tile containing the detail mesh
/// @param poly The polygon
/// @param pos The position to find closest point for
/// @param outClosestPoint Output parameter for the closest point
/// @param onlyBoundary If true, only consider boundary edges
/// @returns The squared distance to the closest point
///  closest point
///
double getClosestPointOnDetailEdges(Vector3 outClosestPoint, NavMeshTile? tile, NavMeshPoly? poly, int polyIndex, Vector3 pos, bool onlyBoundary) {
  final detailMesh = tile?.detailMeshes[polyIndex];

  double dmin = double.maxFinite;
  double tmin = 0;

  final pmin = _closestPointOnDetailEdgesPmin.setValues( 0, 0, 0);
  final pmax = _closestPointOnDetailEdgesPmax.setValues( 0, 0, 0);

  for (int i = 0; i < (detailMesh?.trianglesCount ?? 0); i++) {
    final t = (detailMesh!.trianglesBase + i) * 4;
    final detailTriangles = tile!.detailTriangles;

    // check if triangle has boundary edges (if onlyBoundary is true)
    if (onlyBoundary) {
      final triFlags = detailTriangles[t + 3];
      final ANY_BOUNDARY_EDGE = (detailEdgeBoundary << 0) | (detailEdgeBoundary << 2) | (detailEdgeBoundary << 4);
      if ((triFlags & ANY_BOUNDARY_EDGE) == 0) {
        continue;
      }
    }

    // get triangle vertices
    final List<Vector3> triangleVertices = _closestPointOnDetailEdgesTriangleVertices;
    for (int j = 0; j < 3; j++) {
      final vertexIndex = detailTriangles[t + j];
      if (vertexIndex < poly!.vertices.length) {
        // use polygon vertex
        triangleVertices[j].fromArray(tile.vertices, poly.vertices[vertexIndex] * 3);
      } else {
        final detailIndex = (detailMesh.verticesBase + (vertexIndex - poly.vertices.length)) * 3;
        triangleVertices[j].fromArray(tile.detailVertices, detailIndex);
      }
    }

    // check each edge of the triangle
    for (int k = 0, j = 2; k < 3; j = k++) {
      final triFlags = detailTriangles[t + 3];
      final edgeFlags = getDetailTriEdgeFlags(triFlags, j);

      // skip internal edges if we want only boundaries, or skip duplicate internal edges
      if ((edgeFlags & detailEdgeBoundary) == 0 && (onlyBoundary || detailTriangles[t + j] < detailTriangles[t + k])) {
        // only looking at boundary edges and this is internal, or
        // this is an inner edge that we will see again or have already seen.
        continue;
      }

      final result = distancePtSegSqr2d(
        _closestPointOnDetailEdgesDistancePtSegSqr2dResult,
        pos,
        triangleVertices[j],
        triangleVertices[k],
      );

      if (result.distSqr < dmin) {
        dmin = result.distSqr;
        tmin = result.t;
        pmin.setFrom(triangleVertices[j]);
        pmax.setFrom(triangleVertices[k]);
      }
    }
  }

  // interpolate the final closest point
  //if (pmin && pmax) {
    outClosestPoint.lerpVectors(pmin, pmax, tmin);
  //}

  return dmin;
}

class GetClosestPointOnPolyResult {
  bool success;
  bool isOverPoly;
  late Vector3 position;

  GetClosestPointOnPolyResult({
    this.success = false, 
    this.isOverPoly = false, 
    Vector3? position
  }){
    this.position = position?? Vector3();
  }
}

GetClosestPointOnPolyResult createGetClosestPointOnPolyResult() {
  return GetClosestPointOnPolyResult();
}

final _getClosestPointOnPolyHeightResult = createGetPolyHeightResult();

/// Gets the closest point on a polygon to a given point
/// @param result the result object to populate
/// @param navMesh the navigation mesh
/// @param nodeRef the polygon node reference
/// @param position the point to find the closest point to
/// @returns the result object
GetClosestPointOnPolyResult getClosestPointOnPoly(
  GetClosestPointOnPolyResult result,
  NavMesh navMesh,
  int nodeRef,
  Vector3 position,
) {
  result.success = false;
  result.isOverPoly = false;
  result.position.setFrom(position);

  GetTileAndPolyByRefResult tileAndPoly = getTileAndPolyByRef(navMesh,nodeRef);
  
  if (!tileAndPoly.success) {
    return result;
  }
  result.success = true;

  final tile = tileAndPoly.tile;
  final poly = tileAndPoly.poly;
  final polyIndex = tileAndPoly.polyIndex;
  final polyHeight = getPolyHeight(_getClosestPointOnPolyHeightResult, tile!, poly, polyIndex, position);

  if (polyHeight.success) {
    result.position.setFrom( position);
    result.position[1] = polyHeight.height;
    result.isOverPoly = true;
    return result;
  }

  getClosestPointOnDetailEdges(result.position, tile, poly, polyIndex, position, true);

  return result;
}

final _closestPointOnPolyBoundaryLineStart = Vector3();
final _closestPointOnPolyBoundaryLineEnd = Vector3();
final Map<int,double> _closestPointOnPolyBoundaryVertices = {};
final _closestPointOnPolyBoundary_distancePtSegSqr2dResult = createDistancePtSegSqr2dResult();

/// Gets the closest point on the boundary of a polygon to a given point
/// @param out the output closest point
/// @param navMesh the navigation mesh
/// @param nodeRef the polygon node reference
/// @param point the point to find the closest point to
/// @returns whether the operation was successful
bool getClosestPointOnPolyBoundary(
  Vector3? out,
  NavMesh navMesh,
  int nodeRef,
  Vector3 point,
) {
  final tileAndPoly = getTileAndPolyByRef(navMesh,nodeRef);

  if (!tileAndPoly.success || !point.isFinite() || out == null) {
    return false;
  }

  final tile = tileAndPoly.tile;
  final poly = tileAndPoly.poly;
  final polyIndex = tileAndPoly.polyIndex;

  final lineStart = _closestPointOnPolyBoundaryLineStart;
  final lineEnd = _closestPointOnPolyBoundaryLineEnd;

  // collect vertices
  final verticesCount = poly?.vertices.length ?? 0;
  for (int i = 0; i < verticesCount; ++i) {
    final vIndex = poly!.vertices[i] * 3;
    _closestPointOnPolyBoundaryVertices[i * 3] = tile!.vertices[vIndex];
    _closestPointOnPolyBoundaryVertices[i * 3 + 1] = tile.vertices[vIndex + 1];
    _closestPointOnPolyBoundaryVertices[i * 3 + 2] = tile.vertices[vIndex + 2];
  }
  final vertices = _closestPointOnPolyBoundaryVertices.values.toList();
  // if inside polygon, return the point as-is
  if (pointInPoly(point, vertices, verticesCount)) {
      out[0] = point[0];
      out[2] = point[2];
      out[1] = getDetailMeshHeight(tile!, poly, polyIndex, point);

      return true;
  }

  // otherwise clamp to nearest edge
  double dmin = double.maxFinite;
  int imin = 0;
  for (int i = 0; i < verticesCount; ++i) {
    final j = (i + 1) % verticesCount;
    final vaIndex = i * 3;
    final vbIndex = j * 3;
    lineStart[0] = vertices[vaIndex + 0];
    lineStart[1] = vertices[vaIndex + 1];
    lineStart[2] = vertices[vaIndex + 2];
    lineEnd[0] = vertices[vbIndex + 0];
    lineEnd[1] = vertices[vbIndex + 1];
    lineEnd[2] = vertices[vbIndex + 2];
    distancePtSegSqr2d(_closestPointOnPolyBoundary_distancePtSegSqr2dResult, point, lineStart, lineEnd);
    if (_closestPointOnPolyBoundary_distancePtSegSqr2dResult.distSqr < dmin) {
      dmin = _closestPointOnPolyBoundary_distancePtSegSqr2dResult.distSqr;
      imin = i;
    }
  }

    final j = (imin + 1) % verticesCount;
    final vaIndex = imin * 3;
    final vbIndex = j * 3;
    final va0 = vertices[vaIndex + 0];
    final va1 = vertices[vaIndex + 1];
    final va2 = vertices[vaIndex + 2];
    final vb0 = vertices[vbIndex + 0];
    final vb1 = vertices[vbIndex + 1];
    final vb2 = vertices[vbIndex + 2];

    // compute t on segment (xz plane)
    final pqx = vb0 - va0;
    final pqz = vb2 - va2;
    final dx = point[0] - va0;
    final dz = point[2] - va2;
    final denom = pqx * pqx + pqz * pqz;
    double t = denom > 0 ? (pqx * dx + pqz * dz) / denom : 0;
    if (t < 0) t = 0;
    else if (t > 1) t = 1;

    out[0] = va0 + (vb0 - va0) * t;
    out[1] = va1 + (vb1 - va1) * t;
    out[2] = va2 + (vb2 - va2) * t;

    return true;
}

class FindNearestPolyResult {
  bool success;
  int nodeRef;
  late final Vector3 position;

  FindNearestPolyResult({this.success = false, this.nodeRef = 0, Vector3? position}){
    this.position = position ?? Vector3();
  }
}

FindNearestPolyResult createFindNearestPolyResult() {
  return FindNearestPolyResult();
}

final _findNearestPolyClosestPointResult = createGetClosestPointOnPolyResult();
final _findNearestPolyDiff = Vector3();
final BoundingBox _findNearestPolyBounds = BoundingBox();

FindNearestPolyResult findNearestPoly(
  FindNearestPolyResult result,
  NavMesh navMesh,
  Vector3 center,
  Vector3 halfExtents,
  QueryFilter queryFilter,
){
  result.success = false;
  result.nodeRef = 0;
  result.position.setFrom( center);

  // get bounds for the query
  final BoundingBox bounds = _findNearestPolyBounds;
  bounds.min.sub2( center, halfExtents);
  bounds.max.add2( center, halfExtents);

  // query polygons within the query bounds
  final polys = queryPolygons(navMesh, bounds, queryFilter);
  double nearestDistSqr = double.infinity;

  // find the closest polygon
  for (final ref in polys) {
    final closestPoint = getClosestPointOnPoly(_findNearestPolyClosestPointResult, navMesh, ref, center);

    if (!closestPoint.success) continue;
    
    final node = getNodeByRef(navMesh, ref);
    final tileId = node?.tileId;

    final tile = navMesh.tiles[tileId];

    if (tile == null) continue;

    // calculate difference vector
    _findNearestPolyDiff.sub2( center, closestPoint.position);

    double distSqr;

    // if a point is directly over a polygon and closer than
    // climb height, favor that instead of straight line nearest point.
    if (closestPoint.isOverPoly) {
      final heightDiff = _findNearestPolyDiff[1].abs() - tile.walkableClimb;
      distSqr = heightDiff > 0 ? heightDiff * heightDiff : 0;
    } else {
      distSqr = _findNearestPolyDiff.length2;
    }

    if (distSqr < nearestDistSqr) {
      nearestDistSqr = distSqr;
      result.nodeRef = ref;
      result.position.setFrom( closestPoint.position);
      result.success = true;
    }
  }

  return result;
}

final _queryPolygonsInTileBmax = Vector3();
final _queryPolygonsInTileBmin = Vector3();

void queryPolygonsInTile(
  List<int> out,
  NavMesh navMesh,
  NavMeshTile tile,
  BoundingBox bounds,
  QueryFilter filter,
) {
    final qmin = bounds.min;
    final qmax = bounds.max;

    int nodeIndex = 0;
    final endIndex = tile.bvTree.nodes.length;
    final tbmin = tile.bounds.min;
    final tbmax = tile.bounds.max;
    final qfac = tile.bvTree.quantFactor;

    // clamp query box to world box.
    final minx = math.max(math.min(qmin[0], tbmax[0]), tbmin[0]) - tbmin[0];
    final miny = math.max(math.min(qmin[1], tbmax[1]), tbmin[1]) - tbmin[1];
    final minz = math.max(math.min(qmin[2], tbmax[2]), tbmin[2]) - tbmin[2];
    final maxx = math.max(math.min(qmax[0], tbmax[0]), tbmin[0]) - tbmin[0];
    final maxy = math.max(math.min(qmax[1], tbmax[1]), tbmin[1]) - tbmin[1];
    final maxz = math.max(math.min(qmax[2], tbmax[2]), tbmin[2]) - tbmin[2];

    // quantize
    _queryPolygonsInTileBmin[0] = ((qfac * minx).floor() & 0xfffe) * 1.0;
    _queryPolygonsInTileBmin[1] = ((qfac * miny).floor() & 0xfffe) * 1.0;
    _queryPolygonsInTileBmin[2] = ((qfac * minz).floor() & 0xfffe) * 1.0;
    _queryPolygonsInTileBmax[0] = ((qfac * maxx + 1).floor() | 1) * 1.0;
    _queryPolygonsInTileBmax[1] = ((qfac * maxy + 1).floor() | 1) * 1.0;
    _queryPolygonsInTileBmax[2] = ((qfac * maxz + 1).floor() | 1) * 1.0;

    // traverse tree
    while (nodeIndex < endIndex) {
      final bvNode = tile.bvTree.nodes[nodeIndex];

      final nodeBounds = bvNode.bounds;
      final overlap =
          _queryPolygonsInTileBmin[0] <= nodeBounds.max[0] &&
          _queryPolygonsInTileBmax[0] >= nodeBounds.min[0] &&
          _queryPolygonsInTileBmin[1] <= nodeBounds.max[1] &&
          _queryPolygonsInTileBmax[1] >= nodeBounds.min[1] &&
          _queryPolygonsInTileBmin[2] <= nodeBounds.max[2] &&
          _queryPolygonsInTileBmax[2] >= nodeBounds.min[2];

      final isLeafNode = bvNode.i >= 0;

      if (isLeafNode && overlap) {
        final polyIndex = bvNode.i;
        final node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

        if (filter.passFilter(node!.ref, navMesh)) {
          out.add(node.ref);
        }
      }

      if (overlap || isLeafNode) {
        nodeIndex++;
      } else {
        final escapeIndex = -bvNode.i;
        nodeIndex += escapeIndex;
      }
    }
}

final _queryPolygonsMinTile = Vector2();
final _queryPolygonsMaxTile = Vector2();

List<int> queryPolygons(NavMesh navMesh, BoundingBox bounds, QueryFilter filter) {
    final result = <int>[];

    // find min and max tile positions
    final minTile = worldToTilePosition(_queryPolygonsMinTile, navMesh, bounds.min);
    final maxTile = worldToTilePosition(_queryPolygonsMaxTile, navMesh, bounds.max);
    // iterate through the tiles in the query bounds
    if (!minTile.isFinite() || !maxTile.isFinite()) {
      return result;
    }
    for (int x = minTile[0].toInt()-1; x < maxTile[0]; x++) {
      for (int y = minTile[1].toInt()-1; y < maxTile[1]; y++) {
        final tiles = getTilesAt(navMesh, x+1, y+1);
        for (final tile in tiles) {
          queryPolygonsInTile(result, navMesh, tile, bounds, filter);
        }
      }
    }

    return result;
}

NavMeshNode allocateNode(NavMesh navMesh) {
  final nodeIndex = IndexPool.requestIndex(navMesh.nodeIndexPool).toInt();

  NavMeshNode? node = navMesh.nodes[nodeIndex];

  if (node == null) {
      node = navMesh.nodes[nodeIndex] = NavMeshNode(
        allocated: true,
        index: nodeIndex,
        ref: 0,
        area: 0,
        flags: 0,
        links: [],
        type: 0,
        tileId: -1,
        polyIndex: -1,
        offMeshConnectionId: -1,
    );
  }

  node.allocated = true;

  return node;
}

void releaseNode(NavMesh navMesh, int index) {
    final node = navMesh.nodes[index];

    node?.allocated = false;
    node?.links.length = 0;
    node?.ref = 0;
    node?.type = 0;
    node?.area = -1;
    node?.flags = -1;
    node?.tileId = -1;
    node?.polyIndex = -1;
    node?.offMeshConnectionId = -1;

    IndexPool.releaseIndex(navMesh.nodeIndexPool, index);
}

/**
 * Allocates a link and returns it's index
 */
int allocateLink(NavMesh navMesh) {
  final linkIndex = IndexPool.requestIndex(navMesh.linkIndexPool).toInt();

  NavMeshLink? link = navMesh.links[linkIndex];

  if (link == null) {
    link = navMesh.links[linkIndex] = NavMeshLink(
      allocated: true,
      index: linkIndex,
      fromNodeIndex: -1,
      fromNodeRef: invalidNodeRef,
      toNodeIndex: -1,
      toNodeRef: invalidNodeRef,
      edge: 0,
      side: 0,
      bmin: 0,
      bmax: 0,
    );
  }

  link.allocated = true;

  return linkIndex;
}

/**
 * Releases a link
 */
void releaseLink(NavMesh navMesh, int index) {
  final link = navMesh.links[index];

  link?.allocated = false;
  link?.fromNodeIndex = -1;
  link?.fromNodeRef = invalidNodeRef;
  link?.toNodeIndex = -1;
  link?.toNodeRef = invalidNodeRef;
  link?.edge = 0;
  link?.side = 0;
  link?.bmin = 0;
  link?.bmax = 0;

  IndexPool.releaseIndex(navMesh.linkIndexPool, index);
}

void connectInternalLinks(NavMesh navMesh, NavMeshTile tile) {
  // create links between polygons within the tile
  // based on the neighbor information stored in each polygon

  for (int polyIndex = 0; polyIndex < (tile.polys?.length ?? 0); polyIndex++) {
    final poly = tile.polys![polyIndex];
    final node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

    for (int edgeIndex = 0; edgeIndex < poly.vertices.length; edgeIndex++) {
      final neiValue = poly.neis[edgeIndex];

      // skip external links and border edges
      if (neiValue == 0 || (neiValue & polyNeisFlagExtLink) != 0) {
        continue;
      }

      // internal connection - create link
      final neighborPolyIndex = neiValue - 1; // convert back to 0-based indexing

      if (neighborPolyIndex >= 0 && neighborPolyIndex < (tile.polys?.length ?? 0)) {
        final linkIndex = allocateLink(navMesh);
        final link = navMesh.links[linkIndex];

        final neighbourNode = getNodeByTileAndPoly(navMesh, tile, neighborPolyIndex);

        link?.fromNodeIndex = node!.index;
        link?.fromNodeRef = node!.ref;
        link?.toNodeIndex = neighbourNode!.index;
        link?.toNodeRef = neighbourNode!.ref;
        link?.edge = edgeIndex; // edge index in current polygon
        link?.side = 0xff; // not a boundary link
        link?.bmin = 0; // not used for internal links
        link?.bmax = 0; // not used for internal links

        node?.links.add(linkIndex);
      }
    }
  }
}

int oppositeTile(int side) {
  return (side + 4) & 0x7;
}

// Compute a scalar coordinate along the primary axis for the slab
num getSlabCoord(Vector3 v, int side) {
  if (side == 0 || side == 4) return v[0]; // x portals measure by x
  if (side == 2 || side == 6) return v[2]; // z portals measure by z
  return 0;
}

// Calculate 2D endpoints (u,y) for edge segment projected onto the portal axis plane.
// For x-portals (side 0/4) we use u = z, for z-portals (2/6) u = x.
void calcSlabEndPoints(Vector3 va, Vector3 vb, Vector3 bmin, Vector3 bmax, int side) {
  if (side == 0 || side == 4) {
    if (va[2] < vb[2]) {
      bmin[0] = va[2];
      bmin[1] = va[1];
      bmax[0] = vb[2];
      bmax[1] = vb[1];
    } else {
      bmin[0] = vb[2];
      bmin[1] = vb[1];
      bmax[0] = va[2];
      bmax[1] = va[1];
    }
  } else if (side == 2 || side == 6) {
    if (va[0] < vb[0]) {
      bmin[0] = va[0];
      bmin[1] = va[1];
      bmax[0] = vb[0];
      bmax[1] = vb[1];
    } else {
      bmin[0] = vb[0];
      bmin[1] = vb[1];
      bmax[0] = va[0];
      bmax[1] = va[1];
    }
  }
}

// Overlap test of two edge slabs in (u,y) space, with tolerances px (horizontal pad) and py (vertical threshold)
bool overlapSlabs(Vector3 amin, Vector3 amax, Vector3 bmin, Vector3 bmax, num px, num py) {
  final minx = math.max(amin[0] + px, bmin[0] + px);
  final maxx = math.min(amax[0] - px, bmax[0] - px);
  if (minx > maxx) return false; // no horizontal overlap

  // Vertical overlap test via line interpolation along u
  final ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
  final ak = amin[1] - ad * amin[0];
  final bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
  final bk = bmin[1] - bd * bmin[0];
  final aminy = ad * minx + ak;
  final amaxy = ad * maxx + ak;
  final bminy = bd * minx + bk;
  final bmaxy = bd * maxx + bk;
  final dmin = bminy - aminy;
  final dmax = bmaxy - amaxy;
  if (dmin * dmax < 0) return true; // crossing
  final thr = py * 2 * (py * 2);
  if (dmin * dmin <= thr || dmax * dmax <= thr) return true; // near endpoints
  return false;
}

final _amin = Vector3();
final _amax = Vector3();
final _bmin = Vector3();
final _bmax = Vector3();

class ConnectingPoly {
  int ref;
  double tmin;
  double tmax;

  ConnectingPoly({required this.ref, required this.tmin, required this.tmax});
}

/**
 * Find connecting external polys between edge va->vb in target tile on opposite side.
 * Returns array of { ref, tmin, tmax } describing overlapping intervals along the edge.
 * @param va vertex A
 * @param vb vertex B
 * @param target target tile
 * @param side portal side
 * @returns array of connecting polygons
 */
List<ConnectingPoly> findConnectingPolys(
  NavMesh navMesh,
  Vector3 va,
  Vector3 vb,
  NavMeshTile? target,
  int side,
) {
    if (target == null) return [];
    calcSlabEndPoints(va, vb, _amin, _amax, side); // store u,y
    final apos = getSlabCoord(va, side);

    final results = <ConnectingPoly>[];

    // iterate target polys & their boundary edges (those marked ext link in that direction)
    for (int i = 0; i < (target.polys?.length ?? 0); i++) {
        final poly = target.polys![i];
        final nv = poly.vertices.length;
        for (int j = 0; j < nv; j++) {
            final nei = poly.neis[j];

            // not an external edge
            if ((nei & polyNeisFlagExtLink) == 0) continue;

            final dir = nei & polyNeisFlagExtLinkDirMask;

            // only edges that face the specified side from target perspective
            if (dir != side) continue;

            final vcIndex = poly.vertices[j]; 
            final vdIndex = poly.vertices[(j + 1) % nv];
            final vc = Vector3(
                target.vertices[vcIndex * 3],
                target.vertices[vcIndex * 3 + 1],
                target.vertices[vcIndex * 3 + 2],
            );
            final vd = Vector3(
                target.vertices[vdIndex * 3],
                target.vertices[vdIndex * 3 + 1],
                target.vertices[vdIndex * 3 + 2],
            );

            final bpos = getSlabCoord(vc, side);

            // not co-planar enough
            if ((apos - bpos).abs() > 0.01) continue;

            calcSlabEndPoints(vc, vd, _bmin, _bmax, side);
            if (!overlapSlabs(_amin, _amax, _bmin, _bmax, 0.01, target.walkableClimb)) continue;

            // record overlap interval
            final polyRef = getNodeByTileAndPoly(navMesh, target, i)!.ref;

            results.add(ConnectingPoly(
                ref: polyRef,
                tmin: math.max(_amin[0], _bmin[0]),
                tmax: math.min(_amax[0], _bmax[0]),
            ));

            // proceed to next polygon (edge matched)
            break;
        }
    }
    return results;
}

final _va = Vector3();
final _vb = Vector3();

void connectExternalLinks(NavMesh navMesh, NavMeshTile tile, NavMeshTile target, int side) {
    // connect border links
    for (int polyIndex = 0; polyIndex < (tile.polys?.length ?? 0); polyIndex++) {
        final poly = tile.polys![polyIndex];

        // get the node for this poly
        final node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

        final nv = poly.vertices.length;

        for (int j = 0; j < nv; j++) {
            // skip non-portal edges
            if ((poly.neis[j] & polyNeisFlagExtLink) == 0) {
                continue;
            }

            final dir = poly.neis[j] & polyNeisFlagExtLinkDirMask;
            if (side != -1 && dir != side) {
                continue;
            }

            // create new links
            final va = _va.fromArray(tile.vertices, poly.vertices[j] * 3);
            final vb = _vb.fromArray(tile.vertices, poly.vertices[(j + 1) % nv] * 3);

            // find overlaps against target tile along the opposite side direction
            final overlaps = findConnectingPolys(navMesh, va, vb, target, oppositeTile(dir));

            for (final o in overlaps) {
                // parameterize overlap interval along this edge to [0,1]
                num tmin;
                num tmax;

                if (dir == 0 || dir == 4) {
                    // x portals param by z
                    tmin = (o.tmin - va[2]) / (vb[2] - va[2]);
                    tmax = (o.tmax - va[2]) / (vb[2] - va[2]);
                } else {
                    // z portals param by x
                    tmin = (o.tmin - va[0]) / (vb[0] - va[0]);
                    tmax = (o.tmax - va[0]) / (vb[0] - va[0]);
                }

                if (tmin > tmax) {
                    final tmp = tmin;
                    tmin = tmax;
                    tmax = tmp;
                }

                tmin = math.max(0, math.min(1, tmin));
                tmax = math.max(0, math.min(1, tmax));

                final linkIndex = allocateLink(navMesh);
                final link = navMesh.links[linkIndex];

                link?.fromNodeIndex = node!.index;
                link?.fromNodeRef = node!.ref;
                link?.toNodeIndex = getNodeRefIndex(o.ref);
                link?.toNodeRef = o.ref;
                link?.edge = j;
                link?.side = dir;
                link?.bmin = (tmin * 255).round().toDouble();
                link?.bmax = (tmax * 255).round().toDouble();

                node?.links.add(linkIndex);
            }
        }
    }
}

/**
 * Disconnect external links from tile to target tile
 */
void disconnectExternalLinks(NavMesh navMesh, NavMeshTile tile, NavMeshTile target) {
    final targetId = target.id;

    for (int polyIndex = 0; polyIndex < (tile.polys?.length ?? 0); polyIndex++) {
        final node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

        final filteredLinks = <int>[];

        for (int k = 0; k < (node?.links.length ?? 0); k++) {
            final linkIndex = node!.links[k];
            final link = navMesh.links[linkIndex];

            final neiNode = getNodeByRef(navMesh, link!.toNodeRef);

            if (neiNode!.tileId == targetId) {
                releaseLink(navMesh, linkIndex);
            } else {
                filteredLinks.add(linkIndex);
            }
        }

        node!.links = filteredLinks;
    }
}

void createOffMeshLink(NavMesh navMesh, NavMeshNode from, NavMeshNode to, int edge) {
  final linkIndex = allocateLink(navMesh);

  final link = navMesh.links[linkIndex];
  link?.fromNodeIndex = from.index;
  link?.fromNodeRef = from.ref;
  link?.toNodeIndex = to.index;
  link?.toNodeRef = to.ref;
  link?.edge = edge;
  link?.side = 0; // not used for offmesh links
  link?.bmin = 0; // not used for offmesh links
  link?.bmax = 0; // not used for offmesh links

  from.links.add(linkIndex);
}

final _connectOffMeshConnection_nearestPolyStart = createFindNearestPolyResult();
final _connectOffMeshConnection_nearestPolyEnd = createFindNearestPolyResult();
final _connectOffMeshConnection_halfExtents = Vector3();

bool connectOffMeshConnection(NavMesh navMesh, OffMeshConnection offMeshConnection) {
    // find polys for the start and end positions
    final radiusHalfExtents = _connectOffMeshConnection_halfExtents.setValues(
      offMeshConnection.radius,
      offMeshConnection.radius,
      offMeshConnection.radius,
    );

    final startTilePolyResult = findNearestPoly(
        _connectOffMeshConnection_nearestPolyStart,
        navMesh,
        offMeshConnection.start,
        radiusHalfExtents,
        defaultQueryFilter,
    );

    final endTilePolyResult = findNearestPoly(
        _connectOffMeshConnection_nearestPolyEnd,
        navMesh,
        offMeshConnection.end,
        radiusHalfExtents,
        defaultQueryFilter,
    );

    // exit if we couldn't find a start or an end poly, can't connect off mesh connection
    if (!startTilePolyResult.success || !endTilePolyResult.success) {
        return false;
    }

    // get start and end poly nodes
    final startNodeRef = startTilePolyResult.nodeRef;
    final startNode = getNodeByRef(navMesh, startNodeRef);

    final endNodeRef = endTilePolyResult.nodeRef;
    final endNode = getNodeByRef(navMesh, endNodeRef);

    // create off mesh connection state, for quick revalidation of connections when adding and removing tiles
    final offMeshConnectionState = OffMeshConnectionAttachment(
      offMeshNode: -1,
      startPolyNode: startNodeRef,
      endPolyNode: endNodeRef,
    );

    navMesh.offMeshConnectionAttachments[offMeshConnection.id] = offMeshConnectionState;

    // create a node for the off mesh connection
    final offMeshNode = allocateNode(navMesh);
    final offMeshNodeRef = serNodeRef(NodeType.offMesh, offMeshNode.index, offMeshConnection.sequence);
    offMeshNode.type = NodeType.offMesh.value;
    offMeshNode.ref = offMeshNodeRef;
    offMeshNode.area = offMeshConnection.area;
    offMeshNode.flags = offMeshConnection.flags;
    offMeshNode.offMeshConnectionId = offMeshConnection.id;

    offMeshConnectionState.offMeshNode = offMeshNodeRef;

    // start poly -> off mesh node -> end poly
    createOffMeshLink(navMesh, startNode!, offMeshNode, 0);
    createOffMeshLink(navMesh, offMeshNode, endNode!, 1);

    if (offMeshConnection.direction == OffMeshConnectionDirection.bidirectional) {
        // end poly -> off mesh node -> start poly
        createOffMeshLink(navMesh, endNode, offMeshNode, 1);
        createOffMeshLink(navMesh, offMeshNode, startNode, 0);
    }

    // connected the off mesh connection, return true
    return true;
}

bool disconnectOffMeshConnection(NavMesh navMesh, OffMeshConnection offMeshConnection) {
    final offMeshConnectionState = navMesh.offMeshConnectionAttachments[offMeshConnection.id];

    // the off mesh connection is not connected, return false
    if (offMeshConnectionState == null) return false;

    final offMeshConnectionNodeRef = offMeshConnectionState.offMeshNode;
    final startPolyNode = offMeshConnectionState.startPolyNode;
    final endPolyNode = offMeshConnectionState.endPolyNode;

    // release links in the start and end polys that reference off mesh connection nodes
    final startNode = getNodeByRef(navMesh, startPolyNode);

    if (startNode != null) {
      for (int i = startNode.links.length - 1; i >= 0; i--) {
        final linkId = startNode.links[i];
        final link = navMesh.links[linkId];
        if (link?.toNodeRef == offMeshConnectionNodeRef) {
          releaseLink(navMesh, linkId);
          startNode.links.removeAt(i);
        }
      }
    }

    final endNode = getNodeByRef(navMesh, endPolyNode);

    if (endNode != null) {
      for (int i = endNode.links.length - 1; i >= 0; i--) {
        final linkId = endNode.links[i];
        final link = navMesh.links[linkId];
        if (link?.toNodeRef == offMeshConnectionNodeRef) {
          releaseLink(navMesh, linkId);
          endNode.links.removeAt(i);
        }
      }
    }

    // release the off mesh node and links
    final offMeshNode = getNodeByRef(navMesh, offMeshConnectionNodeRef);

    if (offMeshNode != null) {
      for (int i = offMeshNode.links.length - 1; i >= 0; i--) {
        final linkId = offMeshNode.links[i];
        releaseLink(navMesh, linkId);
      }
    }

    releaseNode(navMesh, getNodeRefIndex(offMeshConnectionNodeRef));

    // remove the off mesh connection state
    navMesh.offMeshConnectionAttachments.remove(offMeshConnection.id);

    // the off mesh connection was disconnected, return true
    return true;
}

/// Reconnects an off mesh connection. This must be called if any properties of an off mesh connection are changed, for example the start or end positions.
/// @param navMesh the navmesh
/// @param offMeshConnection the off mesh connectionion to reconnect
/// @returns whether the off mesh connection was successfully reconnected
bool reconnectOffMeshConnection(NavMesh navMesh, OffMeshConnection offMeshConnection) {
  disconnectOffMeshConnection(navMesh, offMeshConnection);
  return connectOffMeshConnection(navMesh, offMeshConnection);
}

void updateOffMeshConnections(NavMesh navMesh) {
  for (final id in navMesh.offMeshConnections.keys) {
    final offMeshConnection = navMesh.offMeshConnections[id]!;
    final connected = isOffMeshConnectionConnected(navMesh, offMeshConnection.id);

    if (!connected) {
      reconnectOffMeshConnection(navMesh, offMeshConnection);
    }
  }
}

/// Builds a navmesh tile from the given parameters
/// This builds a BV-tree for the tile, and initializes runtime tile properties
/// @param params the parameters to build the tile from
/// @returns the built navmesh tile
NavMeshTile buildTile(NavMeshTileParams params) {
  final bvTree = buildNavMeshBvTree(params);

  final tile = NavMeshTile.set(
    params,
  )
    ..id = -1
    ..sequence = -1
    ..bvTree = bvTree
    ..polyNodes = [];

  return tile;
}

/// Adds a tile to the navmesh.
/// If a tile already exists at the same position, it will be removed first.
/// @param navMesh the navmesh to add the tile to
/// @param tile the tile to add
/// @returns void
void addTile(NavMesh navMesh, NavMeshTile tile) {
    final tilePositionHash = getTilePositionHash(tile.tileX, tile.tileY, tile.tileLayer);

    // remove any existing tile at the same position
    if (navMesh.tilePositionToTileId[tilePositionHash] != null) {
        removeTile(navMesh, tile.tileX, tile.tileY, tile.tileLayer);
    }

    // tile sequence
    int? sequence = navMesh.tilePositionToSequenceCounter[tilePositionHash];
    if (sequence == null) {
        sequence = 0;
    } else {
        sequence = (sequence + 1) % maxSequence;
    }

    navMesh.tilePositionToSequenceCounter[tilePositionHash] = sequence;

    // get tile id
    final id = IndexPool.requestIndex(navMesh.tileIndexPool).toInt();

    // set tile id and sequence
    tile.id = id;
    tile.sequence = sequence;

    // store tile in navmesh
    navMesh.tiles[tile.id] = tile;

    // store position lookup
    navMesh.tilePositionToTileId[tilePositionHash] = tile.id;

    // store column lookup
    final tileColumnHash = getTileColumnHash(tile.tileX, tile.tileY);
    if (navMesh.tileColumnToTileIds[tileColumnHash] == null) {
        navMesh.tileColumnToTileIds[tileColumnHash] = [];
    }
    navMesh.tileColumnToTileIds[tileColumnHash]!.add(tile.id);

    // allocate nodes
    for (int i = 0; i < (tile.polys?.length ?? 0); i++) {
        final node = allocateNode(navMesh);

        node.ref = serNodeRef(NodeType.poly, node.index, tile.sequence);
        node.type = NodeType.poly.value;
        node.area = tile.polys![i].area;
        node.flags = tile.polys![i].flags;
        node.tileId = tile.id;
        node.polyIndex = i;
        node.links.length = 0;

        tile.polyNodes.add(node.index);
    }

    // create internal links within the tile
    connectInternalLinks(navMesh, tile);

    // connect with layers in current tile.
    final tilesAtCurrentPosition = getTilesAt(navMesh, tile.tileX, tile.tileY);

    for (final tileAtCurrentPosition in tilesAtCurrentPosition) {
        if (tileAtCurrentPosition.id == tile.id) continue;

        connectExternalLinks(navMesh, tileAtCurrentPosition, tile, -1);
        connectExternalLinks(navMesh, tile, tileAtCurrentPosition, -1);
    }

    // connect with neighbouring tiles
    for (int side = 0; side < 8; side++) {
        final neighbourTiles = getNeighbourTilesAt(navMesh, tile.tileX, tile.tileY, side);

        for (final neighbourTile in neighbourTiles) {
            connectExternalLinks(navMesh, tile, neighbourTile, side);
            connectExternalLinks(navMesh, neighbourTile, tile, oppositeTile(side));
        }
    }

    // update off mesh connections
    updateOffMeshConnections(navMesh);
}

/// Removes the tile at the given location
/// @param navMesh the navmesh to remove the tile from
/// @param x the x coordinate of the tile
/// @param y the y coordinate of the tile
/// @param layer the layer of the tile
/// @returns true if the tile was removed, otherwise false
bool removeTile(NavMesh navMesh, int x, int y, int layer) {
    final tileHash = getTilePositionHash(x, y, layer);
    final tileId = navMesh.tilePositionToTileId[tileHash];
    final tile = navMesh.tiles[tileId];

    if (tile == null) {
        return false;
    }

    // disconnect external links with tiles in the same layer
    final tilesAtCurrentPosition = getTilesAt(navMesh, x, y);

    for (final tileAtCurrentPosition in tilesAtCurrentPosition) {
        if (tileAtCurrentPosition.id == tileId) continue;

        disconnectExternalLinks(navMesh, tileAtCurrentPosition, tile);
        disconnectExternalLinks(navMesh, tile, tileAtCurrentPosition);
    }

    // disconnect external links with neighbouring tiles
    for (int side = 0; side < 8; side++) {
        final neighbourTiles = getNeighbourTilesAt(navMesh, x, y, side);

        for (final neighbourTile in neighbourTiles) {
            disconnectExternalLinks(navMesh, neighbourTile, tile);
            disconnectExternalLinks(navMesh, tile, neighbourTile);
        }
    }

    // release internal links
    for (int polyIndex = 0; polyIndex < (tile.polys?.length ?? 0); polyIndex++) {
        final node = getNodeByTileAndPoly(navMesh, tile, polyIndex);

        for (final link in node?.links ?? []) {
            releaseLink(navMesh, link);
        }
    }

    // release nodes
    for (int i = 0; i < tile.polyNodes.length; i++) {
        releaseNode(navMesh, tile.polyNodes[i]);
    }
    tile.polyNodes.length = 0;

    // remove tile from navmesh
    navMesh.tiles.remove(tileId);

    // remove position lookup
    navMesh.tilePositionToTileId.remove(tileHash);

    // remove column lookup
    final tileColumnHash = getTileColumnHash(x, y);
    final tileColumn = navMesh.tileColumnToTileIds[tileColumnHash];
    if (tileColumn != null) {
        final tileIndexInColumn = tileColumn.indexOf(tileId!);
        if (tileIndexInColumn != -1) {
            tileColumn.removeAt(tileIndexInColumn);
        }
        if (tileColumn.isEmpty) {
            navMesh.tileColumnToTileIds.remove(tileColumnHash);
        }
    }

    // release tile index to the pool
    IndexPool.releaseIndex(navMesh.tileIndexPool, tileId!);

    // update off mesh connections
    updateOffMeshConnections(navMesh);

    return true;
}

/// Adds a new off mesh connection to the NavMesh, and returns it's ID
/// @param navMesh the navmesh to add the off mesh connection to
/// @param offMeshConnectionParams the parameters of the off mesh connection to add
/// @returns the ID of the added off mesh connection
int addOffMeshConnection(NavMesh navMesh, OffMeshConnection offMeshConnectionParams) {
  final id = IndexPool.requestIndex(navMesh.offMeshConnectionIndexPool).toInt();

  final sequence = navMesh.offMeshConnectionSequenceCounter;
  navMesh.offMeshConnectionSequenceCounter = (navMesh.offMeshConnectionSequenceCounter + 1) % maxSequence;

  final offMeshConnection = OffMeshConnection.copy(offMeshConnectionParams)
    ..id = id
    ..sequence = sequence;

  navMesh.offMeshConnections[id] = offMeshConnection;

  connectOffMeshConnection(navMesh, offMeshConnection);

  return id;
}

/// Removes an off mesh connection from the NavMesh
/// @param navMesh the navmesh to remove the off mesh connection from
/// @param offMeshConnectionId the ID of the off mesh connection to remove
void removeOffMeshConnection(NavMesh navMesh, int offMeshConnectionId) {
  final offMeshConnection = navMesh.offMeshConnections[offMeshConnectionId];

  if (offMeshConnection == null) return;

  IndexPool.releaseIndex(navMesh.offMeshConnectionIndexPool, offMeshConnection.id);

  disconnectOffMeshConnection(navMesh, offMeshConnection);

  navMesh.offMeshConnections.remove(offMeshConnection.id);
}

/// Returns whether the off mesh connection with the given ID is currently connected to the navmesh.
/// An off mesh connection may be disconnected if the start or end positions have no valid polygons nearby to connect to.
/// @param navMesh the navmesh
/// @param offMeshConnectionId the ID of the off mesh connection
/// @returns whether the off mesh connection is connected
bool isOffMeshConnectionConnected(NavMesh navMesh, int offMeshConnectionId) {
  final offMeshConnectionState = navMesh.offMeshConnectionAttachments[offMeshConnectionId];

  // no off mesh connection state, not connected
  if (offMeshConnectionState == null) return false;

  final startPolyNode = offMeshConnectionState.startPolyNode;
  final endPolyNode = offMeshConnectionState.endPolyNode;

  // valid if both the start and end poly node refs are valid
  return isValidNodeRef(navMesh, startPolyNode) && isValidNodeRef(navMesh, endPolyNode);
}

/// A query filter used in navigation queries.
///
/// This allows for customization of what nodes are considered traversable, and
/// the cost of traversing between nodes.
///
/// If you are getting started, you can use the built-in @see DEFAULT_QUERY_FILTER or @see ANY_QUERY_FILTER
/// A query filter used in navigation queries.
class QueryFilter {
  final bool Function(int nodeRef, NavMesh navMesh)? _customPassFilter;
  final double Function(
    Vector3 pa,
    Vector3 pb,
    NavMesh navMesh,
    int? prevRef,
    int curRef,
    int? nextRef,
  )? _customGetCost;

  // Constructor accepts optional lambda callbacks
  QueryFilter({
    bool Function(int nodeRef, NavMesh navMesh)? passFilter,
    double Function(
      Vector3 pa,
      Vector3 pb,
      NavMesh navMesh,
      int? prevRef,
      int curRef,
      int? nextRef,
    )? getCost,
  })  : _customPassFilter = passFilter,
        _customGetCost = getCost;

  /// Checks if a NavMesh node passes the filter.
  bool passFilter(int nodeRef, NavMesh navMesh) {
    // If a runtime custom lambda was provided, use it. Otherwise, use class logic.
    if (_customPassFilter != null) {
      return _customPassFilter(nodeRef, navMesh);
    }
    return true;
  }

  /// Calculates the cost of moving from one point to another.
  double getCost(
    Vector3 pa,
    Vector3 pb,
    NavMesh navMesh,
    int? prevRef,
    int curRef,
    int? nextRef,
  ) {
    // If a runtime custom lambda was provided, use it. Otherwise, use class logic.
    if (_customGetCost != null) {
      return _customGetCost(pa, pb, navMesh, prevRef, curRef, nextRef);
    }
    return pa.distanceTo(pb);
  }
}

/// A fully open filter instance that permits crossing any node reference 
/// while mapping raw distance directly to cost
class AnyQueryFilter extends QueryFilter {
  AnyQueryFilter();

  @override
  double getCost(
    Vector3 pa, 
    Vector3 pb, 
    NavMesh navMesh, 
    int? prevRef, 
    int curRef, 
    int? nextRef
  ) {
    return pa.distanceTo(pb);
  }

  @override
  bool passFilter(int nodeRef, NavMesh navMesh) {
    return true;
  }
}

// Global instance matching the TS finalant export requirement
final QueryFilter anyQueryFilter = AnyQueryFilter();

/// A standard flag-checked path filter implementation tracking specific inclusive/exclusive masks
class DefaultQueryFilter extends QueryFilter {
  int includeFlags;
  int excludeFlags;

  DefaultQueryFilter({
    this.includeFlags = 0xffffffff,
    this.excludeFlags = 0,
  });

  @override
  double getCost(
    Vector3 pa, 
    Vector3 pb, 
    NavMesh navMesh, 
    int? prevRef, 
    int curRef, 
    int? nextRef
  ) {
    return pa.distanceTo(pb);
  }

  @override
  bool passFilter(int nodeRef, NavMesh navMesh) {
    // getNodeByRef extracts structural parameters from your engine instance
    final dynamic node = getNodeByRef(navMesh, nodeRef);
    final int flags = node.flags as int;
    
    return (flags & includeFlags) != 0 && (flags & excludeFlags) == 0;
  }
}

// Global default reference tracking instance template
final QueryFilter defaultQueryFilter = DefaultQueryFilter();
