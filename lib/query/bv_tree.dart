import 'package:navcat/generate/common.dart';
import 'package:three_js_math/three_js_math.dart';
import './nav_mesh.dart';

int compareItemX(NavMeshBvNode a, NavMeshBvNode b) {
  if (a.bounds.min[0] < b.bounds.min[0]) return -1;
  if (a.bounds.min[0] > b.bounds.min[0]) return 1;
  return 0;
}

int compareItemY(NavMeshBvNode a, NavMeshBvNode b) {
  if (a.bounds.min[1] < b.bounds.min[1]) return -1;
  if (a.bounds.min[1] > b.bounds.min[1]) return 1;
  return 0;
}

int compareItemZ(NavMeshBvNode a, NavMeshBvNode b) {
  if (a.bounds.min[2] < b.bounds.min[2]) return -1;
  if (a.bounds.min[2] > b.bounds.min[2]) return 1;
  return 0;
}

BoundingBox calcExtends(List<NavMeshBvNode> items, int imin, int imax) {
  final bounds = BoundingBox(
    items[imin].bounds.min,
    items[imin].bounds.max,
  );

  for (int i = imin + 1; i < imax; ++i) {
    final it = items[i];
    if (it.bounds.min[0] < bounds.min[0]) bounds.min[0] = it.bounds.min[0];
    if (it.bounds.min[1] < bounds.min[1]) bounds.min[1] = it.bounds.min[1];
    if (it.bounds.min[2] < bounds.min[2]) bounds.min[2] = it.bounds.min[2];

    if (it.bounds.max[0] > bounds.max[0]) bounds.max[0] = it.bounds.max[0];
    if (it.bounds.max[1] > bounds.max[1]) bounds.max[1] = it.bounds.max[1];
    if (it.bounds.max[2] > bounds.max[2]) bounds.max[2] = it.bounds.max[2];
  }

  return bounds;
}

int longestAxis(double x, double y, double z) {
  int axis = 0;
  double maxVal = x;
  if (y > maxVal) {
    axis = 1;
    maxVal = y;
  }
  if (z > maxVal) {
    axis = 2;
  }
  return axis;
}

void subdivide(
  List<NavMeshBvNode> items,
  int imin,
  int imax,
  Nint curNode,
  List<NavMeshBvNode> nodes,
) {
  final inum = imax - imin;
  final icur = curNode.value;

  final node = NavMeshBvNode(
    bounds: BoundingBox(Vector3(),Vector3()),
    i: 0,
  );
  nodes[curNode.value++] = node;

  if (inum == 1) {
    node.bounds.min[0] = items[imin].bounds.min[0];
    node.bounds.min[1] = items[imin].bounds.min[1];
    node.bounds.min[2] = items[imin].bounds.min[2];

    node.bounds.max[0] = items[imin].bounds.max[0];
    node.bounds.max[1] = items[imin].bounds.max[1];
    node.bounds.max[2] = items[imin].bounds.max[2];

    node.i = items[imin].i;
  } else {
    // Split
    final extents = calcExtends(items, imin, imax);
    node.bounds.min[0] = extents.min[0];
    node.bounds.min[1] = extents.min[1];
    node.bounds.min[2] = extents.min[2];
    node.bounds.max[0] = extents.max[0];
    node.bounds.max[1] = extents.max[1];
    node.bounds.max[2] = extents.max[2];

    final axis = longestAxis(
      node.bounds.max[0] - node.bounds.min[0],
      node.bounds.max[1] - node.bounds.min[1],
      node.bounds.max[2] - node.bounds.min[2],
    );

    if (axis == 0) {
      // Sort along x-axis
      final segment = items.sublist(imin, imax);
      segment.sort(compareItemX);
      for (int i = 0; i < segment.length; i++) {
        items[imin + i] = segment[i];
      }
    } else if (axis == 1) {
      // Sort along y-axis
      final segment = items.sublist(imin, imax);
      segment.sort(compareItemY);
      for (int i = 0; i < segment.length; i++) {
        items[imin + i] = segment[i];
      }
    } else {
      // Sort along z-axis
      final segment = items.sublist(imin, imax);
      segment.sort(compareItemZ);
      for (int i = 0; i < segment.length; i++) {
        items[imin + i] = segment[i];
      }
    }

    final isplit = imin + (inum / 2).floor();

    // Left
    subdivide(items, imin, isplit, curNode, nodes);
    // Right
    subdivide(items, isplit, imax, curNode, nodes);

    final int iescape = curNode.value - icur;
    // Negative index means escape.
    node.i = -iescape;
  }
}

///
/// Builds a bounding volume tree for the given nav mesh tile.
/// @param navMeshTile the nav mesh tile to build the BV tree for
/// @returns
///
NavMeshTileBvTree buildNavMeshBvTree(
  NavMeshTileParams params
) {
  // use cellSize for quantization factor
  final quantFactor = 1 / params.cellSize;
  
  // early exit if the tile has no polys
  if (params.polys.length == 0) {
    return NavMeshTileBvTree(
      nodes: [],
      quantFactor: quantFactor,
    );
  }

  // allocate bv tree nodes for polys
  final List<NavMeshBvNode> items = [];

  for(int i = 0; i < params.polys.length;i++){
    items.add(NavMeshBvNode());
  }

  // calculate bounds for each polygon
  for (int i = 0; i < params.polys.length; i++) {
    final item = NavMeshBvNode(
      bounds: BoundingBox(Vector3(),Vector3()),
      i: i,
    );

    final poly = params.polys[i];
    final nvp = poly.vertices.length;

    if (nvp > 0) {
      // expand bounds with polygon vertices
      final firstVertIndex = poly.vertices[0] * 3;

      item.bounds.min[0] = item.bounds.max[0] = params.vertices[firstVertIndex];
      item.bounds.min[1] = item.bounds.max[1] = params.vertices[firstVertIndex + 1];
      item.bounds.min[2] = item.bounds.max[2] = params.vertices[firstVertIndex + 2];

      for (int j = 1; j < nvp; j++) {
        final vertexIndex = poly.vertices[j];
        if (vertexIndex == meshNullIdx) break;

        final vertIndex = vertexIndex * 3;
        final x = params.vertices[vertIndex];
        final y = params.vertices[vertIndex + 1];
        final z = params.vertices[vertIndex + 2];

        if (x < item.bounds.min[0]) item.bounds.min[0] = x;
        if (y < item.bounds.min[1]) item.bounds.min[1] = y;
        if (z < item.bounds.min[2]) item.bounds.min[2] = z;

        if (x > item.bounds.max[0]) item.bounds.max[0] = x;
        if (y > item.bounds.max[1]) item.bounds.max[1] = y;
        if (z > item.bounds.max[2]) item.bounds.max[2] = z;
      }

      // expand bounds with additional detail vertices if available
      if (params.detailMeshes.length > 0 && params.detailVertices.length > 0) {
        final detailMesh = params.detailMeshes[i];
        final vb = detailMesh!.verticesBase;
        final ndv = detailMesh.verticesCount;

        // iterate through additional detail vertices (not including poly vertices)
        for (int j = 0; j < ndv; j++) {
          final vertIndex = (vb + j) * 3;
          final x = params.detailVertices[vertIndex];
          final y = params.detailVertices[vertIndex + 1];
          final z = params.detailVertices[vertIndex + 2];

          if (x < item.bounds.min[0]) item.bounds.min[0] = x;
          if (y < item.bounds.min[1]) item.bounds.min[1] = y;
          if (z < item.bounds.min[2]) item.bounds.min[2] = z;

          if (x > item.bounds.max[0]) item.bounds.max[0] = x;
          if (y > item.bounds.max[1]) item.bounds.max[1] = y;
          if (z > item.bounds.max[2]) item.bounds.max[2] = z;
        }
      }

      // bv tree uses cellSize for all dimensions, quantize relative to tile bounds
      item.bounds.min[0] = (item.bounds.min[0] - params.bounds.min.x) * quantFactor;
      item.bounds.min[1] = (item.bounds.min[1] - params.bounds.min.y) * quantFactor;
      item.bounds.min[2] = (item.bounds.min[2] - params.bounds.min.z) * quantFactor;

      item.bounds.max[0] = (item.bounds.max[0] - params.bounds.min.x) * quantFactor;
      item.bounds.max[1] = (item.bounds.max[1] - params.bounds.min.y) * quantFactor;
      item.bounds.max[2] = (item.bounds.max[2] - params.bounds.min.z) * quantFactor;
    }

    items[i] = item;
  }

  Nint curNode = Nint();
  final nPolys = params.polys.length;
  final List<NavMeshBvNode> nodes = [];

  for(int i = 0; i < nPolys * 2;i++){
    nodes.add(NavMeshBvNode());
  }

  subdivide(items, 0, nPolys, curNode, nodes);

  // trim the nodes array to actual size
  final trimmedNodes = nodes.sublist(0, curNode.value);

  final bvTree = NavMeshTileBvTree(
    nodes: trimmedNodes,
    quantFactor: quantFactor,
  );

  return bvTree;
}
