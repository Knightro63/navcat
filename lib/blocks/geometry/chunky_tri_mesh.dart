import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';

/*
 * Spatial chunking utility for triangles based on Recast's ChunkyTriMesh.
 *
 * This builds a hierarchical spatial data structure (binary tree) that allows
 * efficient querying of triangles overlapping with spatial regions, avoiding
 * the need to test all triangles against each tile.
 */

class ChunkyTriMeshNode{
  int count;
  int index;
  final BoundingBox bounds;

  ChunkyTriMeshNode({
    required this.bounds,
    this.index = 0,
    this.count = 0
  });
}

class BoundsItem {
  final BoundingBox bounds;
  int index;

  BoundsItem({
    required this.bounds,
    this.index = 0
  });
}

/** Spatial chunking structure for triangles */
class ChunkyTriMesh{
  late final List<ChunkyTriMeshNode> nodes;
  late final List<int> triangles;
  int maxTrisPerChunk;
  
  ChunkyTriMesh({
    List<int>? triangles,
    this.maxTrisPerChunk = 0,
    List<ChunkyTriMeshNode>? nodes
  }){
    this.triangles = triangles ?? [];
    this.nodes = nodes ?? [];
  }

  static ChunkyTriMesh create(List<double> vertices, List<int> indices, [int trisPerChunk = 256]){
    final numTriangles = indices.length ~/ 3;

    // build bounding items for all triangles
    final List<BoundsItem> items = [];
    for (int i = 0; i < numTriangles; i++) {
      items.add(BoundsItem(
        bounds: calculateTriangleBounds(vertices, indices, i),
        index: i,
      ));
    }

    // build spatial tree
    final List<ChunkyTriMeshNode> nodes = [];
    final List<int> triangles = [];

    subdivide(items, 0, numTriangles, trisPerChunk, nodes, triangles, indices);

    // calculate max triangles per chunk
    int maxTrisPerChunk = 0;
    for (final node in nodes) {
      if (node.index >= 0 && node.count > maxTrisPerChunk) {
        maxTrisPerChunk = node.count;
      }
    }

    return ChunkyTriMesh(
      nodes: nodes,
      triangles: triangles,
      maxTrisPerChunk: maxTrisPerChunk,
    );
  }

  static BoundingBox calculateTriangleBounds(List<double> vertices, List<int> indices, int triIndex){
    final i0 = indices[triIndex * 3 + 0] * 3;
    final i1 = indices[triIndex * 3 + 1] * 3;
    final i2 = indices[triIndex * 3 + 2] * 3;

    final v0x = vertices[i0 + 0];
    final v0z = vertices[i0 + 2];
    final v1x = vertices[i1 + 0];
    final v1z = vertices[i1 + 2];
    final v2x = vertices[i2 + 0];
    final v2z = vertices[i2 + 2];

    return BoundingBox(
      Vector3(math.min(v0x, math.min(v1x, v2x)), math.min(v0z, math.min(v1z, v2z))),
      Vector3(math.max(v0x, math.max(v1x, v2x)), math.max(v0z, math.max(v1z, v2z))),
    );
  }

  static BoundingBox calculateExtents(List<BoundsItem> items, int min, int max){
    final BoundingBox bounds = BoundingBox(
      Vector3(items[min].bounds.min[0], items[min].bounds.min[1]),
      Vector3(items[min].bounds.max[0], items[min].bounds.max[1]),
    );

    for (int i = min + 1; i < max; i++) {
      final item = items[i];
      bounds.min[0] = math.min(bounds.min[0], item.bounds.min[0]);
      bounds.min[1] = math.min(bounds.min[1], item.bounds.min[1]);
      bounds.max[0] = math.max(bounds.max[0], item.bounds.max[0]);
      bounds.max[1] = math.max(bounds.max[1], item.bounds.max[1]);
    }

    return bounds;
  }

  static int longestAxis(num x, num y){
    return y > x ? 1 : 0;
  }

  static void subdivide(
      List<BoundsItem> items,
      int min,
      int max,
      int trisPerChunk,
      List<ChunkyTriMeshNode> nodes,
      List<int> outTriangles,
      List<int> inTriangles,
  ){
      final nodeIndex = nodes.length;
      final count = max - min;

      final ChunkyTriMeshNode node  = ChunkyTriMeshNode(
        bounds: BoundingBox(Vector3(),Vector3()),
        index: 0,
        count: 0,
      );

      nodes.add(node);

      if (count <= trisPerChunk) {
        // leaf node - calculate bounds and copy triangles
        node.bounds.setFrom(calculateExtents(items, min, max));
        node.index = outTriangles.length ~/ 3;
        node.count = count;

        // copy triangle indices
        for (int i = min; i < max; i++) {
          final triIndex = items[i].index;
          outTriangles.addAll([inTriangles[triIndex * 3 + 0], inTriangles[triIndex * 3 + 1], inTriangles[triIndex * 3 + 2]]);
        }
      } else {
        // internal node - split along longest axis
        node.bounds.setFrom(calculateExtents(items, min, max));

        final axis = longestAxis(node.bounds.max[0] - node.bounds.min[0], node.bounds.max[1] - node.bounds.min[1]);

        // sort items along the chosen axis (in-place sort of the range [min, max))
        final sorted = items.sublist(min, max)..sort((a, b){
            return (a.bounds.min[axis] - b.bounds.min[axis]).toInt();
        });

        for (int i = 0; i < sorted.length; i++) {
          items[min + i] = sorted[i];
        }

        final split = min + (count / 2).floor();

        // recursively build left and right subtrees
        subdivide(items, min, split, trisPerChunk, nodes, outTriangles, inTriangles);
        subdivide(items, split, max, trisPerChunk, nodes, outTriangles, inTriangles);

        // store escape index (negative to indicate internal node)
        final escapeIndex = nodes.length - nodeIndex;
        node.index = -escapeIndex;
      }
  }

  /**
   * Create a chunky triangle mesh from vertices and indices
   *
   * @param vertices flat array of vertex positions [x, y, z, x, y, z, ...]
   * @param indices flat array of triangle indices [i0, i1, i2, i0, i1, i2, ...]
   * @param trisPerChunk target number of triangles per leaf chunk (default: 256)
   * @returns ChunkyTriMesh spatial data structure
   */

  static bool checkOverlapRect(Vector2 aMin, Vector2 aMax, Vector bMin, Vector bMax){
    if (aMin[0] > bMax[0] || aMax[0] < bMin[0]) return false;
    if (aMin[1] > bMax[1] || aMax[1] < bMin[1]) return false;
    return true;
  }

  /**
   * Get all triangle chunks that overlap with a rectangular region
   *
   * @param chunkyTriMesh the chunky tri mesh to query
   * @param boundsMin minimum corner of query rectangle [x, z]
   * @param boundsMax maximum corner of query rectangle [x, z]
   * @returns Array of node indices that overlap the query region
   */
  static List<int> getChunksOverlappingRect(ChunkyTriMesh chunkyTriMesh, Vector2 boundsMin, Vector2 boundsMax){
    final nodes = chunkyTriMesh.nodes;
    final List<int> result = [];

    // traverse tree
    int i = 0;
    while (i < nodes.length) {
      final node = nodes[i];
      final overlap = checkOverlapRect(boundsMin, boundsMax, node.bounds.min, node.bounds.max);
      final isLeaf = node.index >= 0;

      if (isLeaf && overlap) {
        result.add(i);
      }

      if (overlap || isLeaf) {
        i++;
      } else {
        // skip this subtree using escape index
        i += -node.index;
      }
    }

    return result;
  }

  /**
   * Get all triangles that overlap with a rectangular region
   *
   * @param chunkyTriMesh the chunky tri mesh to query
   * @param boundsMin minimum corner of query rectangle [x, z]
   * @param boundsMax maximum corner of query rectangle [x, z]
   * @returns Flat array of triangle indices [i0, i1, i2, i0, i1, i2, ...]
   */
  static List<int> getTrianglesInRect(ChunkyTriMesh chunkyTriMesh, Vector2 boundsMin, Vector2 boundsMax){
    final chunks = getChunksOverlappingRect(chunkyTriMesh, boundsMin, boundsMax);
    final List<int> result = [];

    for (final chunkIndex in chunks) {
      final node = chunkyTriMesh.nodes[chunkIndex];
      final startIndex = node.index * 3;
      final endIndex = startIndex + node.count * 3;

      for (int i = startIndex; i < endIndex; i++) {
        result.add(chunkyTriMesh.triangles[i]);
      }
    }

    return result;
  }

  /**
   * Check if a line segment overlaps with a 2D bounding box
   */
  static bool checkOverlapSegment(Vector2 p, Vector2 q, Vector bMin, Vector bMax){
    final EPSILON = 1e-6;

    double tMin = 0;
    double tMax = 1;
    final Vector2 d = Vector2(q[0] - p[0], q[1] - p[1]);

    for (int i = 0; i < 2; i++) {
      if (d[i].abs() < EPSILON) {
        // Ray is parallel to slab
        if (p[i] < bMin[i] || p[i] > bMax[i]) {
          return false;
        }
      } else {
        // Compute intersection t values
        final ood = 1.0 / d[i];
        double t1 = (bMin[i] - p[i]) * ood;
        double t2 = (bMax[i] - p[i]) * ood;

        if (t1 > t2) {
          [t1, t2] = [t2, t1];
        }

        tMin = math.max(tMin, t1);
        tMax = math.min(tMax, t2);

        if (tMin > tMax) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Get all triangle chunks that overlap with a line segment
   *
   * @param chunkyTriMesh the chunky tri mesh to query
   * @param p start point of segment [x, z]
   * @param q end point of segment [x, z]
   * @returns Array of node indices that overlap the segment
   */
  static List<int> getChunksOverlappingSegment(ChunkyTriMesh chunkyTriMesh, Vector2 p, Vector2 q){
    final nodes = chunkyTriMesh.nodes;
    final List<int> result = [];

    // traverse tree
    int i = 0;
    while (i < nodes.length) {
      final node = nodes[i];
      final overlap = checkOverlapSegment(p, q, node.bounds.min, node.bounds.max);
      final isLeaf = node.index >= 0;

      if (isLeaf && overlap) {
        result.add(i);
      }

      if (overlap || isLeaf) {
        i++;
      } else {
        // skip this subtree using escape index
        i += -node.index;
      }
    }

    return result;
  }
}