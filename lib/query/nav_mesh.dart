import 'package:three_js_math/three_js_math.dart';
import '../index_pool.dart';

/// A navigation mesh based on tiles of convex polygons
class NavMesh {
  /// The world space origin of the navigation mesh's tiles.
  final Vector3 origin;

  /// The width of each tile along the x axis.
  double tileWidth;

  /// The height of each tile along the z axis.
  double tileHeight;

  ///   Global nodes.
  final Map<int,NavMeshNode?> nodes;

  /// Global links. Check 'allocated' for whether the link is in use.
  final Map<int,NavMeshLink?> links;

  /// Off mesh connection definitions.
  final Map<int, OffMeshConnection?> offMeshConnections;

  /// Off mesh connection attachments.
  final Map<int, OffMeshConnectionAttachment> offMeshConnectionAttachments;

  /// Map of tile ids to tiles.
  final Map<int, NavMeshTile?> tiles;

  /// Map of tile position hashes to tile ids.
  final Map<String, int> tilePositionToTileId;

  /// Map of tile column hashes to array of tile ids in that column.
  final Map<String, List<int>> tileColumnToTileIds;

  /// Map of tile position hashes to sequence counter.
  final Map<String, int> tilePositionToSequenceCounter;

  /// The off mesh connection sequence counter.
  int offMeshConnectionSequenceCounter;

  /// Pool for node indices.
  final IndexPool nodeIndexPool;

  /// Pool for tile indices.
  final IndexPool tileIndexPool;

  /// Pool for off mesh connection indices.
  final IndexPool offMeshConnectionIndexPool;

  /// Pool for link indices.
  final IndexPool linkIndexPool;

  NavMesh({
    required this.origin,
    required this.tileWidth,
    required this.tileHeight,
    required this.nodes,
    required this.links,
    required this.offMeshConnections,
    required this.offMeshConnectionAttachments,
    required this.tiles,
    required this.tilePositionToTileId,
    required this.tileColumnToTileIds,
    required this.tilePositionToSequenceCounter,
    required this.offMeshConnectionSequenceCounter,
    required this.nodeIndexPool,
    required this.tileIndexPool,
    required this.offMeshConnectionIndexPool,
    required this.linkIndexPool,
  });
}

class NavMeshPoly {
    /// The indices of the polygon's vertices. vertices are stored in NavMeshTile.vertices.
    final List<int> vertices;

    /// Packed data representing neighbor polygons references and flags for each edge.
    /// This is usually computed by the navcat's `buildPolyNeighbours` function .
    List<int> neis;

    /// The user defined flags for this polygon.
    final int flags;

    /// The user defined area id for this polygon.
    final int area;

    NavMeshPoly({
      required this.vertices,
      required this.neis,
      required this.flags,
      required this.area,
    });
}

class NavMeshPolyDetail {
    /// The offset of the vertices in the NavMeshTile detailVertices array.
    /// If the base index is between 0 and `NavMeshTile.vertices.length`, this is used to index into the NavMeshTile vertices array.
    /// If the base index is greater than `NavMeshTile.vertices.length`, it is used to index into the NavMeshTile detailVertices array.
    /// This allows for detail meshes to either re-use the polygon vertices or to define their own vertices without duplicating data.
    final int verticesBase;

    /// The offset of the triangles in the NavMeshTile detailTriangles array.
    final int trianglesBase;

    /// The number of vertices in thde sub-mesh.
    final int verticesCount;

    /// The number of triangles in the sub-mesh.
    final int trianglesCount;

    NavMeshPolyDetail({
      required this.verticesBase,
      required this.trianglesBase,
      required this.verticesCount,
      required this.trianglesCount,
    });
}

class NavMeshNode {
    /// Whether the node is currently allocated.
    bool allocated;

    /// The index of the node.
    int index;

    /// The node ref, packed type, index, sequence.
    int ref;

    /// Links for this nav mesh node.
    List<int> links;

    /// The user defined flags for this node.
    int flags;

    /// The user defined area id for this node.
    int area;

    /// The type of the nav mesh node.
    int type;

    /// The tile id, for poly nodes.
    int tileId;

    /// The poly index, for poly nodes.
    int polyIndex;

    /// The offmesh connection id, for offmesh connection nodes.
    int offMeshConnectionId;

    NavMeshNode({
      required this.allocated,
      required this.index,
      required this.ref,
      required this.links,
      required this.flags,
      required this.area,
      required this.type,
      required this.tileId,
      required this.polyIndex,
      required this.offMeshConnectionId,
    }); 
}

class NavMeshLink {
    /// Whether the nav mesh link is allocated.
    bool allocated;

    /// The index of the link.
    int index;

    /// The node index that owns this link.
    int fromNodeIndex;

    /// Node reference that owns this link.
    int fromNodeRef;

    /// Index of the neighbour node that ref links to.
    int toNodeIndex;

    /// The neighbour node reference that ref links to.
    int toNodeRef;

    /// Index of the polygon edge that owns this link.
    int edge;

    /// If a boundary link, defines on which side the link is.
    int side;

    /// If a boundary link, defines the min sub-edge area.
    double bmin;

    /// If a boundary link, defines the max sub-edge area.
    double bmax;

    NavMeshLink({
      required this.allocated,
      required this.index,
      required this.fromNodeIndex,
      required this.fromNodeRef,
      required this.toNodeIndex,
      required this.toNodeRef,
      required this.edge,
      required this.side,
      required this.bmin,
      required this.bmax,
    });   
}

enum OffMeshConnectionDirection {
  startToEnd,
  bidirectional,
}

class OffMeshConnection {
  /// The id of the off mesh connection.
  int id;
  /// The sequence of the off mesh connection.
  int sequence;
  /// The start position of the off mesh connection.
  final Vector3 start;
  /// The end position of the off mesh connection.
  final Vector3 end;
  /// The radius of the endpoints.
  final double radius;
  /// The direction of the off mesh connection.
  final OffMeshConnectionDirection direction;
  /// The flags for the off mesh connection.
  final int flags;
  /// The area id for the off mesh connection.
  final int area;

  OffMeshConnection({
    required this.id,
    required this.sequence,
    required this.start,
    required this.end,
    required this.radius,
    required this.direction,
    required this.flags,
    required this.area,
  });

  static OffMeshConnection copy(OffMeshConnection values) {
    return OffMeshConnection(
      id: values.id,
      sequence: values.sequence,
      start: values.start,
      end: values.end,
      radius: values.radius,
      direction: values.direction,
      flags: values.flags,
      area: values.area,
    );
  }
}

//typedef OffMeshConnectionParams = Omit<OffMeshConnection, 'id' | 'sequence'>;

class OffMeshConnectionAttachment {
    /// The offmesh node.
    int offMeshNode;

    /// The start polygon that the off mesh connection has linked to.
    int startPolyNode;

    /// The end polygon that the off mesh connection has linked to.
    int endPolyNode;

    OffMeshConnectionAttachment({
      required this.offMeshNode,
      required this.startPolyNode,
      required this.endPolyNode,
    });
}

class NavMeshBvNode {
    /// Bounds of the bv node.
    late BoundingBox bounds;
    /// The node's index.
    int i = 0;

    NavMeshBvNode({
      BoundingBox? bounds,
      this.i = 0,
    }){
      this.bounds = bounds ?? BoundingBox();
    }
}

class NavMeshTileBvTree {
  /// The tile bounding volume nodes.
  final List<NavMeshBvNode> nodes;

  /// The quantisation factor for the bounding volume tree.
  double quantFactor;

  NavMeshTileBvTree({
    required this.nodes,
    this.quantFactor = 0,
  });
}

class NavMeshTile {
  /// The id of the tile.
  int id;

  /// The salt of the tile.
  int sequence;

  /// The tile x position in the nav mesh.
  int tileX;

  /// The tile y position in the nav mesh.
  int tileY;

  /// The tile layer in the nav mesh.
  int tileLayer;

  /// The bounds of the tile's AABB.
  final BoundingBox bounds;

  /// Nav mesh tile vertices in world space.
  final List<double> vertices;

  /// The detail meshes.
  final List<NavMeshPolyDetail?> detailMeshes;

  /// The detail mesh's unique vertices, in local tile space.
  final List<double> detailVertices;

  /// The detail mesh's triangles.
  final List<int> detailTriangles;

  /// The tile polys.
  final List<NavMeshPoly>? polys;

  /// Poly index to global node index.
  List<int> polyNodes;

  /// The tile's bounding volume tree.
  NavMeshTileBvTree bvTree;

  ///
  /// The xz-plane cell size of the polygon mesh.
  /// If this tile was generated with voxelization, it should be the voxel cell size.
  /// If the tile was created with a different method, use a value that approximates the level of precision required for the tile.
  ///
  double cellSize;

  ///
  /// The y-axis cell height of the polygon mesh.
  /// If this tile was generated with voxelization, it should be the voxel cell height.
  /// If the tile was created with a different method, use a value that approximates the level of precision required for the tile.
  ///
  double cellHeight;

  ///
  /// The agent height in world units.
  ///
  double walkableHeight;

  ///
  /// The agent radius in world units.
  ///
  double walkableRadius;

  ///
  /// The agent maximum traversable ledge (up/down) in world units.
  ///
  double walkableClimb;

  NavMeshTile({
    required this.id,
    required this.sequence,
    required this.tileX,
    required this.tileY,
    required this.tileLayer,
    required this.bounds,
    required this.vertices,
    required this.detailMeshes,
    required this.detailVertices,
    required this.detailTriangles,
    required this.polys,
    required this.polyNodes,
    required this.bvTree,
    required this.cellSize,
    required this.cellHeight,
    required this.walkableHeight,
    required this.walkableRadius,
    required this.walkableClimb,
  });

  static NavMeshTile set(NavMeshTileParams parms){
    return NavMeshTile(
      id: parms.tileX,
      sequence: parms.tileY,
      tileX: parms.tileX,
      tileY: parms.tileY,
      tileLayer: parms.tileLayer,
      bounds: BoundingBox(
        Vector3(parms.bounds.min.x, parms.bounds.min.y, parms.bounds.min.z),
        Vector3(parms.bounds.max.x, parms.bounds.max.y, parms.bounds.max.z),
      ),
      vertices: List<double>.from(parms.vertices),
      detailMeshes: List<NavMeshPolyDetail>.from(parms.detailMeshes),
      detailVertices: List<double>.from(parms.detailVertices),
      detailTriangles: List<int>.from(parms.detailTriangles),
      polys: List<NavMeshPoly>.from(parms.polys),
      polyNodes: List<int>.filled(parms.polys.length, 0),
      bvTree: NavMeshTileBvTree(nodes: []),
      cellSize: parms.cellSize,
      cellHeight: parms.cellHeight,
      walkableHeight: parms.walkableHeight,
      walkableRadius: parms.walkableRadius,
      walkableClimb: parms.walkableClimb,
    );
  }
}

class NavMeshTileParams {
  /// The tile x position in the nav mesh.
  final int tileX;

  /// The tile y position in the nav mesh.
  final int tileY;

  /// The tile layer in the nav mesh.
  final int tileLayer;

  /// The bounds of the tile's AABB.
  final BoundingBox bounds;

  /// Nav mesh tile vertices in world space.
  final List<double> vertices;

  /// The detail meshes.
  final List<NavMeshPolyDetail?> detailMeshes;

  /// The detail mesh's unique vertices, in local tile space.
  final List<double> detailVertices;

  /// The detail mesh's triangles.
  final List<int> detailTriangles;

  /// The tile polys.
  final List<NavMeshPoly> polys;

  ///
  /// The xz-plane cell size of the polygon mesh.
  /// If this tile was generated with voxelization, it should be the voxel cell size.
  /// If the tile was created with a different method, use a value that approximates the level of precision required for the tile.
  ///
  final double cellSize;

  ///
  /// The y-axis cell height of the polygon mesh.
  /// If this tile was generated with voxelization, it should be the voxel cell height.
  /// If the tile was created with a different method, use a value that approximates the level of precision required for the tile.
  ///
  final double cellHeight;

  ///
  /// The agent height in world units.
  ///
  final double walkableHeight;

  ///
  /// The agent radius in world units.
  ///
  final double walkableRadius;

  ///
  /// The agent maximum traversable ledge (up/down) in world units.
  ///
  final double walkableClimb;

  NavMeshTileParams({
    required this.tileX,
    required this.tileY,
    required this.tileLayer,
    required this.bounds,
    required this.vertices,
    required this.detailMeshes,
    required this.detailVertices,
    required this.detailTriangles,
    required this.polys,
    required this.cellSize,
    required this.cellHeight,
    required this.walkableHeight,
    required this.walkableRadius,
    required this.walkableClimb,
  });
}
