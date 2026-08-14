import 'dart:typed_data';
import 'package:three_js_math/three_js_math.dart';
import '../../navcat.dart';

class TiledNavMeshInput {
  final Float32List positions;
  final Int32List indices;

  TiledNavMeshInput({
    required this.positions,
    required this.indices,
  });
}

class TiledNavMeshOptions {
  final double cellSize;
  final double cellHeight;
  final int tileSizeVoxels;
  final double tileSizeWorld;
  final double walkableRadiusVoxels;
  final double walkableRadiusWorld;
  final double walkableClimbVoxels;
  final double walkableClimbWorld;
  final double walkableHeightVoxels;
  final double walkableHeightWorld;
  final double walkableSlopeAngleDegrees;
  final int borderSize;
  final double minRegionArea;
  final double mergeRegionArea;
  final double maxSimplificationError;
  final double maxEdgeLength;
  final int maxVerticesPerPoly;
  final double detailSampleDistance;
  final double detailSampleMaxError;

  TiledNavMeshOptions({
    required this.cellSize,
    required this.cellHeight,
    required this.tileSizeVoxels,
    required this.tileSizeWorld,
    required this.walkableRadiusVoxels,
    required this.walkableRadiusWorld,
    required this.walkableClimbVoxels,
    required this.walkableClimbWorld,
    required this.walkableHeightVoxels,
    required this.walkableHeightWorld,
    required this.walkableSlopeAngleDegrees,
    required this.borderSize,
    required this.minRegionArea,
    required this.mergeRegionArea,
    required this.maxSimplificationError,
    required this.maxEdgeLength,
    required this.maxVerticesPerPoly,
    required this.detailSampleDistance,
    required this.detailSampleMaxError,
  });
}

class TiledNavMeshIntermediates {
  final BuildContextState buildContext;
  final TiledNavMeshInput input;
  final BoundingBox inputBounds; // Swapped Box3 to BoundingBox
  final ChunkyTriMesh chunkyTriMesh;
  final List<Uint8List> triAreaIds;
  final List<Heightfield> heightfield;
  final List<CompactHeightfield> compactHeightfield;
  final List<ContourSet> contourSet;
  final List<PolyMesh> polyMesh;
  final List<PolyMeshDetail> polyMeshDetail;

  TiledNavMeshIntermediates({
    required this.buildContext,
    required this.input,
    required this.inputBounds,
    required this.chunkyTriMesh,
    required this.triAreaIds,
    required this.heightfield,
    required this.compactHeightfield,
    required this.contourSet,
    required this.polyMesh,
    required this.polyMeshDetail,
  });
}

class TiledNavMeshResult {
  final NavMesh navMesh;
  final TiledNavMeshIntermediates intermediates;

  TiledNavMeshResult({
    required this.navMesh,
    required this.intermediates,
  });
}

// Assuming this return structural pattern from your class conversions
class NavMeshTileBuildResult {
  final Uint8List triAreaIds;
  final BoundingBox expandedTileBounds;
  final Heightfield heightfield;
  final CompactHeightfield compactHeightfield;
  final ContourSet contourSet;
  final PolyMesh polyMesh;
  final PolyMeshDetail polyMeshDetail;

  NavMeshTileBuildResult({
    required this.triAreaIds,
    required this.expandedTileBounds,
    required this.heightfield,
    required this.compactHeightfield,
    required this.contourSet,
    required this.polyMesh,
    required this.polyMeshDetail,
  });
}

NavMeshTileBuildResult buildNavMeshTile(
  BuildContextState ctx,
  Float32List positions,
  ChunkyTriMesh inputChunkyTriMesh,
  BoundingBox tileBounds,
  double cellSize,
  double cellHeight,
  int borderSize,
  double walkableSlopeAngleDegrees,
  double walkableClimbVoxels,
  double walkableHeightVoxels,
  double walkableRadiusVoxels,
  int tileSizeVoxels,
  double minRegionArea,
  double mergeRegionArea,
  double maxSimplificationError,
  double maxEdgeLength,
  int maxVerticesPerPoly,
  double detailSampleDistance,
  double detailSampleMaxError,
) {
  // 1. Clone bounding box and expand boundaries
  final expandedTileBounds = tileBounds.clone();
  expandedTileBounds.min.x -= borderSize * cellSize;
  expandedTileBounds.min.z -= borderSize * cellSize;
  expandedTileBounds.max.x += borderSize * cellSize;
  expandedTileBounds.max.z += borderSize * cellSize;

  /* 2. query chunks overlapping the tile bounds */
  final tbmin = Vector2(expandedTileBounds.min.x, expandedTileBounds.min.z);
  final tbmax = Vector2(expandedTileBounds.max.x, expandedTileBounds.max.z);
  final chunks = ChunkyTriMesh.getChunksOverlappingRect(inputChunkyTriMesh, tbmin, tbmax);

  /* 3. create heightfield for rasterization */
  final heightfieldWidth = (tileSizeVoxels + borderSize * 2).floor() * 1.0;
  final heightfieldHeight = (tileSizeVoxels + borderSize * 2).floor() * 1.0;
  final heightfield = createHeightfield(
    heightfieldWidth,
    heightfieldHeight,
    expandedTileBounds,
    cellSize,
    cellHeight,
  );

  /* 4. allocate triAreaIds for max chunk size */
  final triAreaIds = Uint8List(inputChunkyTriMesh.maxTrisPerChunk)..fillRange(0, inputChunkyTriMesh.maxTrisPerChunk, 0);

  /* 5. rasterize triangles chunk by chunk */
  for (final chunkIndex in chunks) {
    final node = inputChunkyTriMesh.nodes[chunkIndex];
    final startIdx = node.index * 3;
    final triangleCount = node.count;

    // Get this chunk's triangles using sublist
    final chunkTriangles = inputChunkyTriMesh.triangles.sublist(startIdx, startIdx + triangleCount * 3);

    // Reset area tracking array
    triAreaIds.fillRange(0, triAreaIds.length, 0);

    // Use sublistView to simulate memory-efficient subarray slices
    final activeAreaView = Uint8List.sublistView(triAreaIds, 0, triangleCount);

    markWalkableTriangles(
      positions,
      chunkTriangles,
      activeAreaView,
      walkableSlopeAngleDegrees,
    );

    rasterizeTriangles(
      ctx,
      heightfield,
      positions,
      chunkTriangles,
      activeAreaView,
      walkableClimbVoxels,
    );
  }

  /* 6. filter walkable surfaces */
  filterLowHangingWalkableObstacles(heightfield, walkableClimbVoxels);
  filterLedgeSpans(heightfield, walkableHeightVoxels, walkableClimbVoxels);
  filterWalkableLowHeightSpans(heightfield, walkableHeightVoxels);

  /* 7. build the compact heightfield */
  final compactHeightfield = buildCompactHeightfield(ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield);

  /* 8. erode the walkable area by the agent radius / walkable radius */
  erodeWalkableArea(walkableRadiusVoxels, compactHeightfield);

  /* 9. prepare for region partitioning */
  buildDistanceField(compactHeightfield);

  /* 10. partition the walkable surface into simple regions without holes */
  buildRegions(ctx, compactHeightfield, borderSize, minRegionArea, mergeRegionArea);

  /* 11. trace and simplify region contours */
  final contourSet = buildContours(
    ctx,
    compactHeightfield,
    maxSimplificationError,
    maxEdgeLength,
    ContourBuildFlags.contourTessWallEdges,
  );

  /* 12. build polygons mesh from contours */
  final polyMesh = buildPolyMesh(ctx, contourSet, maxVerticesPerPoly);

  for (int polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
    if (polyMesh.areas[polyIndex] == walkableArea) {
      polyMesh.areas[polyIndex] = 0;
    }
    if (polyMesh.areas[polyIndex] == 0) {
      polyMesh.flags[polyIndex] = 1;
    }
  }

  /* 13. create detail mesh which allows to access approximate height */
  final polyMeshDetail = buildPolyMeshDetail(
    ctx,
    polyMesh,
    compactHeightfield,
    detailSampleDistance,
    detailSampleMaxError,
  );

  return NavMeshTileBuildResult(
    triAreaIds: triAreaIds,
    expandedTileBounds: expandedTileBounds,
    heightfield: heightfield,
    compactHeightfield: compactHeightfield,
    contourSet: contourSet,
    polyMesh: polyMesh,
    polyMeshDetail: polyMeshDetail,
  );
}

TiledNavMeshResult generateTiledNavMesh(TiledNavMeshInput input, TiledNavMeshOptions options) {
  final positions = input.positions;
  final indices = input.indices;

  /* 0. define generation parameters */
  final cellSize = options.cellSize;
  final cellHeight = options.cellHeight;
  final tileSizeVoxels = options.tileSizeVoxels;
  final tileSizeWorld = options.tileSizeWorld;
  final walkableRadiusVoxels = options.walkableRadiusVoxels;
  final walkableRadiusWorld = options.walkableRadiusWorld;
  final walkableClimbVoxels = options.walkableClimbVoxels;
  final walkableClimbWorld = options.walkableClimbWorld;
  final walkableHeightVoxels = options.walkableHeightVoxels;
  final walkableHeightWorld = options.walkableHeightWorld;
  final walkableSlopeAngleDegrees = options.walkableSlopeAngleDegrees;
  final borderSize = options.borderSize;
  final minRegionArea = options.minRegionArea;
  final mergeRegionArea = options.mergeRegionArea;
  final maxSimplificationError = options.maxSimplificationError;
  final maxEdgeLength = options.maxEdgeLength;
  final maxVerticesPerPoly = options.maxVerticesPerPoly;
  final detailSampleDistance = options.detailSampleDistance;
  final detailSampleMaxError = options.detailSampleMaxError;

  final ctx = BuildContextState.create();

  /* 1. calculate mesh bounds and create tiled nav mesh */
  final meshBounds = calculateMeshBounds(BoundingBox(), positions, indices);
  final gridSize = calculateGridSize(Vector2(), meshBounds, cellSize);
  
  final nav = createNavMesh();
  nav.tileWidth = tileSizeWorld;
  nav.tileHeight = tileSizeWorld;
  
  // Assuming Box3.min(nav.origin, meshBounds) logic matches BoundingBox min setups
  nav.origin.setFrom(meshBounds.min);

  /* 2. build chunky tri mesh for efficient spatial queries */
  final inputChunkyTriMesh = ChunkyTriMesh.create(positions, indices);

  /* 3. initialize intermediates for debugging */
  final intermediates = TiledNavMeshIntermediates(
    buildContext: ctx,
    input: TiledNavMeshInput(positions: positions, indices: indices),
    inputBounds: meshBounds,
    chunkyTriMesh: inputChunkyTriMesh,
    triAreaIds: [],
    heightfield: [],
    compactHeightfield: [],
    contourSet: [],
    polyMesh: [],
    polyMeshDetail: [],
  );

  /* 4. generate tiles */
  final nTilesX = (gridSize.x + tileSizeVoxels - 1) ~/ tileSizeVoxels;
  final nTilesY = (gridSize.y + tileSizeVoxels - 1) ~/ tileSizeVoxels;

  for (int tileX = 0; tileX < nTilesX; tileX++) {
    for (int tileY = 0; tileY < nTilesY; tileY++) {
      
      final tileBounds = BoundingBox(
        Vector3(
          meshBounds.min.x + tileX * tileSizeWorld,
          meshBounds.min.y,
          meshBounds.min.z + tileY * tileSizeWorld,
        ),
        Vector3(
          meshBounds.min.x + (tileX + 1) * tileSizeWorld,
          meshBounds.max.y,
          meshBounds.min.z + (tileY + 1) * tileSizeWorld,
        ),
      );

      final result = buildNavMeshTile(
        ctx,
        positions,
        inputChunkyTriMesh,
        tileBounds,
        cellSize,
        cellHeight,
        borderSize,
        walkableSlopeAngleDegrees,
        walkableClimbVoxels,
        walkableHeightVoxels,
        walkableRadiusVoxels,
        tileSizeVoxels,
        minRegionArea,
        mergeRegionArea,
        maxSimplificationError,
        maxEdgeLength,
        maxVerticesPerPoly,
        detailSampleDistance,
        detailSampleMaxError,
      );

      if (result.polyMesh.vertices.isEmpty) continue;

      intermediates.triAreaIds.add(result.triAreaIds);
      intermediates.heightfield.add(result.heightfield);
      intermediates.compactHeightfield.add(result.compactHeightfield);
      intermediates.contourSet.add(result.contourSet);
      intermediates.polyMesh.add(result.polyMesh);
      intermediates.polyMeshDetail.add(result.polyMeshDetail);

      final tilePolys = polyMeshToTilePolys(result.polyMesh);
      final tileDetailMesh = polyMeshDetailToTileDetailMesh(tilePolys.polys, result.polyMeshDetail);

      final tileParams = NavMeshTileParams(
        bounds: result.polyMesh.bounds,
        vertices: tilePolys.vertices,
        polys: tilePolys.polys,
        detailMeshes: tileDetailMesh.detailMeshes,
        detailVertices: tileDetailMesh.detailVertices,
        detailTriangles: tileDetailMesh.detailTriangles,
        tileX: tileX,
        tileY: tileY,
        tileLayer: 0,
        cellSize: cellSize,
        cellHeight: cellHeight,
        walkableHeight: walkableHeightWorld,
        walkableRadius: walkableRadiusWorld,
        walkableClimb: walkableClimbWorld,
      );

      final tile = buildTile(tileParams);
      addTile(nav, tile);
    }
  }

  return TiledNavMeshResult(
    navMesh: nav,
    intermediates: intermediates,
  );
}
