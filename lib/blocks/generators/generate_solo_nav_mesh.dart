import 'dart:typed_data';
import 'package:navcat/math/vector.dart';
import 'package:navcat/navcat.dart';
import 'package:three_js_math/three_js_math.dart';

class SoloNavMeshInput {
  final List<double> positions;
  final List<int> indices;

  SoloNavMeshInput({
    required this.positions,
    required this.indices,
  });
}

class SoloNavMeshOptions {
  final double cellSize;
  final double cellHeight;
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

  SoloNavMeshOptions({
    required this.cellSize,
    required this.cellHeight,
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

class SoloNavMeshIntermediates {
  final BuildContextState buildContextState;
  final MeshInput input;
  final Uint8List triAreaIds;
  final Heightfield heightfield;
  final CompactHeightfield compactHeightfield;
  final ContourSet contourSet;
  final PolyMesh polyMesh;
  final PolyMeshDetail polyMeshDetail;

  SoloNavMeshIntermediates({
    required this.buildContextState,
    required this.input,
    required this.triAreaIds,
    required this.heightfield,
    required this.compactHeightfield,
    required this.contourSet,
    required this.polyMesh,
    required this.polyMeshDetail,
  });
}

class SoloNavMeshResult {
  final NavMesh navMesh;
  final SoloNavMeshIntermediates intermediates;

  SoloNavMeshResult({
    required this.navMesh,
    required this.intermediates,
  });
}

SoloNavMeshResult generateSoloNavMesh(MeshInput input, SoloNavMeshOptions options) {
  /* 1. create build context, gather inputs and options */
  final ctx = BuildContextState.create();
  BuildContextState.start(ctx, 'navmesh generation');
  
  final positions = input.positions;
  final indices = input.indices;
  
  final cellSize = options.cellSize;
  final cellHeight = options.cellHeight;
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

  /* 2. mark walkable triangles */
  BuildContextState.start(ctx, 'mark walkable triangles');
  final triAreaIds = Uint8List((indices.length / 3).floor());
  markWalkableTriangles(positions, indices, triAreaIds, walkableSlopeAngleDegrees);
  BuildContextState.end(ctx, 'mark walkable triangles');

  /* 3. rasterize the triangles to a voxel heightfield */
  BuildContextState.start(ctx, 'rasterize triangles');
  final bounds = calculateMeshBounds(BoundingBox(), positions, indices);
  final gridSize = calculateGridSize(Vector2(), bounds, cellSize);
  final heightfieldWidth = gridSize[0];
  final heightfieldHeight = gridSize[1];
  final heightfield = createHeightfield(heightfieldWidth, heightfieldHeight, bounds, cellSize, cellHeight);
  rasterizeTriangles(ctx, heightfield, positions, indices, triAreaIds, walkableClimbVoxels);
  BuildContextState.end(ctx, 'rasterize triangles');

  /* 4. filter walkable surfaces */
  BuildContextState.start(ctx, 'filter walkable surfaces');
  filterLowHangingWalkableObstacles(heightfield, walkableClimbVoxels);
  filterLedgeSpans(heightfield, walkableHeightVoxels, walkableClimbVoxels);
  filterWalkableLowHeightSpans(heightfield, walkableHeightVoxels);
  BuildContextState.end(ctx, 'filter walkable surfaces');

  /* 5. compact the heightfield */
  BuildContextState.start(ctx, 'build compact heightfield');
  final compactHeightfield = buildCompactHeightfield(ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield);
  BuildContextState.end(ctx, 'build compact heightfield');

  /* 6. erode the walkable area by the agent radius / walkable radius */
  BuildContextState.start(ctx, 'erode walkable area');
  erodeWalkableArea(walkableRadiusVoxels, compactHeightfield);
  BuildContextState.end(ctx, 'erode walkable area');

  /* 7. prepare for region partitioning by calculating a distance field along the walkable surface */
  BuildContextState.start(ctx, 'build compact heightfield distance field');
  buildDistanceField(compactHeightfield);
  BuildContextState.end(ctx, 'build compact heightfield distance field');

  /* 8. partition the walkable surface into simple regions without holes */
  BuildContextState.start(ctx, 'build compact heightfield regions');
  buildRegions(ctx, compactHeightfield, borderSize, minRegionArea, mergeRegionArea);
  BuildContextState.end(ctx, 'build compact heightfield regions');

  /* 9. trace and simplify region contours */
  BuildContextState.start(ctx, 'trace and simplify region contours');
  final contourSet = buildContours(
    ctx,
    compactHeightfield,
    maxSimplificationError,
    maxEdgeLength,
    ContourBuildFlags.contourTessWallEdges,
  );
  BuildContextState.end(ctx, 'trace and simplify region contours');

  /* 10. build polygons mesh from contours */
  BuildContextState.start(ctx, 'build polygons mesh from contours');
  final polyMesh = buildPolyMesh(ctx, contourSet, maxVerticesPerPoly);
  for (int polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
    if (polyMesh.areas[polyIndex] == walkableArea) {
      polyMesh.areas[polyIndex] = 0;
    }
    if (polyMesh.areas[polyIndex] == 0) {
      polyMesh.flags[polyIndex] = 1;
    }
  }
  BuildContextState.end(ctx, 'build polygons mesh from contours');

  /* 11. create detail mesh which allows to access approximate height on each polygon */
  BuildContextState.start(ctx, 'build detail mesh from contours');
  final polyMeshDetail = buildPolyMeshDetail(ctx, polyMesh, compactHeightfield, detailSampleDistance, detailSampleMaxError);
  BuildContextState.end(ctx, 'build detail mesh from contours');
  BuildContextState.end(ctx, 'navmesh generation');

  /* store intermediates for debugging */
  final intermediates = SoloNavMeshIntermediates(
    buildContextState: ctx,
    input: MeshInput(
      positions: positions,
      indices: indices,
    ),
    triAreaIds: triAreaIds,
    heightfield: heightfield,
    compactHeightfield: compactHeightfield,
    contourSet: contourSet,
    polyMesh: polyMesh,
    polyMeshDetail: polyMeshDetail,
  );

  /* create a single tile nav mesh */
  final nav = createNavMesh();
  nav.tileWidth = polyMesh.bounds.max.x - polyMesh.bounds.min.x;
  nav.tileHeight = polyMesh.bounds.max.z - polyMesh.bounds.min.z;
  box3.min.min2(nav.origin, polyMesh.bounds.min);
  
  final tilePolys = polyMeshToTilePolys(polyMesh);
  final tileDetailMesh = polyMeshDetailToTileDetailMesh(tilePolys.polys, polyMeshDetail);
  
  final tileParams = NavMeshTileParams(
    bounds: polyMesh.bounds,
    vertices: tilePolys.vertices,
    polys: tilePolys.polys,
    detailMeshes: tileDetailMesh.detailMeshes,
    detailVertices: tileDetailMesh.detailVertices,
    detailTriangles: tileDetailMesh.detailTriangles,
    tileX: 0,
    tileY: 0,
    tileLayer: 0,
    cellSize: cellSize,
    cellHeight: cellHeight,
    walkableHeight: walkableHeightWorld,
    walkableRadius: walkableRadiusWorld,
    walkableClimb: walkableClimbWorld,
  );
  
  final tile = buildTile(tileParams);
  addTile(nav, tile);

  return SoloNavMeshResult(
    navMesh: nav,
    intermediates: intermediates,
  );
}
