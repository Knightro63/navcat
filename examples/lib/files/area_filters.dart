import 'dart:async';
import 'dart:typed_data';
import 'package:examples/src/common/base.dart';
import 'package:examples/src/common/debug.dart';
import 'package:examples/src/gui.dart';
import 'package:flutter/material.dart';
import 'package:navcat/math/vector.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart';
import 'package:three_js_line/three_js_line.dart';

enum NavMeshAreaType {none,ground,water}

class AreaFilters extends StatefulWidget {
  const AreaFilters({super.key});
  @override
  createState() => _State();
}

class _State extends State<AreaFilters> {
  late ExampleBase base;
  three.Camera get camera => base.camera;
  three.Scene get scene => base.scene;
  three.PeripheralsState get domElement => base.domElement;
  late Gui gui;
  ThreeDebug get debug => base.debug;

  @override
  void initState() {
    base = ExampleBase(
      setState,
      setup,
    );
    gui = Gui((){setState(() {});});
    super.initState();
  }
  @override
  void dispose() {
    base.dispose();
    three.loading.clear();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          base.build,
          if(base.threeJs.mounted)Positioned(
            top: 20,
            right: 20,
            child: SizedBox(
              height: base.threeJs.height,
              width: 240,
              child: gui.render()
            )
          )  
        ],
      ) 
    );
  }

  late final three.OrbitControls orbitControls;

  final QueryFilter waterQueryFilter = QueryFilter(
    passFilter: (nodeRef, navMesh) {
        final node = getNodeByRef(navMesh, nodeRef);
        return node!.area == NavMeshAreaType.water.index;
    },
    getCost: defaultQueryFilter.getCost,
  );
  final QueryFilter groundQueryFilter = QueryFilter(
    passFilter: (nodeRef, navMesh) {
        final node = getNodeByRef(navMesh, nodeRef);
        return node!.area == NavMeshAreaType.ground.index;
    },
    getCost: defaultQueryFilter.getCost,
  );

  final raycaster = three.Raycaster();
  final pointer = three.Vector2();

  three.Vector3 start = three.Vector3(-8, 1.5, -2.3);
  three.Vector3 end = three.Vector3(8, 1, -0.5);
  final three.Vector3 halfExtents = three.Vector3(1, 1, 1);

  List<three.Object3D> visuals = [];

  final List<three.Mesh> walkableMeshes = [];
  late final NavMesh navMesh;

  final debugConfig = {
    'navMesh': false,
    'heightfield': false,
    'compactHeightfield': false,
  };
  final queryFilterConfig = <String,dynamic>{
    'filter': 'ground'
  };

  Future<void> setup() async {
    camera.position.setValues(-2, 10, 10);

    orbitControls = three.OrbitControls(camera, base.key);
    orbitControls.enableDamping = true;

    final navTestModel = await three.GLTFLoader().fromAsset('assets/models/bridges.glb');
    scene.add(navTestModel!.scene);

    scene.traverse((object){
      three.console.verbose(object.userData);
      if (object.userData['walkable'] == false) return;

      if (object is three.Mesh) {
        walkableMeshes.add(object);
      }
    });

    final pi = base.getPositionsAndIndices(walkableMeshes);
    final positions = pi.positions;
    final indices = pi.indices;

    final navMeshInput = NavMeshInput(
      positions: positions,
      indices: indices,
      waterBounds: three.BoundingBox(
        three.Vector3(-100, -1, -100),
        three.Vector3(100, 1, 100)
      ),
    );

    const cellSize = 0.2;
    const cellHeight = 0.15;

    const walkableRadiusWorld = 0.1;
    final walkableRadiusVoxels = (walkableRadiusWorld / cellSize);
    const walkableClimbWorld = 0.5;
    final walkableClimbVoxels = (walkableClimbWorld / cellHeight);
    const walkableHeightWorld = 0.1;
    final walkableHeightVoxels = (walkableHeightWorld / cellHeight);
    const walkableSlopeAngleDegrees = 45.0;

    const borderSize = 4;
    const minRegionArea = 8.0;
    const mergeRegionArea = 20.0;

    const maxSimplificationError = 1.3;
    const maxEdgeLength = 12.0;

    const maxVerticesPerPoly = 5;

    const detailSampleDistanceVoxels = 6;
    const double detailSampleDistance = detailSampleDistanceVoxels < 0.9 ? 0 : cellSize * detailSampleDistanceVoxels;

    const detailSampleMaxErrorVoxels = 1;
    const detailSampleMaxError = cellHeight * detailSampleMaxErrorVoxels;

    final navMeshConfig = NavMeshOptions(
      cellSize: cellSize,
      cellHeight: cellHeight,
      walkableRadiusWorld:walkableRadiusWorld,
      walkableRadiusVoxels:walkableRadiusVoxels,
      walkableClimbWorld:walkableClimbWorld,
      walkableClimbVoxels:walkableClimbVoxels,
      walkableHeightWorld:walkableHeightWorld,
      walkableHeightVoxels:walkableHeightVoxels,
      walkableSlopeAngleDegrees:walkableSlopeAngleDegrees,
      borderSize:borderSize,
      minRegionArea:minRegionArea,
      mergeRegionArea:mergeRegionArea,
      maxSimplificationError:maxSimplificationError,
      maxEdgeLength:maxEdgeLength,
      maxVerticesPerPoly:maxVerticesPerPoly,
      detailSampleDistance:detailSampleDistance,
      detailSampleMaxError:detailSampleMaxError,
    );

    final navMeshResult = generateNavMesh(navMeshInput, navMeshConfig);
    navMesh = navMeshResult['navMesh'];

    final navMeshHelper = debug.createNavMeshHelper(navMesh);
    navMeshHelper.position.y += 0.1;
    scene.add(navMeshHelper);

    final heightfieldHelper = debug.createHeightfieldHelper(navMeshResult['intermediates'].heightfield);
    heightfieldHelper.position.y += 0.05;
    scene.add(heightfieldHelper);

    final compactHeightfieldHelper = debug.createCompactHeightfieldSolidHelper(navMeshResult['intermediates'].compactHeightfield);
    scene.add(compactHeightfieldHelper);
    compactHeightfieldHelper.position.y += 0.1;

    final debugFolder = gui.addFolder('Debug Views');

    updateDebugViews(_){
      heightfieldHelper.visible = debugConfig['heightfield']!;
      compactHeightfieldHelper.visible = debugConfig['compactHeightfield']!;
      navMeshHelper.visible = debugConfig['navMesh']!;
    }

    updateDebugViews(null);

    debugFolder.addCheckBox(debugConfig, 'navMesh').onChange(updateDebugViews);
    debugFolder.addCheckBox(debugConfig, 'heightfield').onChange(updateDebugViews);
    debugFolder.addCheckBox(debugConfig, 'compactHeightfield').onChange(updateDebugViews);
    debugFolder.open();

    final queryFilterFolder = gui.addFolder('Query Filter');
    queryFilterFolder.addDropDown(queryFilterConfig, 'filter', ['all', 'ground', 'water'])..name = 'area'..onChange(updatePath);

    domElement.addEventListener(three.PeripheralType.pointerdown, (event){
        final point = getPointOnNavMesh(event);
        if (point == null) return;
        if (event.button == 0) {
          start = point;
        } else if (event.button == 2) {
          end = point;
        }
        updatePath(null);
    });
    updatePath(null);
  }

  Map<String,dynamic> generateNavMesh(NavMeshInput input, NavMeshOptions options){
    final ctx = BuildContextState.create();
    BuildContextState.start(ctx, 'navmesh generation');

    final positions = input.positions;
    final indices = input.indices;

    /* 1. mark walkable triangles */
    BuildContextState.start(ctx, 'mark walkable triangles');

    final triAreaIds = Uint8List(indices.length ~/ 3);
    markWalkableTriangles(positions, indices, triAreaIds, options.walkableSlopeAngleDegrees);

    BuildContextState.end(ctx, 'mark walkable triangles');

    /* 2. rasterize the triangles to a voxel heightfield */
    BuildContextState.start(ctx, 'rasterize triangles');

    final bounds = calculateMeshBounds(three.BoundingBox(), positions, indices);
    final hwhh = calculateGridSize(three.Vector2(), bounds, options.cellSize);
    final heightfieldWidth = hwhh.width;
    final heightfieldHeight = hwhh.height;

    final heightfield = createHeightfield(heightfieldWidth, heightfieldHeight, bounds, options.cellSize, options.cellHeight);

    rasterizeTriangles(ctx, heightfield, positions, indices, triAreaIds, options.walkableClimbVoxels);

    BuildContextState.end(ctx, 'rasterize triangles');

    /* 3. filter walkable surfaces */
    BuildContextState.start(ctx, 'filter walkable surfaces');

    filterLowHangingWalkableObstacles(heightfield, options.walkableClimbVoxels);
    filterLedgeSpans(heightfield, options.walkableHeightVoxels, options.walkableClimbVoxels);
    filterWalkableLowHeightSpans(heightfield, options.walkableHeightVoxels);

    BuildContextState.end(ctx, 'filter walkable surfaces');

    /* 4. compact the heightfield */
    BuildContextState.start(ctx, 'build compact heightfield');

    final compactHeightfield = buildCompactHeightfield(ctx, options.walkableHeightVoxels, options.walkableClimbVoxels, heightfield);

    BuildContextState.end(ctx, 'build compact heightfield');

    /* 5. mark custom areas */

    // mark 'water' custom area with bounds
    markBoxArea(input.waterBounds, NavMeshAreaType.water.index, compactHeightfield);

    /* 6. erode the walkable area by the agent radius / walkable radius */
    BuildContextState.start(ctx, 'erode walkable area');

    erodeWalkableArea(options.walkableRadiusVoxels, compactHeightfield);

    BuildContextState.end(ctx, 'erode walkable area');

    /* 7. prepare for region partitioning by calculating a distance field along the walkable surface */
    BuildContextState.start(ctx, 'build compact heightfield distance field');

    buildDistanceField(compactHeightfield);

    BuildContextState.end(ctx, 'build compact heightfield distance field');

    /* 8. partition the walkable surface into simple regions without holes */
    BuildContextState.start(ctx, 'build compact heightfield regions');

    buildRegions(ctx, compactHeightfield, options.borderSize, options.minRegionArea, options.mergeRegionArea);

    BuildContextState.end(ctx, 'build compact heightfield regions');

    /* 9. trace and simplify region contours */
    BuildContextState.start(ctx, 'trace and simplify region contours');

    final contourSet = buildContours(
      ctx,
      compactHeightfield,
      options.maxSimplificationError,
      options.maxEdgeLength,
      ContourBuildFlags.contourTessWallEdges,
    );

    BuildContextState.end(ctx, 'trace and simplify region contours');

    /* 10. build polygons mesh from contours */
    BuildContextState.start(ctx, 'build polygons mesh from contours');

    final polyMesh = buildPolyMesh(ctx, contourSet, options.maxVerticesPerPoly);

    for (int polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
      if (polyMesh.areas[polyIndex] == walkableArea) {
        polyMesh.areas[polyIndex] = NavMeshAreaType.ground.index;
        polyMesh.flags[polyIndex] = 0x01;
      } else if (polyMesh.areas[polyIndex] == NavMeshAreaType.water.index) {
        polyMesh.areas[polyIndex] = NavMeshAreaType.water.index;
        polyMesh.flags[polyIndex] = 0x02;
      }
    }

    BuildContextState.end(ctx, 'build polygons mesh from contours');

    /* 11. create detail mesh which allows to access approximate height on each polygon */
    BuildContextState.start(ctx, 'build detail mesh from contours');

    final polyMeshDetail = buildPolyMeshDetail(ctx, polyMesh, compactHeightfield, options.detailSampleDistance, options.detailSampleMaxError);

    BuildContextState.end(ctx, 'build detail mesh from contours');

    BuildContextState.end(ctx, 'navmesh generation');

    /* store intermediates for debugging */
    final intermediates = NavMeshIntermediates(
      buildContext: ctx,
      input: input,
      triAreaIds: triAreaIds,
      heightfield: heightfield,
      compactHeightfield: compactHeightfield,
      contourSet: contourSet,
      polyMesh: polyMesh,
      polyMeshDetail: polyMeshDetail,
    );

    /* create a single tile nav mesh */
    final nav = createNavMesh();
    nav.tileWidth = polyMesh.bounds.max[0] - polyMesh.bounds.min[0];
    nav.tileHeight = polyMesh.bounds.max[2] - polyMesh.bounds.min[2];
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
        cellSize: options.cellSize,
        cellHeight: options.cellHeight,
        walkableHeight: options.walkableHeightWorld,
        walkableRadius:options.walkableRadiusWorld,
        walkableClimb: options.walkableClimbWorld,
    );

    final tile = buildTile(tileParams);

    addTile(nav, tile);

    return {
      'navMesh': nav,
      'intermediates': intermediates,
    };
  }

  void clearVisuals() {
    for (final visual in visuals) {
      scene.remove(visual);
      visual.dispose();
    }
    visuals = [];
  }

  void addVisual(three.Object3D visual) {
    visuals.add(visual);
    scene.add(visual);
  }

  void updatePath(_) {
    clearVisuals();

    final startFlag = base.createFlag(0x2196f3);
    startFlag.position.setFrom(start);
    addVisual(startFlag);

    final endFlag = base.createFlag(0x00ff00);
    endFlag.position.setFrom(end);
    addVisual(endFlag);

    final queryFilter =
        queryFilterConfig['filter'] == 'all'
            ? defaultQueryFilter
            : queryFilterConfig['filter'] == 'ground'
              ? groundQueryFilter
              : waterQueryFilter;

    final pathResult = findPath(navMesh, start, end, halfExtents, queryFilter);

    three.console.verbose('pathResult $pathResult');
    three.console.verbose('partial? ${(pathResult.straightPathFlags & FindStraightPathResultFlags.partialPath.value) != 0}');

    final pn = pathResult;
    final path = pn.path;
    final nodePath = pn.nodePath;

    if (nodePath != null) {
      final searchNodesHelper = debug.createSearchNodesHelper(nodePath.nodes);
      addVisual(searchNodesHelper);

      for (int i = 0; i < nodePath.path.length; i++) {
        final node = nodePath.path[i];
        if (getNodeRefType(node) == NodeType.poly.value) {
          final polyHelper = debug.createNavMeshPolyHelper(navMesh, node);
          polyHelper.position.y += 0.15;
          addVisual(polyHelper);
        }
      }
    }

    if (path != null) {
      for (int i = 0; i < path.length; i++) {
        final point = path[i];
        // point
        final mesh = three.Mesh(three.SphereGeometry(0.1), three.MeshBasicMaterial.fromMap({ 'color': 0xff0000 }));
        mesh.position.setFrom(point.position);
        addVisual(mesh);

        // line
        if (i > 0) {
          final prevPoint = path[i - 1];
          final geometry = LineGeometry();
          geometry.setPositions(Float32List.fromList([...three.Vector3.copy(prevPoint.position).storage,...three.Vector3.copy(point.position).storage]));
          final material = LineMaterial.fromMap({
            'color': 0xffff00,
            'linewidth': 2.0,
          });

          final line = Line2(geometry, material);
          line.computeLineDistances();
          line.scale.setValues( 1, 1, 1 );

          addVisual(line,);
        }
      }
    }
  }

  three.Vector3? getPointOnNavMesh(event){
    final RenderBox rect = base.key.currentContext!.findRenderObject() as RenderBox;
    final size = rect.size;

    pointer.x = ((event.clientX - 0) / size.width) * 2 - 1;
    pointer.y = -((event.clientY - 0) / size.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);

    final intersects = raycaster.intersectObjects(walkableMeshes, true);
    if (intersects.isNotEmpty) {
      final p = intersects[0].point;
      return p;
    }
    return null;
  }
}

class NavMeshIntermediates{
  BuildContextState buildContext;
  NavMeshInput input;
  Uint8List triAreaIds;
  Heightfield heightfield;
  CompactHeightfield compactHeightfield;
  ContourSet contourSet;
  PolyMesh polyMesh;
  PolyMeshDetail polyMeshDetail;

  NavMeshIntermediates({
    required this.buildContext,
    required this.input,
    required this.triAreaIds,
    required this.heightfield,
    required this.compactHeightfield,
    required this.contourSet,
    required this.polyMesh,
    required this.polyMeshDetail,
  });
}

class NavMeshInput{
  List<double> positions;
  List<int> indices;
  three.BoundingBox waterBounds;

  NavMeshInput({
    required this.positions,
    required this.indices,
    required this.waterBounds
  });
}