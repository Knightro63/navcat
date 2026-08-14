import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:examples/src/common/base.dart';
import 'package:examples/src/common/debug.dart';
import 'package:examples/src/gui.dart';
import 'package:flutter/material.dart';
import 'package:navcat/math/vector.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart';
import 'package:three_js_line/three_js_line.dart';

enum NavMeshAreaType {
  none,
  ground,
  red,
  green;
}

class AreaCosts extends StatefulWidget {
  const AreaCosts({super.key});
  @override
  createState() => _State();
}

class _State extends State<AreaCosts> {
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

  final three.BoundingBox redZoneBounds = three.BoundingBox(
    three.Vector3(-8, -2, -2),
    three.Vector3(2, 2, 12)
  );

  final greenEntryBounds = three.BoundingBox(
      three.Vector3(2, -2, -12),
      three.Vector3(14, 2, -2)
  );

  final greenLaneBounds = three.BoundingBox(
      three.Vector3(8, -2, -2),
      three.Vector3(14, 2, 12)
  );

  final greenExitBounds = three.BoundingBox(
      three.Vector3(-12, -2, 12),
      three.Vector3(14, 2, 20,)
  );

  final redColor = 0xff3b30;
  final greenColor = 0x00ff6a;
  final floorSize = 40.0;

  late final three.OrbitControls orbitControls;
  final raycaster = three.Raycaster();
  final pointer = three.Vector2();
  late final three.Mesh floorMesh;
  late final NavMesh navMesh;
  late final QueryFilter queryFilter;
  String? moving;//: 'start' | 'end' | null

  three.Vector3 start = three.Vector3(-6, 0, -10);
  three.Vector3 end = three.Vector3(-6, 0, 18);
  final three.Vector3 halfExtents = three.Vector3(0.6, 1, 0.6);

  List<three.Object3D> visuals = [];

  final NavMeshOptions navMeshOptions = NavMeshOptions();

  final areaCostConfig = {
    'ground': 1.0,
    'red': 3.0,
    'green': 0.45,
  };

  Future<void> setup() async {
    camera.position.setValues(-18, 16, 22);
    camera.lookAt(three.Vector3());

    orbitControls = three.OrbitControls(camera, base.key);
    orbitControls.enableDamping = true;

    /* base floor */
    final floorGeometry = three.PlaneGeometry(floorSize, floorSize);
    floorGeometry.rotateX(-math.pi / 2);
    final floorMaterial = three.MeshBasicMaterial.fromMap({ 'color': 0x1d4ed8 });
    floorMesh = three.Mesh(floorGeometry, floorMaterial);
    floorMesh.receiveShadow = true;
    scene.add(floorMesh);

    final redOverlay = createOverlay(redZoneBounds, redColor);
    final greenEntryOverlay = createOverlay(greenEntryBounds, greenColor);
    final greenLaneOverlay = createOverlay(greenLaneBounds, greenColor);
    final greenExitOverlay = createOverlay(greenExitBounds, greenColor);

    scene.addAll([redOverlay, greenEntryOverlay, greenLaneOverlay, greenExitOverlay]);

    final pi = base.getPositionsAndIndices([floorMesh]);
    final positions = pi.positions;
    final indices = pi.indices;
    navMesh = generateFlatNavMesh(NavMeshGenerationInput(positions: positions, indices: indices), navMeshOptions);

    final navMeshHelper = debug.createNavMeshHelper(navMesh);
    navMeshHelper.visible = false;
    scene.add(navMeshHelper);

    /* cost-aware query filter */

    queryFilter = QueryFilter(
      passFilter: (_, _) => true,
      getCost: (pa, pb, navMeshInstance, _, curRef, _) {
        final base = pa.distanceTo(pb);
        final node = getNodeByRef(navMeshInstance, curRef);
        final multiplier =
            node?.area == NavMeshAreaType.red.index
                ? areaCostConfig['red']
                : node?.area == NavMeshAreaType.red.index
                  ? areaCostConfig['green']
                  : areaCostConfig['ground'];
        return base * multiplier!;
      },
    );

    domElement.addEventListener(three.PeripheralType.pointerdown, (three.WebPointerEvent event){
      //event.preventDefault();
      final point = getPointOnFloor(event);
      if (point == null) return;

      if (event.button == 0) {
        if (moving == 'start') {
          moving = null;
          //domElement.style.cursor = '';
          start = point;
        } else {
          moving = 'start';
          //domElement.style.cursor = 'crosshair';
          start = point;
        }
      } else if (event.button == 2) {
        if (moving == 'end') {
          moving = null;
          //domElement.style.cursor = '';
          end = point;
        } else {
          moving = 'end';
          //domElement.style.cursor = 'crosshair';
          end = point;
        }
      }

      updatePath();
    });

    domElement.addEventListener(three.PeripheralType.pointerHover, (event){
      if (moving == null) return;
      final point = getPointOnFloor(event);
      if (point == null) return;

      if (moving == 'start') {
        start = point;
      } else if (moving == 'end') {
        end = point;
      }
      updatePath();
    });

    final costsFolder = gui.addFolder('Cost multipliers');
    final sliderMin = 0.1;
    final sliderMax = 5;
    final sliderStep = 0.05;

    costsFolder.addSlider(areaCostConfig, 'ground', sliderMin, sliderMax, sliderStep)..name = 'Ground'..onChange((_) => updatePath());
    costsFolder.addSlider(areaCostConfig, 'red', sliderMin, sliderMax, sliderStep)..name = 'Red'..onChange((_) => updatePath());
    costsFolder.addSlider(areaCostConfig, 'green', sliderMin, sliderMax, sliderStep)..name = 'Green'..onChange((_) => updatePath());
    costsFolder.open();

    updatePath();
  }

  three.Mesh createOverlay(three.BoundingBox bounds, int color) {
    final sizeX = bounds.max[0] - bounds.min[0];
    final sizeZ = bounds.max[2] - bounds.min[2];
    final geometry = three.PlaneGeometry(sizeX, sizeZ);
    geometry.rotateX(-math.pi / 2);
    final material = three.MeshBasicMaterial.fromMap({
      'color' : color,
      'transparent': false,
      'side': three.FrontSide,
      'depthWrite': false,
    });
    final mesh = three.Mesh(geometry, material);
    mesh.position.setValues(bounds.min[0] + sizeX / 2, 0.01, bounds.min[2] + sizeZ / 2);
    mesh.renderOrder = 1;
    return mesh;
  }

  NavMesh generateFlatNavMesh(NavMeshGenerationInput input, NavMeshOptions options){
    final ctx = BuildContextState.create();
    BuildContextState.start(ctx, 'navmesh generation');

    final walkableClimbVoxels = (options.walkableClimbWorld / options.cellHeight).ceil() * 1.0;
    final walkableHeightVoxels = (options.walkableHeightWorld / options.cellHeight).ceil() * 1.0;
    final walkableRadiusVoxels = (options.walkableRadiusWorld / options.cellSize).ceil() * 1.0;

    final triAreaIds = Uint8List.fromList(List.filled(input.indices.length ~/ 3,0));
    markWalkableTriangles(input.positions, input.indices, triAreaIds, options.walkableSlopeAngleDegrees);

    final bounds = calculateMeshBounds(three.BoundingBox(), input.positions, input.indices);
    final three.Vector2 gridSize = calculateGridSize(three.Vector2(), bounds, options.cellSize);

    final heightfield = createHeightfield(gridSize[0], gridSize[1], bounds, options.cellSize, options.cellHeight);

    rasterizeTriangles(ctx, heightfield, input.positions, input.indices, triAreaIds, walkableClimbVoxels);

    filterLowHangingWalkableObstacles(heightfield, walkableClimbVoxels);
    filterLedgeSpans(heightfield, walkableHeightVoxels, walkableClimbVoxels);
    filterWalkableLowHeightSpans(heightfield, walkableHeightVoxels);

    final compactHeightfield = buildCompactHeightfield(ctx, walkableHeightVoxels, walkableClimbVoxels, heightfield);

    markBoxArea(redZoneBounds, NavMeshAreaType.red.index, compactHeightfield);
    markBoxArea(greenEntryBounds, NavMeshAreaType.green.index, compactHeightfield);
    markBoxArea(greenLaneBounds, NavMeshAreaType.green.index, compactHeightfield);
    markBoxArea(greenExitBounds, NavMeshAreaType.green.index, compactHeightfield);

    erodeWalkableArea(walkableRadiusVoxels, compactHeightfield);

    buildDistanceField(compactHeightfield);
    buildRegions(ctx, compactHeightfield, options.borderSize, options.minRegionArea, options.mergeRegionArea);

    final contourSet = buildContours(
      ctx,
      compactHeightfield,
      options.maxSimplificationError,
      options.maxEdgeLength,
      ContourBuildFlags.contourTessWallEdges,
    );

    final polyMesh = buildPolyMesh(ctx, contourSet, options.maxVerticesPerPoly);

    for (int polyIndex = 0; polyIndex < polyMesh.nPolys; polyIndex++) {
      if (polyMesh.areas[polyIndex] == walkableArea) {
        polyMesh.areas[polyIndex] = NavMeshAreaType.ground.index;
        polyMesh.flags[polyIndex] = 0x01;
      } else if (polyMesh.areas[polyIndex] == NavMeshAreaType.red.index) {
        polyMesh.flags[polyIndex] = 0x02;
      } else if (polyMesh.areas[polyIndex] == NavMeshAreaType.green.index) {
        polyMesh.flags[polyIndex] = 0x04;
      }
    }

    final polyMeshDetail = buildPolyMeshDetail(
      ctx,
      polyMesh,
      compactHeightfield,
      options.detailSampleDistance,
      options.detailSampleMaxError,
    );

    final navMesh = createNavMesh();
    navMesh.tileWidth = polyMesh.bounds.max[0] - polyMesh.bounds.min[0];
    navMesh.tileHeight = polyMesh.bounds.max[2] - polyMesh.bounds.min[2];
    box3.min.min2(navMesh.origin, polyMesh.bounds.min);

    final tilePolys = polyMeshToTilePolys(polyMesh);
    final tileDetailMesh = polyMeshDetailToTileDetailMesh(tilePolys.polys, polyMeshDetail);

    final NavMeshTileParams tileParams = NavMeshTileParams(
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
      walkableRadius: options.walkableRadiusWorld,
      walkableClimb: options.walkableClimbWorld,
    );

    final tile = buildTile(tileParams);
    addTile(navMesh, tile);

    BuildContextState.end(ctx, 'navmesh generation');

    return navMesh;
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

  three.Object3D createPathPoint(three.Vector3 position) {
    final geometry = three.SphereGeometry(0.2);
    final material = three.MeshBasicMaterial.fromMap({ 'color': 0xffffff });
    final mesh = three.Mesh(geometry, material);
    mesh.position.setFrom(position);
    return mesh;
  }

  three.Object3D createPathSegment(three.Vector3 a, three.Vector3 b) {
    final geometry = LineGeometry();
    geometry.setPositions(Float32List.fromList([...three.Vector3.copy(a).storage,...three.Vector3.copy(b).storage]));
    final material = LineMaterial.fromMap({
      'color': 0xffffff,
      'linewidth': 2.0,
    });

    final line = Line2(geometry, material);
    line.computeLineDistances();
    line.scale.setValues( 1, 1, 1 );

    return line;
}

  void updatePath() {
    clearVisuals();

    final startFlag = base.createFlag(0x2196f3);
    startFlag.position.setFrom(start);
    addVisual(startFlag);

    final endFlag = base.createFlag(greenColor);
    endFlag.position.setFrom(end);
    addVisual(endFlag);

    final pathResult = findPath(navMesh, start, end, halfExtents, queryFilter);

    final path = pathResult.path;

    if (path != null) {
      for (int i = 0; i < path.length; i++) {
        final point = path[i];
        addVisual(createPathPoint(point.position));

        if (i > 0) {
          final prev = path[i - 1];
          addVisual(createPathSegment(prev.position, point.position));
        }
      }
    }
  }

  three.Vector3? getPointOnFloor(three.WebPointerEvent event){
    final RenderBox rect = base.key.currentContext!.findRenderObject() as RenderBox;
    final size = rect.size;

    pointer.x = ((event.clientX - 0) / size.width) * 2 - 1;
    pointer.y = -((event.clientY - 0) / size.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);

    final intersects = raycaster.intersectObjects([floorMesh], true);
    if (intersects.isNotEmpty) {
      final p = intersects[0].point;
      return p;
    }
    return null;
  }
}