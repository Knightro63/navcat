import 'dart:async';
import 'package:examples/src/common/base.dart';
import 'package:examples/src/common/debug.dart';
import 'package:examples/src/gui.dart';
import 'package:flutter/material.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart';

class SoloNavmesh extends StatefulWidget {
  const SoloNavmesh({super.key});
  @override
  createState() => _State();
}

class _State extends State<SoloNavmesh> {
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

  final Map<String,dynamic> config = {
    'cellSize': 0.15,
    'cellHeight': 0.15,
    'walkableRadiusWorld': 0.1,
    'walkableClimbWorld': 0.5,
    'walkableHeightWorld': 0.25,
    'walkableSlopeAngleDegrees': 45.0,
    'borderSize': 0,
    'minRegionArea': 8.0,
    'mergeRegionArea': 20.0,
    'maxSimplificationError': 1.3,
    'maxEdgeLength': 12.0,
    'maxVerticesPerPoly': 5,
    'detailSampleDistance': 6.0,
    'detailSampleMaxError': 1,
  };

  /* debug helpers configuration */
  final Map<String,dynamic> debugConfig = {
    'showMesh': true,
    'showTriangleAreaIds': false,
    'showHeightfield': false,
    'showCompactHeightfieldSolid': false,
    'showCompactHeightfieldDistances': false,
    'showCompactHeightfieldRegions': false,
    'showRawContours': false,
    'showSimplifiedContours': false,
    'showPolyMesh': false,
    'showPolyMeshDetail': false,
    'showNavMeshBvTree': false,
    'showNavMesh': true,
    'showNavMeshLinks': false,
  };

  SoloNavMeshResult? result;
  DebugHelpers debugHelpers = DebugHelpers();

  Future<void> setup() async {
    camera.position.setValues(-2, 10, 10);

    final orbitControls = three.OrbitControls(camera, base.key);
    orbitControls.enableDamping = true;

    final navTestModel = await three.GLTFLoader().fromAsset('assets/models/nav-test.glb');
    scene.add(navTestModel!.scene);

    final cellFolder = gui.addFolder('Heightfield');
    cellFolder.addSlider(config, 'cellSize', 0.01, 1, 0.01);
    cellFolder.addSlider(config, 'cellHeight', 0.01, 1, 0.01);

    final walkableFolder = gui.addFolder('Agent');
    walkableFolder.addSlider(config, 'walkableRadiusWorld', 0, 2, 0.01);
    walkableFolder.addSlider(config, 'walkableClimbWorld', 0, 2, 0.01);
    walkableFolder.addSlider(config, 'walkableHeightWorld', 0, 2, 0.01);
    walkableFolder.addSlider(config, 'walkableSlopeAngleDegrees', 0, 90, 1);

    final regionFolder = gui.addFolder('Region');
    regionFolder.addSlider(config, 'borderSize', 0, 10, 1);
    regionFolder.addSlider(config, 'minRegionArea', 0, 50, 1);
    regionFolder.addSlider(config, 'mergeRegionArea', 0, 50, 1);

    final contourFolder = gui.addFolder('Contour');
    contourFolder.addSlider(config, 'maxSimplificationError', 0.1, 10, 0.1);
    contourFolder.addSlider(config, 'maxEdgeLength', 0, 50, 1);

    final polyMeshFolder = gui.addFolder('PolyMesh');
    polyMeshFolder.addSlider(config, 'maxVerticesPerPoly', 3, 12, 1);

    final detailFolder = gui.addFolder('Detail');
    detailFolder.addSlider(config, 'detailSampleDistance', 0, 16, 0.1);
    detailFolder.addSlider(config, 'detailSampleMaxError', 0, 16, 0.1);

    final debugFolder = gui.addFolder('Debug Helpers');
    debugFolder
        .addButton(debugConfig, 'showMesh')
        ..name = 'Show Mesh'
        ..onChange((_) => {
            navTestModel.scene.visible = debugConfig['showMesh']
        });
    debugFolder.addButton(debugConfig, 'showTriangleAreaIds')..name = 'Triangle Area IDs'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showHeightfield')..name = 'Heightfield'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showCompactHeightfieldSolid')..name = 'Compact Heightfield Solid'..onChange(updateDebugHelpers);
    debugFolder
        .addButton(debugConfig, 'showCompactHeightfieldDistances')
        ..name = 'ompact Heightfield Distances'
        ..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showCompactHeightfieldRegions')..name = 'Compact Heightfield Regions'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showRawContours')..name = 'Raw Contours'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showSimplifiedContours')..name = 'Simplified Contours'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showPolyMesh')..name = 'Poly Mesh'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showPolyMeshDetail')..name = 'Poly Mesh Detail'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showNavMeshBvTree')..name = 'NavMesh BV Tree'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showNavMesh')..name = 'NavMesh'..onChange(updateDebugHelpers);
    debugFolder.addButton(debugConfig, 'showNavMeshLinks')..name = 'NavMesh Links'..onChange(updateDebugHelpers);
  
    generate();
  }

  void clearDebugHelpers() {
    debugHelpers.forEach((helper){
      if (helper != null) {
        scene.remove(helper);
        helper.dispose();
      }
    });

    // Reset all references
    debugHelpers.triangleAreaIds = null;
    debugHelpers.heightfield = null;
    debugHelpers.compactHeightfieldSolid = null;
    debugHelpers.compactHeightfieldDistances = null;
    debugHelpers.compactHeightfieldRegions = null;
    debugHelpers.rawContours.clear();
    debugHelpers.simplifiedContours.clear();
    debugHelpers.polyMesh.clear();
    debugHelpers.polyMeshDetail.clear();
    debugHelpers.navMeshBvTree.clear();
    debugHelpers.navMesh.clear();
    debugHelpers.navMeshLinks.clear();
  }

  void updateDebugHelpers(value) {
    if (result == null) return;

    final intermediates = result!.intermediates;
    final navMesh = result!.navMesh;

    // Clear existing helpers
    clearDebugHelpers();

    // Create debug helpers based on current config
    if (debugConfig['showTriangleAreaIds']) {
      debugHelpers.triangleAreaIds = createTriangleAreaIdsHelper(intermediates.input, intermediates.triAreaIds);
      final o = debug.primitiveToThreeJS(debugHelpers.triangleAreaIds!);
      scene.add(o);
    }

    if (debugConfig['showHeightfield']) {
        debugHelpers.heightfield = createHeightfieldHelper(intermediates.heightfield);
      final o = debug.primitiveToThreeJS(debugHelpers.heightfield!);
      scene.add(o);
    }

    if (debugConfig['showCompactHeightfieldSolid']) {
        debugHelpers.compactHeightfieldSolid = createCompactHeightfieldSolidHelper(intermediates.compactHeightfield);
      final o = debug.primitiveToThreeJS(debugHelpers.compactHeightfieldSolid!);
      scene.add(o);
    }

    if (debugConfig['showCompactHeightfieldDistances']) {
        debugHelpers.compactHeightfieldDistances = createCompactHeightfieldDistancesHelper(intermediates.compactHeightfield);
      final o = debug.primitiveToThreeJS(debugHelpers.compactHeightfieldDistances!);
      scene.add(o);
    }

    if (debugConfig['showCompactHeightfieldRegions']) {
        debugHelpers.compactHeightfieldRegions = createCompactHeightfieldRegionsHelper(intermediates.compactHeightfield);
      final o = debug.primitiveToThreeJS(debugHelpers.compactHeightfieldRegions!);
      scene.add(o);
    }

    if (debugConfig['showRawContours']) {
        debugHelpers.rawContours = createRawContoursHelper(intermediates.contourSet);
      final o = debug.primitivesToThreeJS(debugHelpers.rawContours);
      scene.add(o);
    }

    if (debugConfig['showSimplifiedContours']) {
        debugHelpers.simplifiedContours = createSimplifiedContoursHelper(intermediates.contourSet);
      final o = debug.primitivesToThreeJS(debugHelpers.simplifiedContours);
      scene.add(o);
    }

    if (debugConfig['showPolyMesh']) {
      debugHelpers.polyMesh = createPolyMeshHelper(intermediates.polyMesh);
      final o = debug.primitivesToThreeJS(debugHelpers.polyMesh);
      scene.add(o);    
    }

    if (debugConfig['showPolyMeshDetail']) {
      debugHelpers.polyMeshDetail = createPolyMeshDetailHelper(intermediates.polyMeshDetail);
      final o = debug.primitivesToThreeJS(debugHelpers.polyMeshDetail);
      scene.add(o);    
    }

    if (debugConfig['showNavMeshBvTree']) {
      debugHelpers.navMeshBvTree = createNavMeshBvTreeHelper(navMesh);
      final o = debug.primitivesToThreeJS(debugHelpers.navMeshBvTree);
      scene.add(o);
    }

    if (debugConfig['showNavMesh']) {
        debugHelpers.navMesh = createNavMeshHelper(navMesh);
      final o = debug.primitivesToThreeJS(debugHelpers.navMesh);
      o.position.y += 0.1;
      scene.add(o);
    }

    if (debugConfig['showNavMeshLinks']) {
      debugHelpers.navMeshLinks = createNavMeshLinksHelper(navMesh);
      final o = debug.primitivesToThreeJS(debugHelpers.navMeshLinks);
      scene.add(o);
    }
  }


  void generate() {
    /* clear helpers */
    clearDebugHelpers();

    /* generate navmesh */
    final List<three.Mesh> walkableMeshes = [];
    scene.traverse((object){
      if (object is three.Mesh) {
        walkableMeshes.add(object);
      }
    });

    final nmi = base.getPositionsAndIndices(walkableMeshes);

    final MeshInput navMeshInput = MeshInput(
      positions: nmi.positions,
      indices: nmi.indices,
    );

    final walkableRadiusVoxels = (config['walkableRadiusWorld'] / config['cellSize']).ceil() *1.0;
    final walkableClimbVoxels = (config['walkableClimbWorld'] / config['cellHeight']).ceil() *1.0;
    final walkableHeightVoxels = (config['walkableHeightWorld'] / config['cellHeight']).ceil() *1.0;

    final detailSampleDistance = config['detailSampleDistance'] < 0.9 ? 0 : config['cellSize'] * config['detailSampleDistance'];
    final detailSampleMaxError = config['cellHeight'] * config['detailSampleMaxError'];

    final SoloNavMeshOptions navMeshConfig  = SoloNavMeshOptions(
      cellSize: config['cellSize'],
      cellHeight: config['cellHeight'],
      walkableRadiusWorld: config['walkableRadiusWorld'],
      walkableRadiusVoxels: walkableRadiusVoxels,
      walkableClimbWorld: config['walkableClimbWorld'],
      walkableClimbVoxels:walkableClimbVoxels,
      walkableHeightWorld: config['walkableHeightWorld'],
      walkableHeightVoxels:walkableHeightVoxels,
      walkableSlopeAngleDegrees: config['walkableSlopeAngleDegrees'],
      borderSize: config['borderSize'],
      minRegionArea: config['minRegionArea'],
      mergeRegionArea: config['mergeRegionArea'],
      maxSimplificationError: config['maxSimplificationError'],
      maxEdgeLength: config['maxEdgeLength'],
      maxVerticesPerPoly: config['maxVerticesPerPoly'],
      detailSampleDistance: detailSampleDistance,
      detailSampleMaxError: detailSampleMaxError,
    );

    result = generateSoloNavMesh(navMeshInput, navMeshConfig);

    three.console.verbose(result);

    /* update helpers */
    updateDebugHelpers(null);
  }
}

class DebugHelpers{
  DebugPrimitive? triangleAreaIds;
  DebugPrimitive? heightfield;
  DebugPrimitive? compactHeightfieldSolid;
  DebugPrimitive? compactHeightfieldDistances;
  DebugPrimitive? compactHeightfieldRegions;
  late List<DebugPrimitive> rawContours;
  late List<DebugPrimitive> simplifiedContours;
  late List<DebugPrimitive> polyMesh;
  late List<DebugPrimitive> polyMeshDetail;
  late List<DebugPrimitive> navMeshBvTree;
  late List<DebugPrimitive> navMesh;
  late List<DebugPrimitive> navMeshLinks;

  late List<DebugPrimitive?> helpers = [
    triangleAreaIds,
    heightfield,
    compactHeightfieldSolid,
    compactHeightfieldDistances,
    compactHeightfieldRegions,
    ...rawContours,
    ...simplifiedContours,
    ...polyMesh,
    ...polyMeshDetail,
    ...navMeshBvTree,
    ...navMesh,
    ...navMeshLinks,
  ];

  DebugHelpers({
    this.triangleAreaIds,
    this.heightfield,
    this.compactHeightfieldSolid,
    this.compactHeightfieldDistances,
    this.compactHeightfieldRegions,
    List<DebugPrimitive>? rawContours,
    List<DebugPrimitive>? simplifiedContours,
    List<DebugPrimitive>? polyMesh,
    List<DebugPrimitive>? polyMeshDetail,
    List<DebugPrimitive>? navMeshBvTree,
    List<DebugPrimitive>? navMesh,
    List<DebugPrimitive>? navMeshLinks,
  }){
    this.rawContours = rawContours ?? [];
    this.simplifiedContours = simplifiedContours ?? [];
    this.polyMesh = polyMesh ?? [];
    this.polyMeshDetail = polyMeshDetail ?? [];
    this.navMeshBvTree = navMeshBvTree ?? [];
    this.navMesh = navMesh ?? [];
    this.navMeshLinks = navMeshLinks ?? [];
  }

  void Function(void Function(dynamic)) get forEach => helpers.forEach;
}