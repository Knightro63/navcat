import 'dart:typed_data';
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart' as navcat;

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(
        colorScheme: .fromSeed(seedColor: Colors.deepPurple),
      ),
      home: const SoloNavmesh(),
    );
  }
}

class SoloNavmesh extends StatefulWidget {
  const SoloNavmesh({super.key});
  @override
  createState() => _State();
}

class _State extends State<SoloNavmesh> {
  late final three.ThreeJS threeJs;
  final _position = three.Vector3();

  @override
  void initState() {
    threeJs = three.ThreeJS(
      settings: three.Settings(
        toneMapping: three.ACESFilmicToneMapping,
      ),
      onSetupComplete: (){setState(() {});},
      setup: setup,
    );
    super.initState();
  }
  @override
  void dispose() {
    threeJs.dispose();
    three.loading.clear();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: threeJs.build(),
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

  navcat.SoloNavMeshResult? result;

  Future<void> setup() async {
    threeJs.scene = three.Scene();
    threeJs.scene.background = three.Color.fromHex32(0x202020);

    threeJs.camera = three.PerspectiveCamera(
      75,
      threeJs.width / threeJs.height,
      0.1,
      1000,
    );
    threeJs.camera.position.setValues(0, 0, 5);

    // lighting
    final ambientLight = three.AmbientLight(0xffffff, 0.5);
    threeJs.scene.add(ambientLight);

    final directionalLight = three.DirectionalLight(0xffffff, 1);
    directionalLight.position.setValues(5, 5, 5);
    threeJs.scene.add(directionalLight);

    threeJs.camera.position.setValues(-2, 10, 10);

    final orbitControls = three.OrbitControls(threeJs.camera, threeJs.globalKey);
    orbitControls.enableDamping = true;

    final navTestModel = await three.GLTFLoader().fromAsset('assets/models/nav-test.glb');
    threeJs.scene.add(navTestModel!.scene);

    generate();
  }

  void generate() {
    /* generate navmesh */
    final List<three.Mesh> walkableMeshes = [];
    threeJs.scene.traverse((object){
      if (object is three.Mesh) {
        walkableMeshes.add(object);
      }
    });

    final nmi = getPositionsAndIndices(walkableMeshes);

    final navcat.MeshInput navMeshInput = navcat.MeshInput(
      positions: nmi.positions,
      indices: nmi.indices,
    );

    final walkableRadiusVoxels = (config['walkableRadiusWorld'] / config['cellSize']).ceil() *1.0;
    final walkableClimbVoxels = (config['walkableClimbWorld'] / config['cellHeight']).ceil() *1.0;
    final walkableHeightVoxels = (config['walkableHeightWorld'] / config['cellHeight']).ceil() *1.0;

    final detailSampleDistance = config['detailSampleDistance'] < 0.9 ? 0 : config['cellSize'] * config['detailSampleDistance'];
    final detailSampleMaxError = config['cellHeight'] * config['detailSampleMaxError'];

    final navcat.SoloNavMeshOptions navMeshConfig  = navcat.SoloNavMeshOptions(
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

    result = navcat.generateSoloNavMesh(navMeshInput, navMeshConfig);

    three.console.verbose(result);
  }

  navcat.NavMeshGenerationInput getPositionsAndIndices(List<three.Mesh> meshes){
    final List<navcat.NavMeshGenerationInput> toMerge = [];

    for (final mesh in meshes) {
      final positionAttribute = mesh.geometry?.attributes['position'] as three.Float32BufferAttribute?;

      if (positionAttribute == null || positionAttribute.itemSize != 3) {
        continue;
      }

      mesh.updateMatrixWorld();

      final positions = Float32List(positionAttribute.count * 3);

      for (int i = 0; i < positionAttribute.count; i++) {
        final pos = _position.fromBuffer(positionAttribute, i);
        mesh.localToWorld(pos);
        final indx = i * 3;
        positions[indx] = pos.x;
        positions[indx + 1] = pos.y;
        positions[indx + 2] = pos.z;
      }
      final index = mesh.geometry?.getIndex()?.array;
      List<int>? indices =  index != null? List.from(index.buffer.asUint16List()): null;
      if (indices == null) {
        // this will become indexed when merging with other meshes
        final List<int> ascendingIndex = [];
        for (int i = 0; i < positionAttribute.count; i++) {
          ascendingIndex.add(i);
        }
        indices = ascendingIndex;
      }

      toMerge.add(navcat.NavMeshGenerationInput(positions: positions, indices: indices));
    }

    return navcat.mergePositionsAndIndices(toMerge);
  }
}