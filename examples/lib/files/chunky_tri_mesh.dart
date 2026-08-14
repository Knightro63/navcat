import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:examples/src/common/base.dart';
import 'package:examples/src/common/debug.dart';
import 'package:examples/src/gui.dart';
import 'package:flutter/material.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart';

enum NavMeshAreaType {none,ground,water}

class ChunkyTriMeshEx extends StatefulWidget {
  const ChunkyTriMeshEx({super.key});
  @override
  createState() => _State();
}

class _State extends State<ChunkyTriMeshEx> {
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

  final trianglesGroup = three.Group();
  final chunkBoundsGroup = three.Group();
  final queryRegionGroup = three.Group();

  final List<three.Mesh> walkableMeshes = [];
  late final NavMesh navMesh;
  late final ChunkyTriMesh levelChunkyTriMesh;

  late final List<double> positions;
  late final List<int> indices;
  bool isDragging = false;

  final config = <String,dynamic>{
    'showTriangles': true,
    'showChunkBounds': true,
    'showQueryRegion': true,
    'queryRegionX': 0.0,
    'queryRegionZ': 0.0,
    'queryRegionSize': 5,
    'triangleInQueryColor': 0x00ff00,
    'triangleOutQueryColor': 0x666666,
    'chunkBoundsColor': 0x0088ff,
    'queryRegionColor': 0xffff00,
    'wireframe': true,
  };

  Future<void> setup() async {
    camera.position.setValues(0, 100, -50);
    camera.zoom = 0.1;

    orbitControls = three.OrbitControls(camera, base.key);
    orbitControls.enableDamping = true;
    orbitControls.target.setValues(0, 0, -50);

    final levelModel = await three.GLTFLoader().fromAsset('assets/models/dungeon.gltf');
    scene.add(levelModel!.scene);

    scene.traverse((object){
      three.console.verbose(object.userData);
      if (object.userData['walkable'] == false) return;

      if (object is three.Mesh) {
        walkableMeshes.add(object);
      }
    });

    final pi = base.getPositionsAndIndices(walkableMeshes);
    positions = pi.positions;
    indices = pi.indices;

    levelChunkyTriMesh = ChunkyTriMesh.create(positions, indices, 256);

    scene.add(trianglesGroup);
    scene.add(chunkBoundsGroup);
    scene.add(queryRegionGroup);

    domElement.addEventListener(three.PeripheralType.pointerdown, (event){
      if (event.button != 0) return; // Only left click
      isDragging = true;
      orbitControls.enabled = false;

      final point = getPointOnGround(event);
      if (point != null) {
        config['queryRegionX'] = point.x;
        config['queryRegionZ'] = point.z;
        updateVisualization(null);

        // Update GUI
        // gui.controllersRecursive().forEach((controller) => {
        //   controller.updateDisplay();
        // });
      }
    });

    domElement.addEventListener(three.PeripheralType.pointermove, (event){
      if (!isDragging) return;

      final point = getPointOnGround(event);
      if (point != null) {
        config['queryRegionX'] = point.x;
        config['queryRegionZ'] = point.z;
        updateVisualization(null);

        // Update GUI
        // gui.controllersRecursive().forEach((controller) => {
        //   controller.updateDisplay();
        // });
      }
    });

    domElement.addEventListener(three.PeripheralType.pointerup, (event){
      isDragging = false;
      orbitControls.enabled = true;
    });

    final trianglesFolder = gui.addFolder('Triangles');
    trianglesFolder.addCheckBox(config, 'showTriangles')..name = 'Show Triangles'..onChange(updateVisualization);
    trianglesFolder.addCheckBox(config, 'wireframe')..name = 'Wireframe'..onChange(updateVisualization);
    trianglesFolder.addColor(config, 'triangleInQueryColor')..name = 'In Query Color'..onChange(updateVisualization);
    trianglesFolder.addColor(config, 'triangleOutQueryColor')..name = 'Outside Query Color'..onChange(updateVisualization);
    trianglesFolder.open();

    final chunksFolder = gui.addFolder('Chunk Bounds');
    chunksFolder.addCheckBox(config, 'showChunkBounds')..name = 'Show Bounds'..onChange(updateVisualization);
    chunksFolder.addColor(config, 'chunkBoundsColor')..name = 'Bounds Color'..onChange(updateVisualization);
    chunksFolder.open();

    final queryFolder = gui.addFolder('Query Region');
    queryFolder.addCheckBox(config, 'showQueryRegion')..name = 'Show Query'..onChange(updateVisualization);
    // queryFolder.addSlider(config, 'queryRegionX', -10, 10, 0.1)..name = 'Query X'..onChange(updateVisualization);
    // queryFolder.addSlider(config, 'queryRegionZ', -10, 10, 0.1)..name = 'Query Z'..onChange(updateVisualization);
    // queryFolder.addSlider(config, 'queryRegionSize', 0.5, 15, 0.1)..name = 'Query Size'..onChange(updateVisualization);
    queryFolder.addColor(config, 'queryRegionColor')..name = 'Region Color'..onChange(updateVisualization);
    queryFolder.open();

    updateVisualization(null);
  }

  three.Line createChunkEdges(double minX, double minZ, double maxX, double maxZ, int color) {
    final points = [
      three.Vector3(minX, 0.05, minZ),
      three.Vector3(maxX, 0.05, minZ),
      three.Vector3(maxX, 0.05, maxZ),
      three.Vector3(minX, 0.05, maxZ),
      three.Vector3(minX, 0.05, minZ),
    ];

    final geometry = three.BufferGeometry().setFromPoints(points);
    final material = three.LineBasicMaterial.fromMap({ 'color': color, 'linewidth': 1 });
    final line = three.Line(geometry, material);

    return line;
  }

  three.BufferGeometry createTriangleGeometry(List<int> triangleIndices, List<double> positionsArray) {
    final vertexCount = triangleIndices.length;
    final positionAttr = Float32List(vertexCount * 3);

    for (int i = 0; i < triangleIndices.length; i++) {
      final vertexIndex = triangleIndices[i];
      positionAttr[i * 3 + 0] = positionsArray[vertexIndex * 3 + 0];
      positionAttr[i * 3 + 1] = positionsArray[vertexIndex * 3 + 1];
      positionAttr[i * 3 + 2] = positionsArray[vertexIndex * 3 + 2];
    }

    final geometry = three.BufferGeometry();
    geometry.setAttributeFromString('position', three.Float32BufferAttribute(positionAttr, 3));
    geometry.computeVertexNormals();

    return geometry;
  }

  /* visualize triangles colored by query overlap */
  void visualizeTriangles() {
    trianglesGroup.clear();

    if (!config['showTriangles']) return;

    final halfSize = config['queryRegionSize'] / 2;
    final queryMin = three.Vector2(config['queryRegionX'] - halfSize, config['queryRegionZ'] - halfSize);
    final queryMax = three.Vector2(config['queryRegionX'] + halfSize, config['queryRegionZ'] + halfSize);

    // Get triangles in query region
    final trianglesInQuery = ChunkyTriMesh.getTrianglesInRect(levelChunkyTriMesh, queryMin, queryMax);

    // Group triangles by whether they're in the query
    final List<int> inQueryIndices = [];
    final List<int> outQueryIndices = [];

    for (int i = 0; i < indices.length; i += 3) {
      final i0 = indices[i];
      final i1 = (i+1>=indices.length)?0:indices[i + 1];
      final i2 = (i+2>=indices.length)?0:indices[i + 2];

      // Check if this triangle is in the query result
      // We need to check if all three vertices match
      bool isInQuery = false;
      for (int j = 0; j < trianglesInQuery.length; j += 3) {
        if (
          trianglesInQuery[j] == i0 &&
          trianglesInQuery[j + 1] == i1 &&
          trianglesInQuery[j + 2] == i2
        ) {
          isInQuery = true;
          break;
        }
      }

      if (isInQuery) {
        inQueryIndices.addAll([i0, i1, i2]);
      } else {
        outQueryIndices.addAll([i0, i1, i2]);
      }
    }

    three.console.verbose('Triangles in query: ${inQueryIndices.length / 3}/${indices.length / 3}');

    // Create mesh for triangles in query (green)
    if (inQueryIndices.isNotEmpty) {
      final geometry = createTriangleGeometry(inQueryIndices, positions);
      final material = three.MeshBasicMaterial.fromMap({
          'color': config['triangleInQueryColor'],
          'wireframe': config['wireframe'],
          'side': three.DoubleSide,
      });
      final mesh = three.Mesh(geometry, material);
      trianglesGroup.add(mesh);
    }

    // Create mesh for triangles outside query (gray)
    if (outQueryIndices.isNotEmpty) {
      final geometry = createTriangleGeometry(outQueryIndices, positions);
      final material = three.MeshBasicMaterial.fromMap({
          'color': config['triangleOutQueryColor'],
          'wireframe': config['wireframe'],
          'side': three.DoubleSide,
      });
      final mesh = three.Mesh(geometry, material);
      trianglesGroup.add(mesh);
    }
  }

  /* visualize chunk bounds */
  void visualizeChunkBounds() {
    chunkBoundsGroup.clear();

    if (!config['showChunkBounds']) return;

    final halfSize = config['queryRegionSize'] / 2;
    final queryMin = three.Vector2(config['queryRegionX'] - halfSize, config['queryRegionZ'] - halfSize);
    final queryMax = three.Vector2(config['queryRegionX'] + halfSize, config['queryRegionZ'] + halfSize);

    // Get overlapping chunks
    final chunkIndices = ChunkyTriMesh.getChunksOverlappingRect(levelChunkyTriMesh, queryMin, queryMax);
    final overlappingChunks = chunkIndices.sublist(0);

    // Show all leaf nodes with their original indices
    for (int i = 0; i < levelChunkyTriMesh.nodes.length; i++) {
      final node = levelChunkyTriMesh.nodes[i];
      final isLeaf = node.index >= 0;

      if (!isLeaf) continue;

      final isHighlighted = overlappingChunks.contains(i);

      final color = isHighlighted ? config['triangleInQueryColor'] : config['chunkBoundsColor'];

      final edges = createChunkEdges(node.bounds.min[0], node.bounds.min[1], node.bounds.max[0], node.bounds.max[1], color);
      chunkBoundsGroup.add(edges);
    }
  }

  /* visualize query region */
  void visualizeQueryRegion() {
    queryRegionGroup.clear();

    if (!config['showQueryRegion']) return;

    final halfSize = config['queryRegionSize'] / 2;
    final List<double> queryMin = [config['queryRegionX'] - halfSize, config['queryRegionZ'] - halfSize];
    final List<double> queryMax = [config['queryRegionX'] + halfSize, config['queryRegionZ'] + halfSize];

    // Draw query region edges
    final edges = createChunkEdges(queryMin[0], queryMin[1], queryMax[0], queryMax[1], config['queryRegionColor']);
    queryRegionGroup.add(edges);

    // Also draw filled transparent box for query region
    final width = queryMax[0] - queryMin[0];
    final depth = queryMax[1] - queryMin[1];
    final geometry = three.PlaneGeometry(width, depth);
    final material = three.MeshBasicMaterial.fromMap({
        'color': config['queryRegionColor'],
        'transparent': true,
        'opacity': 0.15,
        'side': three.DoubleSide,
    });
    final plane = three.Mesh(geometry, material);
    plane.rotation.x = -math.pi / 2;
    plane.position.setValues(config['queryRegionX'], 0.05, config['queryRegionZ']);
    queryRegionGroup.add(plane);
  }
  three.Vector3? getPointOnGround(event){
    final RenderBox rect = base.key.currentContext!.findRenderObject() as RenderBox;
    final size = rect.size;

    pointer.x = ((event.clientX - 0) / size.width) * 2 - 1;
    pointer.y = -((event.clientY - 0) / size.height) * 2 + 1;

    raycaster.setFromCamera(pointer, camera);

    // Intersect with a ground plane at y=0
    final groundPlane = three.Plane(three.Vector3(0, 1, 0), 0);
    final intersectPoint = three.Vector3();

    if (raycaster.ray.intersectPlane(groundPlane, intersectPoint) != null) {
      return intersectPoint;
    }

    return null;
  }


  void updateVisualization(val) {
    visualizeTriangles();
    visualizeChunkBounds();
    visualizeQueryRegion();
  }
}