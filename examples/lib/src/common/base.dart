import 'dart:typed_data';
import 'package:examples/src/common/debug.dart';
import 'package:flutter/cupertino.dart';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart' as navcat;

class ExampleBase{
  late final three.ThreeJS threeJs;
  three.Scene get scene => threeJs.scene;
  three.Camera get camera => threeJs.camera;
  GlobalKey<three.PeripheralsState> get key => threeJs.globalKey;
  three.PeripheralsState get domElement => threeJs.domElement;
  Widget get build => threeJs.build();
  final ThreeDebug debug = ThreeDebug();

  final Function setState;

  ExampleBase(this.setState, Future<void> Function()? setup){
    threeJs = three.ThreeJS(
      settings: three.Settings(
        toneMapping: three.ACESFilmicToneMapping,
      ),
      onSetupComplete: (){setState(() {});},
      setup: () async{
        init();
        await setup?.call();
      },
    );
  }

  void dispose(){
    threeJs.dispose();
  }

  void init(){
    threeJs.scene = three.Scene();
    scene.background = three.Color.fromHex32(0x202020);

    threeJs.camera = three.PerspectiveCamera(
      75,
      threeJs.width / threeJs.height,
      0.1,
      1000,
    );
    camera.position.setValues(0, 0, 5);

    // lighting
    final ambientLight = three.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    final directionalLight = three.DirectionalLight(0xffffff, 1);
    directionalLight.position.setValues(5, 5, 5);
    scene.add(directionalLight);
  }

  three.Object3D createFlag(int color){
    final poleGeom = three.BoxGeometry(0.12, 1.2, 0.12);
    final poleMat = three.MeshStandardMaterial.fromMap({ 'color': 0x888888 });
    final pole = three.Mesh(poleGeom, poleMat);
    pole.position.setValues(0, 0.6, 0);

    final flagGeom = three.BoxGeometry(0.32, 0.22, 0.04);
    final flagMat = three.MeshStandardMaterial.fromMap({ 'color': color });
    final flag = three.Mesh(flagGeom, flagMat);
    flag.position.setValues(0.23, 1.0, 0);

    final group = three.Group();
    group.add(pole);
    group.add(flag);

    return group;
  }

  ///
  /// Computes a uniform cost flow field.
  /// The "uniform cost" approach trades off accuracy for speed. We are assuming the cost of traversing each polygon is uniform,
  /// meaning we aren't taking into account polygon sizes or other custom cost calculations from QueryFilter.getCost.
  /// We do however still check QueryFilter.passFilter.
  ////
  FlowField computeUniformCostFlowField(
    navcat.NavMesh navMesh,
    int targetRef,
    navcat.QueryFilter queryFilter,
    int maxIterations,
  ){
    final cost = <int, double>{};
    final next = <int, int?>{};
    final visited = <int>[];
    final List<Map<String,dynamic>> queue = [{ 'nodeRef': targetRef, 'c': 0 }];

    cost[targetRef] = 0;
    next[targetRef] = null;
    visited.add(targetRef);

    int iterations = 0;
    while (queue.isNotEmpty && iterations < maxIterations) {
      final q = queue.removeAt(0);
      final currentRef = q['nodeRef'];
      final currentCost = q['c'];
      iterations++;

      // get links for this node
      final node = navcat.getNodeByRef(navMesh, currentRef);

      for (final linkIndex in node?.links ?? []) {
        final link = navMesh.links[linkIndex];

        final neighborRef = link?.toNodeRef ?? 0;

        if (visited.contains(neighborRef)) continue;

        if (!queryFilter.passFilter(neighborRef, navMesh)) continue;

        cost[neighborRef] = currentCost + 1;
        next[neighborRef] = currentRef;
        visited.add(neighborRef);
        queue.add({ 'nodeRef': neighborRef, 'c': currentCost + 1 });
      }
    }

    return FlowField(cost:cost, next:next, visited:visited );
  }

  ///
  /// Extracts a path from a start node to the target using the flow field.
  /// @param flowField - The computed flow field.
  /// @param startNodeRef - The starting node reference.
  /// @returns An array of NodeRefs representing the path, or null if unreachable.
  ///
  List<int>? getNodePathFromFlowField(FlowField flowField, int startNodeRef){
    final List<int> path = [];
    int current = startNodeRef;
    final visited = <int>[];

    while (current != navcat.invalidNodeRef && flowField.next.containsKey(current) && !visited.contains(current)) {
      path.add(current);
      visited.add(current);
      final next = flowField.next[current];
      if (next == null) break; // reached target
      current = next;
    }

    // If the last node is not the target, path is incomplete/unreachable
    if (!flowField.next.containsKey(current) || flowField.next[current] != null) {
      return null;
    }

    return path;
  }

  final _position = three.Vector3();

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
      List<int>? indices =  index != null? List.from(index): null;
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

class FlowField {
  final Map<int, double> cost;
  final Map<int, int?> next;
  final List<int> visited;

  FlowField({
    required this.cost,
    required this.next,
    required this.visited
  });
}

class NavMeshOptions {
  double walkableClimbVoxels = 0;
  double walkableHeightVoxels = 0;
  double walkableRadiusVoxels = 0;

  double cellSize;
  double cellHeight;
  double walkableRadiusWorld;
  double walkableClimbWorld;
  double walkableHeightWorld;
  double walkableSlopeAngleDegrees;
  int borderSize;
  double minRegionArea;
  double mergeRegionArea;
  double maxSimplificationError;
  double maxEdgeLength;
  int maxVerticesPerPoly;
  double detailSampleDistance;
  double detailSampleMaxError;

  NavMeshOptions({
    this.cellSize = 0.5,
    this.cellHeight = 0.2,
    this.walkableRadiusWorld = 0.3,
    this.walkableClimbWorld = 0.4,
    this.walkableHeightWorld = 0.5,
    this.walkableSlopeAngleDegrees = 45,
    this.borderSize = 4,
    this.minRegionArea = 8,
    this.mergeRegionArea = 20,
    this.maxSimplificationError = 1.3,
    this.maxEdgeLength = 12,
    this.maxVerticesPerPoly = 6,
    this.detailSampleDistance = 0.6,
    this.detailSampleMaxError = 0.2,

    this.walkableClimbVoxels = 0,
    this.walkableHeightVoxels = 0,
    this.walkableRadiusVoxels = 0
  });
}