import 'dart:typed_data';
import 'package:three_js/three_js.dart' as three;
import 'package:navcat/navcat.dart' as navcat;

class ThreeDebug{
  three.Group createNavMeshHelper(navcat.NavMesh navMesh){
    final primitives = navcat.createNavMeshHelper(navMesh);
    return primitivesToThreeJS(primitives);
  }

  three.Group primitivesToThreeJS(List<navcat.DebugPrimitive> primitives) {
    final group = three.Group();

    for (final primitive in primitives) {
      final object = primitiveToThreeJS(primitive);
      group.add(object);
    }

    return  group;
  }

  three.Object3D primitiveToThreeJS(navcat.DebugPrimitive primitive){
    switch (primitive.type) {
        case navcat.DebugPrimitiveType.triangles: {
          final triPrimitive = primitive as navcat.DebugTriangles;
          final geometry = three.BufferGeometry();

          geometry.setAttributeFromString('position', three.Float32BufferAttribute(Float32List.fromList(triPrimitive.positions), 3));
          geometry.setAttributeFromString('color', three.Float32BufferAttribute(Float32List.fromList(triPrimitive.colors), 3));

          if (triPrimitive.indices != null && triPrimitive.indices!.isNotEmpty) {
            geometry.setIndex(three.Uint32BufferAttribute(Uint32List.fromList(triPrimitive.indices!), 1));
          }

          final material = three.MeshBasicMaterial.fromMap({
              'vertexColors': true,
              'transparent': triPrimitive.transparent ?? false,
              'opacity': triPrimitive.opacity ?? 1.0,
              'side': triPrimitive.doubleSided != null? three.DoubleSide : three.FrontSide,
          });

          return three.Mesh(geometry, material);
        }

        case navcat.DebugPrimitiveType.lines:{
          final linePrimitive = primitive as navcat.DebugLines;
          final geometry = three.BufferGeometry();

          geometry.setAttributeFromString('position', three.Float32BufferAttribute(Float32List.fromList(linePrimitive.positions), 3));
          geometry.setAttributeFromString('color', three.Float32BufferAttribute(Float32List.fromList(linePrimitive.colors), 3));

          final material = three.LineBasicMaterial.fromMap({
            'vertexColors': true,
            'transparent': linePrimitive.transparent ?? false,
            'opacity': linePrimitive.opacity ?? 1.0,
            'linewidth': linePrimitive.lineWidth ?? 1.0,
          });

          return three.LineSegments(geometry, material);
        }

        case navcat.DebugPrimitiveType.points: {
            final pointPrimitive = primitive as navcat.DebugPoints;
            final group = three.Group();

            final numPoints = pointPrimitive.positions.length ~/ 3;

            if (numPoints > 0) {
                // Create sphere geometry for instancing
                final sphereGeometry = three.SphereGeometry(1, 8, 6); // Low-poly sphere for performance

                final material = three.MeshBasicMaterial.fromMap({
                    'vertexColors': true,
                    'transparent': pointPrimitive.transparent ?? false,
                    'opacity': pointPrimitive.opacity ?? 1.0,
                });

                final instancedMesh = three.InstancedMesh(sphereGeometry, material, numPoints);

                final matrix = three.Matrix4();
                final baseSize = pointPrimitive.size ?? 1.0;

                for (int i = 0; i < numPoints; i++) {
                  final x = pointPrimitive.positions[i * 3];
                  final y = pointPrimitive.positions[i * 3 + 1];
                  final z = pointPrimitive.positions[i * 3 + 2];

                  final sphereSize = baseSize;

                  matrix.makeScale(sphereSize, sphereSize, sphereSize);
                  matrix.setPosition(x, y, z);
                  instancedMesh.setMatrixAt(i, matrix);

                  final color = three.Color(
                    pointPrimitive.colors[i * 3],
                    pointPrimitive.colors[i * 3 + 1],
                    pointPrimitive.colors[i * 3 + 2],
                  );
                  instancedMesh.setColorAt(i, color);
                }

                instancedMesh.instanceMatrix?.needsUpdate = true;
                instancedMesh.instanceColor?.needsUpdate = true;

                group.add(instancedMesh);
            }

            return group;
        }

        case navcat.DebugPrimitiveType.boxes: {
            final boxPrimitive = primitive as navcat.DebugBoxes;
            final group = three.Group();

            // Create instanced mesh for all boxes
            final boxGeometry = three.BoxGeometry(1, 1, 1);
            final numBoxes = boxPrimitive.positions.length ~/ 3;

            if (numBoxes > 0) {
                final material = three.MeshBasicMaterial.fromMap({
                  'vertexColors': false,
                  'transparent': boxPrimitive.transparent ?? false,
                  'opacity': boxPrimitive.opacity ?? 1.0,
                });

                final instancedMesh = three.InstancedMesh(boxGeometry, material, numBoxes);
                final matrix = three.Matrix4();

                for (int i = 0; i < numBoxes; i++) {
                  final x = boxPrimitive.positions[i * 3];
                  final y = boxPrimitive.positions[i * 3 + 1];
                  final z = boxPrimitive.positions[i * 3 + 2];

                  final double scaleX = boxPrimitive.scales != null? boxPrimitive.scales![i * 3] : 1;
                  final double scaleY = boxPrimitive.scales != null? boxPrimitive.scales![i * 3 + 1] : 1;
                  final double scaleZ = boxPrimitive.scales != null? boxPrimitive.scales![i * 3 + 2] : 1;

                  matrix.makeScale(scaleX, scaleY, scaleZ);
                  matrix.setPosition(x, y, z);
                  instancedMesh.setMatrixAt(i, matrix);

                  final color = three.Color(
                    boxPrimitive.colors[i * 3],
                    boxPrimitive.colors[i * 3 + 1],
                    boxPrimitive.colors[i * 3 + 2],
                  );
                  instancedMesh.setColorAt(i, color);
                }

                instancedMesh.instanceMatrix?.needsUpdate = true;
                instancedMesh.instanceColor?.needsUpdate = true;

                group.add(instancedMesh);
            }

            return group;
        }
    }
  }

  three.Object3D createTriangleAreaIdsHelper ({
    required navcat.MeshInput input,
    required List<int> triAreaIds,
  }){
    final primitives = navcat.createTriangleAreaIdsHelper(input, triAreaIds);
    return primitiveToThreeJS(primitives!);
  }

 three.Object3D createHeightfieldHelper(navcat.Heightfield heightfield){
    final primitives = navcat.createHeightfieldHelper(heightfield);
    return primitiveToThreeJS(primitives!);
  }

  three.Object3D createCompactHeightfieldSolidHelper(navcat.CompactHeightfield compactHeightfield){
    final primitives = navcat.createCompactHeightfieldSolidHelper(compactHeightfield);
    return primitiveToThreeJS(primitives!);
  }

  three.Object3D createCompactHeightfieldDistancesHelper(navcat.CompactHeightfield compactHeightfield){
    final primitives = navcat.createCompactHeightfieldDistancesHelper(compactHeightfield);
    return primitiveToThreeJS(primitives!);
  }

  three.Object3D createCompactHeightfieldRegionsHelper(navcat.CompactHeightfield compactHeightfield){
    final primitives = navcat.createCompactHeightfieldRegionsHelper(compactHeightfield);
    return primitiveToThreeJS(primitives!);
  }

  three.Group createRawContoursHelper(navcat.ContourSet contourSet){
    final primitives = navcat.createRawContoursHelper(contourSet);
    return primitivesToThreeJS(primitives);
  }

  three.Group createSimplifiedContoursHelper(navcat.ContourSet contourSet){
    final primitives = navcat.createSimplifiedContoursHelper(contourSet);
    return primitivesToThreeJS(primitives);
  }

  three.Group createPolyMeshHelper(navcat.PolyMesh polyMesh){
    final primitives = navcat.createPolyMeshHelper(polyMesh);
    return primitivesToThreeJS(primitives);
  }

  three.Group createPolyMeshDetailHelper(navcat.PolyMeshDetail polyMeshDetail){
    final primitives = navcat.createPolyMeshDetailHelper(polyMeshDetail);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshTileHelper(navcat.NavMeshTile tile){
    final primitives = navcat.createNavMeshTileHelper(tile);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshPolyHelper(
    navcat.NavMesh navMesh,
    int nodeRef,
    [List<double>? color]
  ){
    color ??= [0, 0.75, 1];
    final primitives = navcat.createNavMeshPolyHelper(navMesh, nodeRef, color);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshTileBvTreeHelper(navcat.NavMeshTile navMeshTile){
    final primitives = navcat.createNavMeshTileBvTreeHelper(navMeshTile);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshLinksHelper(navcat.NavMesh navMesh){
    final primitives = navcat.createNavMeshLinksHelper(navMesh);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshBvTreeHelper(navcat.NavMesh navMesh){
    final primitives = navcat.createNavMeshBvTreeHelper(navMesh);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshTilePortalsHelper(navcat.NavMeshTile navMeshTile){
    final primitives = navcat.createNavMeshTilePortalsHelper(navMeshTile);
    return primitivesToThreeJS(primitives);
  }

  three.Group createNavMeshPortalsHelper(navcat.NavMesh navMesh){
    final primitives = navcat.createNavMeshPortalsHelper(navMesh);
    return primitivesToThreeJS(primitives);
  }

  three.Group createSearchNodesHelper(navcat.SearchNodePool nodePool){
    final primitives = navcat.createSearchNodesHelper(nodePool);
    return primitivesToThreeJS(primitives);
  }
  
  three.Group createNavMeshOffMeshConnectionsHelper(navcat.NavMesh navMesh){
    final primitives = navcat.createNavMeshOffMeshConnectionsHelper(navMesh);
    return primitivesToThreeJS(primitives);
  }
}