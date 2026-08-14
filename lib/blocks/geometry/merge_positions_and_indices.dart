import 'package:navcat/blocks/nav_gen_input.dart';

NavMeshGenerationInput mergePositionsAndIndices(List<NavMeshGenerationInput> meshes) {
  final List<double> mergedPositions = [];
  final List<int> mergedIndices = [];
  final Map<String, int> positionToIndex = {};
  int indexCounter = 0;

  for (final mesh in meshes) {
    final positions = mesh.positions;
    final indices = mesh.indices;

    for (int i = 0; i < indices.length; i++) {
      final int pt = indices[i] * 3;
      final double x = positions[pt];
      final double y = positions[pt + 1];
      final double z = positions[pt + 2];

      final String key = '${x}_${y}_${z}';
      int? idx = positionToIndex[key];

      if (idx == null) {
        idx = indexCounter;
        positionToIndex[key] = idx;
        mergedPositions.addAll([x, y, z]);
        indexCounter++;
      }
      
      // FIX: This must be outside the 'if' block so every index is captured
      mergedIndices.add(idx); 
    }
  }

  return NavMeshGenerationInput(positions: mergedPositions, indices: mergedIndices);
}
