import '../../navcat.dart'; 

class NavMeshFill { 
  final List<int> reachable; 
  final List<int> unreachable; 

  NavMeshFill({ 
    List<int>? reachable, 
    List<int>? unreachable 
  })  : this.reachable = reachable ?? [], 
        this.unreachable = unreachable ?? []; 
} 

NavMeshFill floodFillNavMesh(NavMesh navMesh, List<int> startNodeRefs) { 
  // 1. Corrected 'number' to 'int'
  final visited = <int>{}; 
  
  // 2. Used a Queue class for O(1) performance instead of List
  final queue = <int>[]; 

  for (final startRef in startNodeRefs) { 
    queue.add(startRef); 
  } 

  // 3. Changed 'length > 0' to 'isNotEmpty'
  while (queue.isNotEmpty) { 
    // 4. Changed JS '.shift()!' to Dart '.removeFirst()'
    final currentNodeRef = queue.removeAt(0); 
    
    if (visited.contains(currentNodeRef)) continue; 

    visited.add(currentNodeRef); 

    final nodeIndex = getNodeRefIndex(currentNodeRef); 
    final node = navMesh.nodes[nodeIndex]; 

    for (final linkIndex in (node?.links ?? [])) { 
      final link = navMesh.links[linkIndex]; 
      if (visited.contains(link!.toNodeRef)) continue; 
      queue.add(link.toNodeRef); 
    } 
  } 

  // 5. Corrected conversion of Set to List
  final List<int> reachable = visited.toList(); 
  final List<int> unreachable = []; 

  // 6. Evaluated map loops cleanly
  for (final tileId in navMesh.tiles.keys) { 
    final tile = navMesh.tiles[tileId]!; 
    for (int polyIndex = 0; polyIndex < (tile.polys?.length ?? 0); polyIndex++) { 
      final node = getNodeByTileAndPoly(navMesh, tile, polyIndex); 
      if (!visited.contains(node?.ref)) { 
        unreachable.add(node!.ref); 
      } 
    } 
  } 

  return NavMeshFill( 
    reachable: reachable, 
    unreachable: unreachable, 
  ); 
}
