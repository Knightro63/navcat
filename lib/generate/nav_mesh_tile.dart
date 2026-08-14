import 'dart:typed_data';
import 'package:navcat/generate/poly_neighbours.dart';
import 'package:navcat/navcat.dart';
import 'package:three_js_math/three_js_math.dart';

/// Converts a PolyMesh to the NavMeshTile structure.
TilePolys polyMeshToTilePolys(PolyMesh polyMesh) {
  final List<double> vertices = polyMesh.vertices.sublist(0);
  
  final int nvp = polyMesh.maxVerticesPerPoly;
  final int nPolys = polyMesh.nPolys;
  final List<NavMeshPoly> polys = [];
  
  for (int i = 0; i < nPolys; i++) {
    // Replicating NavMeshPoly structural mapping cleanly
    final NavMeshPoly poly = NavMeshPoly(
      vertices: <int>[],
      neis: <int>[],
      flags: polyMesh.flags[i],
      area: polyMesh.areas[i],
    );
    
    // extract polygon data for this polygon
    final int polyStart = i * nvp;
    for (int j = 0; j < nvp; j++) {
      final int vertIndex = polyMesh.polys[polyStart + j];
      if (vertIndex == meshNullIdx) break;
      poly.vertices.add(vertIndex);
    }
    polys.add(poly);
  }
  
  // build poly neighbours information
  buildPolyNeighbours(
    polys, 
    vertices, 
    polyMesh.borderSize, 
    0.0, 
    0.0, 
    polyMesh.localWidth, 
    polyMesh.localHeight,
  );
  
  // convert vertices to world space
  // we do this after buildPolyNeighbours so that neighbour calculation can be done with quantized values
  for (int i = 0; i < vertices.length; i += 3) {
    vertices[i] = polyMesh.bounds.min[0] + vertices[i] * polyMesh.cellSize;
    vertices[i + 1] = polyMesh.bounds.min[1] + vertices[i + 1] * polyMesh.cellHeight;
    vertices[i + 2] = polyMesh.bounds.min[2] + vertices[i + 2] * polyMesh.cellSize;
  }
  
  return TilePolys(
    vertices: vertices,
    polys: polys,
  );
}

/// Builds NavMeshTile polys from given external polygon objects.
TilePolys polygonsToNavMeshTilePolys(
  List<NavMeshPoly> polygons,
  Float32List vertices,
  double borderSize,
  BoundingBox bounds,
) {
  final List<NavMeshPoly> polys = [];
  
  for (final poly in polygons) {
    polys.add(NavMeshPoly(
      vertices: List<int>.from(poly.vertices),
      flags: poly.flags,
      area: poly.area,
      neis: <int>[],
    ));
  }
  
  // Accessing three_js_math BoundingBox vector components cleanly
  final double minX = bounds.min.x;
  final double minZ = bounds.min.z;
  final double maxX = bounds.max.x;
  final double maxZ = bounds.max.z;
  
  buildPolyNeighbours(polys, vertices, borderSize, minX, minZ, maxX, maxZ);
  
  return TilePolys(
    vertices: vertices,
    polys: polys,
  );
}

/// Creates a detail mesh from the given polygon data using fan triangulation.
TileDetailMesh polysToTileDetailMesh(List<PolyMesh> polys) {
  final List<int> detailTriangles = [];
  final List<NavMeshPolyDetail?> detailMeshes = List<NavMeshPolyDetail?>.filled(polys.length, null, growable: false);
  
  int tbase = 0;
  
  for (int polyId = 0; polyId < polys.length; polyId++) {
    final PolyMesh poly = polys[polyId];
    final int nv = poly.vertices.length;
    
    // create detail mesh descriptor for this polygon
    final NavMeshPolyDetail detailMesh = NavMeshPolyDetail(
      verticesBase: 0, 
      verticesCount: 0, 
      trianglesBase: tbase, 
      trianglesCount: nv - 2, 
    );
    detailMeshes[polyId] = detailMesh;
    
    // triangulate polygon using fan triangulation (local indices within the polygon)
    for (int j = 2; j < nv; j++) {
      // create triangle using vertex 0 and two consecutive vertices
      detailTriangles.add(0);     // first vertex (local index)
      detailTriangles.add(j - 1); // previous vertex (local index)
      detailTriangles.add(j);     // current vertex (local index)
      
      // edge flags - bit for each edge that belongs to poly boundary
      int edgeFlags = 1 << 2; // edge 2 is always a polygon boundary
      if (j == 2) edgeFlags |= 1 << 0;       // first triangle, edge 0 is boundary
      if (j == nv - 1) edgeFlags |= 1 << 4;  // last triangle, edge 1 is boundary
      detailTriangles.add(edgeFlags);
      tbase++;
    }
  }
  
  return TileDetailMesh(
    detailMeshes: detailMeshes,
    detailTriangles: detailTriangles,
    detailVertices: <double>[],
  );
}

/// Converts a given PolyMeshDetail to the tile detail mesh format.
TileDetailMesh polyMeshDetailToTileDetailMesh(List<NavMeshPoly> polys, PolyMeshDetail polyMeshDetail) {
  final List<NavMeshPolyDetail?> detailMeshes = List<NavMeshPolyDetail?>.filled(polys.length, null, growable: false);
  final List<double> detailVertices = [];
  
  int vbase = 0;
  
  for (int i = 0; i < polys.length; i++) {
    final NavMeshPoly poly = polys[i];
    final int nPolyVertices = poly.vertices.length;
    
    final int vb = polyMeshDetail.meshes[i * 4];
    final int nDetailVertices = polyMeshDetail.meshes[i * 4 + 1];
    final int trianglesBase = polyMeshDetail.meshes[i * 4 + 2];
    final int trianglesCount = polyMeshDetail.meshes[i * 4 + 3];
    
    final int nAdditionalDetailVertices = nDetailVertices - nPolyVertices;
    
    final NavMeshPolyDetail detailMesh = NavMeshPolyDetail(
      verticesBase: vbase,
      verticesCount: nAdditionalDetailVertices,
      trianglesBase: trianglesBase,
      trianglesCount: trianglesCount,
    );
    detailMeshes[i] = detailMesh;
    
    // Copy vertices except the first 'nv' verts which are equal to nav poly verts.
    if (nAdditionalDetailVertices > 0) {
      for (int j = nPolyVertices; j < nDetailVertices; j++) {
        final int detailVertIndex = (vb + j) * 3;
        detailVertices.add(polyMeshDetail.vertices[detailVertIndex]);
        detailVertices.add(polyMeshDetail.vertices[detailVertIndex + 1]);
        detailVertices.add(polyMeshDetail.vertices[detailVertIndex + 2]);
      }
      vbase += nAdditionalDetailVertices;
    }
  }
  
  return TileDetailMesh(
    detailMeshes: detailMeshes,
    detailVertices: detailVertices,
    detailTriangles: polyMeshDetail.triangles,
  );
}


class TileDetailMesh{
  List<NavMeshPolyDetail?>detailMeshes;
  List<double> detailVertices;
  List<int> detailTriangles;

  TileDetailMesh({
    required this.detailMeshes,
    required this.detailTriangles,
    required this.detailVertices
  });
}

class TilePolys{
  List<NavMeshPoly> polys;
  List<double> vertices;

  TilePolys({
    required this.polys,
    required this.vertices
  });
}