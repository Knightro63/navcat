import 'package:navcat/navcat.dart';
import 'dart:typed_data';

// --- Ported Processing Functions ---
void buildPolyNeighbours(
  List<NavMeshPoly> polys,
  List<double> vertices,
  double borderSize,
  double minX,
  double minZ,
  double maxX,
  double maxZ,
) {
  // initialize neis arrays for all polygons
  for (final NavMeshPoly poly in polys) {
    final int vertexCount = poly.vertices.length;
    poly.neis = List<int>.filled(vertexCount, meshNullIdx);
  }
  
  // build adjacency information, finds internal neighbours for each polygon edge
  buildMeshAdjacency(polys, vertices.length ~/ 3);
  
  // find portal edges
  if (borderSize > 0) {
    findPortalEdges(polys, vertices, minX, minZ, maxX, maxZ);
  }
  
  // final poly neis formatting
  finalizePolyNeighbours(polys);
}

class Edge{
  late final List<int> vert;
  late final List<int> polyEdge;
  late final List<int> poly;

  Edge({
    List<int>? poly,
    List<int>? polyEdge,
    List<int>? vert
  }){
    this.poly = poly ?? [];
    this.polyEdge = polyEdge ?? [];
    this.vert = vert ?? [];
  }
}

/// Finds polygon edge neighbours, populates the neis array for each polygon.
void buildMeshAdjacency(List<NavMeshPoly> polys, int vertexCount) {
  final int polygonCount = polys.length;
  
  // Equates to polys.reduce((sum, poly) => sum + poly.vertices.length)
  int maxEdgeCount = 0;
  for (final NavMeshPoly poly in polys) {
    maxEdgeCount += poly.vertices.length;
  }
  
  final List<int> firstEdge = List<int>.filled(vertexCount, meshNullIdx);
  final List<int> nextEdge = List<int>.filled(maxEdgeCount, meshNullIdx);
  int edgeCount = 0;
  
  // Using a list of maps to handle the 'Edge' type without generating a rigid class structure
  final List<Edge> edges = [];//List<Edge>.filled(maxEdgeCount, Edge());
  for (int i = 0; i < maxEdgeCount; i++) {
    edges.add(Edge());
  }
  
  for (int i = 0; i < vertexCount; i++) {
    firstEdge[i] = meshNullIdx;
  }
  
  // build edges
  for (int i = 0; i < polygonCount; i++) {
    final NavMeshPoly poly = polys[i];
    final List<int> polyVerts = List<int>.from(poly.vertices);
    final int nv = polyVerts.length;
    
    for (int j = 0; j < nv; j++) {
      final int v0 = polyVerts[j];
      final int v1 = polyVerts[(j + 1) % nv];
      
      if (v0 < v1) {
        edges[edgeCount] = Edge(
          vert: <int>[v0, v1],
          poly: <int>[i, i],
          polyEdge: <int>[j, 0],
        );
        nextEdge[edgeCount] = firstEdge[v0];
        firstEdge[v0] = edgeCount;
        edgeCount++;
      }
    }
  }
  
  // match edges
  for (int i = 0; i < polygonCount; i++) {
    final NavMeshPoly poly = polys[i];
    final List<int> polyVerts = List<int>.from(poly.vertices);
    final int nv = polyVerts.length;
    
    for (int j = 0; j < nv; j++) {
      final int v0 = polyVerts[j];
      final int v1 = polyVerts[(j + 1) % nv];
      
      if (v0 > v1) {
        for (int e = firstEdge[v1]; e != meshNullIdx; e = nextEdge[e]) {
          final Edge edge = edges[e];
          final List<int> eV = edge.vert;
          final List<int> eP = edge.poly;
          
          if (eV[1] == v0 && eP[0] == eP[1]) {
            eP[1] = i;
            edge.polyEdge[1] = j;
            break;
          }
        }
      }
    }
  }
  
  // store adjacency
  for (int i = 0; i < edgeCount; i++) {
    final Edge e = edges[i];
    final List<int> eP = e.poly;
    final List<int> ePE = e.polyEdge;
    
    if (eP[0] != eP[1]) {
      polys[eP[0]].neis[ePE[0]] = eP[1];
      polys[eP[1]].neis[ePE[1]] = eP[0];
    }
  }
}

void findPortalEdges(
  List<NavMeshPoly> polys,
  List<double> vertices,
  double minX,
  double minZ,
  double maxX,
  double maxZ,
) {
  // Minor performance caching optimization mimicking [0,0,0] coordinates scratchpad 
  final Float32List va = Float32List(3);
  final Float32List vb = Float32List(3);
  
  for (int i = 0; i < polys.length; i++) {
    final NavMeshPoly poly = polys[i];
    final List<int> polyVerts = List<int>.from(poly.vertices);
    final List<int> polyNeis = poly.neis;
    final int nv = polyVerts.length;
    
    for (int j = 0; j < nv; j++) {
      // skip connected edges
      if (polyNeis[j] != meshNullIdx) {
        continue;
      }
      
      final int nj = (j + 1) % nv;
      
      final int vIdx = polyVerts[j] * 3;
      va[0] = vertices[vIdx];
      va[1] = vertices[vIdx + 1];
      va[2] = vertices[vIdx + 2];
      
      final int nvIdx = polyVerts[nj] * 3;
      vb[0] = vertices[nvIdx];
      vb[1] = vertices[nvIdx + 1];
      vb[2] = vertices[nvIdx + 2];
      
      if (va[0] == minX && vb[0] == minX) {
        polyNeis[j] = polyNeisFlagExtLink | 0;
      } else if (va[2] == maxZ && vb[2] == maxZ) {
        polyNeis[j] = polyNeisFlagExtLink | 1;
      } else if (va[0] == maxX && vb[0] == maxX) {
        polyNeis[j] = polyNeisFlagExtLink | 2;
      } else if (va[2] == minZ && vb[2] == minZ) {
        polyNeis[j] = polyNeisFlagExtLink | 3;
      }
    }
  }
}

void finalizePolyNeighbours(List<NavMeshPoly> polys) {
  for (final NavMeshPoly poly in polys) {
    final List<int> polyNeis = poly.neis;
    final int len = polyNeis.length;
    
    for (int i = 0; i < len; i++) {
      final int neiValue = polyNeis[i];
      
      if ((neiValue & polyNeisFlagExtLink) != 0) {
        // border or portal edge
        final int dir = neiValue & 0xf;
        if (dir == 0xf) {
          polyNeis[i] = 0;
        } else if (dir == 0) {
          polyNeis[i] = polyNeisFlagExtLink | 4; // Portal x-
        } else if (dir == 1) {
          polyNeis[i] = polyNeisFlagExtLink | 2; // Portal z+
        } else if (dir == 2) {
          polyNeis[i] = polyNeisFlagExtLink | 0; // Portal x+
        } else if (dir == 3) {
          polyNeis[i] = polyNeisFlagExtLink | 6; // Portal z-
        } else {
          polyNeis[i] = 0;
        }
      } else {
        // normal internal connection (add 1 to convert from 0-based to 1-based indexing)
        polyNeis[i] = neiValue + 1;
      }
    }
  }
}
