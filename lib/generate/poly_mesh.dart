import 'dart:math' as math;
import 'package:navcat/debug.dart';
import 'package:three_js_math/three_js_math.dart';
import 'index.dart';

// Constant parameters matching your geometric evaluation thresholds
const int vertexYtolerence = 2;

class VertexEntry{
  double y;
  int index;

  VertexEntry({
    this.y = 0,
    this.index = 0
  });
}

/// Adds or merges a structural vertex coordinate based on spatial tolerances
int addVertex(
  double x,
  double y,
  double z,
  List<double> vertices,
  List<int> vflags,
  Map<String, List<VertexEntry>> vertexMap,
) {
  // key by X and Z only; we'll search the bucket for a vertex with similar Y
  final String keyXZ = "$x,$z";
  final List<VertexEntry>? bucket = vertexMap[keyXZ];
  
  if (bucket != null) {
    for (int i = 0; i < bucket.length; i++) {
      final VertexEntry b = bucket[i];
      final entryY = b.y;
      final idx = b.index;
      // x and z match, check y tolerance
      if ((entryY - y).abs() <= vertexYtolerence) {
        return idx;
      }
    }
  }
  
  // could not find — create new vertex element sequence
  final int i = vertices.length ~/ 3;
  vertices.add(x);
  vertices.add(y);
  vertices.add(z);
  
  final VertexEntry newEntry = VertexEntry(y: y,index: i);
  if (bucket != null) {
    bucket.add(newEntry);
  } else {
    vertexMap[keyXZ] = [newEntry];
  }
  
  vflags.add(0);
  return i;
}

/// Computes the 2D signed cross product area of a triangle on the XZ-plane
double _area2(Vector3 vertexA, Vector3 vertexB, Vector3 vertexC) {
  return (vertexB[0] - vertexA[0]) * (vertexC[2] - vertexA[2]) - 
         (vertexC[0] - vertexA[0]) * (vertexB[2] - vertexA[2]);
}

bool _xorb(bool x, bool y) => !x != !y;

// Returns true if testVertex is strictly to the left of the directed line through a to b
bool _left(Vector3 firstVertex, Vector3 secondVertex, Vector3 testVertex) => 
    _area2(firstVertex, secondVertex, testVertex) < 0;

bool _leftOn(Vector3 firstVertex, Vector3 secondVertex, Vector3 testVertex) => 
    _area2(firstVertex, secondVertex, testVertex) <= 0;

bool _collinear(Vector3 firstVertex, Vector3 secondVertex, Vector3 testVertex) => 
    _area2(firstVertex, secondVertex, testVertex) == 0;

bool _intersectProp(
  Vector3 segmentAStart,
  Vector3 segmentAEnd,
  Vector3 segmentBStart,
  Vector3 segmentBEnd,
) {
  if (_collinear(segmentAStart, segmentAEnd, segmentBStart) ||
      _collinear(segmentAStart, segmentAEnd, segmentBEnd) ||
      _collinear(segmentBStart, segmentBEnd, segmentAStart) ||
      _collinear(segmentBStart, segmentBEnd, segmentAEnd)) {
    return false;
  }
  return (
    _xorb(_left(segmentAStart, segmentAEnd, segmentBStart), _left(segmentAStart, segmentAEnd, segmentBEnd)) &&
    _xorb(_left(segmentBStart, segmentBEnd, segmentAStart), _left(segmentBStart, segmentBEnd, segmentAEnd))
  );
}

bool _between(Vector3 startVertex, Vector3 endVertex, Vector3 testVertex) {
  if (!_collinear(startVertex, endVertex, testVertex)) return false;
  if (startVertex[0] != endVertex[0]) {
    return (
      (startVertex[0] <= testVertex[0] && testVertex[0] <= endVertex[0]) ||
      (startVertex[0] >= testVertex[0] && testVertex[0] >= endVertex[0])
    );
  }
  return (
    (startVertex[2] <= testVertex[2] && testVertex[2] <= endVertex[2]) ||
    (startVertex[2] >= testVertex[2] && testVertex[2] >= endVertex[2])
  );
}

bool _intersect(Vector3 segmentAStart, Vector3 segmentAEnd, Vector3 segmentBStart, Vector3 segmentBEnd) {
  if (_intersectProp(segmentAStart, segmentAEnd, segmentBStart, segmentBEnd)) return true;
  return (
    _between(segmentAStart, segmentAEnd, segmentBStart) ||
    _between(segmentAStart, segmentAEnd, segmentBEnd) ||
    _between(segmentBStart, segmentBEnd, segmentAStart) ||
    _between(segmentBStart, segmentBEnd, segmentAEnd)
  );
}

// Returns whether the two vertices are equal in the XZ plane
bool vec3EqualXZ(Vector3 vertexA, Vector3 vertexB) => 
    vertexA.x == vertexB.x && vertexA.z == vertexB.z;

// High performance file-private static vector caches for frame loops
final Vector3 _diagonalStart = Vector3();
final Vector3 _diagonalEnd = Vector3();
final Vector3 _edgeStart = Vector3();
final Vector3 _edgeEnd = Vector3();

// Returns true iff (v_i, v_j) is a proper internal or external diagonal of P, ignoring incident edges.
bool diagonalie(
  int startVertexIdx,
  int endVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  final Vector3 diagonalStart = _diagonalStart.fromArray(vertices, (vertexIndices[startVertexIdx] & 0x0fffffff) * 4);
  final Vector3 diagonalEnd = _diagonalEnd.fromArray(vertices, (vertexIndices[endVertexIdx] & 0x0fffffff) * 4);
  
  for (int k = 0; k < polygonVertexCount; k++) {
    final int k1 = next(k, polygonVertexCount);
    if (!(k == startVertexIdx || k1 == startVertexIdx || k == endVertexIdx || k1 == endVertexIdx)) {
      final Vector3 edgeStart = _edgeStart.fromArray(vertices, (vertexIndices[k] & 0x0fffffff) * 4);
      final Vector3 edgeEnd = _edgeEnd.fromArray(vertices, (vertexIndices[k1] & 0x0fffffff) * 4);
      
      if (vec3EqualXZ(diagonalStart, edgeStart) ||
          vec3EqualXZ(diagonalEnd, edgeStart) ||
          vec3EqualXZ(diagonalStart, edgeEnd) ||
          vec3EqualXZ(diagonalEnd, edgeEnd)) {
        continue;
      }
      
      if (_intersect(diagonalStart, diagonalEnd, edgeStart, edgeEnd)) {
        return false;
      }
    }
  }
  return true;
}

final Vector3 _coneVertex = Vector3();
final Vector3 _testVertex = Vector3();
final Vector3 _nextVertex = Vector3();
final Vector3 _prevVertex = Vector3();

bool _inCone(
  int coneVertexIdx,
  int testVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  final Vector3 coneVertex = _coneVertex.fromArray(vertices, (vertexIndices[coneVertexIdx] & 0x0fffffff) * 4);
  final Vector3 testVertex = _testVertex.fromArray(vertices, (vertexIndices[testVertexIdx] & 0x0fffffff) * 4);

  final Vector3 nextVertex = _nextVertex.fromArray( vertices,
    (vertexIndices[next(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
  final Vector3 prevVertex = _prevVertex.fromArray(vertices,
    (vertexIndices[prev(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
  
  if (_leftOn(prevVertex, coneVertex, nextVertex)) {
    return _left(coneVertex, testVertex, prevVertex) && _left(testVertex, coneVertex, nextVertex);
  }
  return !(_leftOn(coneVertex, testVertex, nextVertex) && _leftOn(testVertex, coneVertex, prevVertex));
}

bool diagonal(
  int startVertexIdx,
  int endVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  return _inCone(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) &&
         diagonalie(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices);
}

bool diagonalieLoose(
  int startVertexIdx,
  int endVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  final Vector3 diagonalStart = _diagonalStart.fromArray(vertices, (vertexIndices[startVertexIdx] & 0x0fffffff) * 4);
  final Vector3 diagonalEnd = _diagonalEnd.fromArray(vertices, (vertexIndices[endVertexIdx] & 0x0fffffff) * 4);
  
  for (int k = 0; k < polygonVertexCount; k++) {
    final int k1 = next(k, polygonVertexCount);
    if (!(k == startVertexIdx || k1 == startVertexIdx || k == endVertexIdx || k1 == endVertexIdx)) {
      final Vector3 edgeStart = _edgeStart.fromArray(vertices, (vertexIndices[k] & 0x0fffffff) * 4);
      final Vector3 edgeEnd = _edgeEnd.fromArray(vertices, (vertexIndices[k1] & 0x0fffffff) * 4);
      
      if (vec3EqualXZ(diagonalStart, edgeStart) ||
          vec3EqualXZ(diagonalEnd, edgeStart) ||
          vec3EqualXZ(diagonalStart, edgeEnd) ||
          vec3EqualXZ(diagonalEnd, edgeEnd)) {
        continue;
      }
      
      if (_intersectProp(diagonalStart, diagonalEnd, edgeStart, edgeEnd)) {
        return false;
      }
    }
  }
  return true;
}

bool inConeLoose(
  int coneVertexIdx,
  int testVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  final Vector3 coneVertex = _coneVertex.fromArray(vertices, (vertexIndices[coneVertexIdx] & 0x0fffffff) * 4);
  final Vector3 testVertex = _testVertex.fromArray(vertices, (vertexIndices[testVertexIdx] & 0x0fffffff) * 4);
  final Vector3 nextVertex = _nextVertex.fromArray(vertices,
        (vertexIndices[next(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
  final Vector3 prevVertex = _prevVertex.fromArray(vertices,
        (vertexIndices[prev(coneVertexIdx, polygonVertexCount)] & 0x0fffffff) * 4);
  
  if (_leftOn(prevVertex, coneVertex, nextVertex)) {
    return _leftOn(coneVertex, testVertex, prevVertex) && _leftOn(testVertex, coneVertex, nextVertex);
  }
  return !(_leftOn(coneVertex, testVertex, nextVertex) && _leftOn(testVertex, coneVertex, prevVertex));
}

bool diagonalLoose(
  int startVertexIdx,
  int endVertexIdx,
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
) {
  return inConeLoose(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices) &&
         diagonalieLoose(startVertexIdx, endVertexIdx, polygonVertexCount, vertices, vertexIndices);
}

// Low allocation scratchpad variables mapped for loop ticks
final Vector3 _triangulateP0 = Vector3();
final Vector3 _triangulateP2 = Vector3();

int triangulate(
  int polygonVertexCount,
  List<double> vertices,
  List<int> vertexIndices,
  List<int> triangleIndices,
) {
  int ntris = 0;
  int dst = 0;
  
  // mark removable vertices
  for (int i = 0; i < polygonVertexCount; i++) {
    final int i1 = next(i, polygonVertexCount);
    final int i2 = next(i1, polygonVertexCount);
    if (diagonal(i, i2, polygonVertexCount, vertices, vertexIndices)) {
      vertexIndices[i1] |= 0x80000000;
    }
  }
  
  int nv = polygonVertexCount;
  while (nv > 3) {
    double minLen = -1;
    int mini = -1;
    
    for (int i = 0; i < nv; i++) {
      final int i1 = next(i, nv);
      if ((vertexIndices[i1] & 0x80000000) != 0) {
        final Vector3 p0 = _triangulateP0.fromArray(vertices, (vertexIndices[i] & 0x0fffffff) * 4);
        final Vector3 p2 = _triangulateP2.fromArray(vertices, (vertexIndices[next(i1, nv)] & 0x0fffffff) * 4);
        
        final double dx = p2.x - p0.x;
        final double dy = p2.z - p0.z; // mapping to Z component to match XZ evaluation rules
        final double len = dx * dx + dy * dy;
        
        if (minLen < 0 || len < minLen) {
          minLen = len;
          mini = i;
        }
      }
    }
    
    if (mini == -1) {
      // try loose diagonal test
      for (int i = 0; i < nv; i++) {
        final int i1 = next(i, nv);
        final int i2 = next(i1, nv);
        if (diagonalLoose(i, i2, nv, vertices, vertexIndices)) {
          final Vector3 p0 = _triangulateP0.fromArray(vertices, (vertexIndices[i] & 0x0fffffff) * 4);
          final Vector3 p2 = _triangulateP2.fromArray(vertices, (vertexIndices[next(i2, nv)] & 0x0fffffff) * 4);
          
          final double dx = p2.x - p0.x;
          final double dy = p2.z - p0.z;
          final double len = dx * dx + dy * dy;
          
          if (minLen < 0 || len < minLen) {
            minLen = len;
            mini = i;
          }
        }
      }
      if (mini == -1) {
        return -ntris;
      }
    }
    
    final int i = mini;
    int i1 = next(i, nv);
    final int i2 = next(i1, nv);
    
    triangleIndices[dst++] = vertexIndices[i] & 0x0fffffff;
    triangleIndices[dst++] = vertexIndices[i1] & 0x0fffffff;
    triangleIndices[dst++] = vertexIndices[i2] & 0x0fffffff;
    ntris++;
    
    // remove vertex
    nv--;
    for (int k = i1; k < nv; k++) {
      vertexIndices[k] = vertexIndices[k + 1];
    }
    if (i1 >= nv) i1 = 0;
    
    final int iPrev = prev(i1, nv);
    // update diagonal flags
    if (diagonal(prev(iPrev, nv), i1, nv, vertices, vertexIndices)) {
      vertexIndices[iPrev] |= 0x80000000;
    } else {
      vertexIndices[iPrev] &= 0x0fffffff;
    }
    
    if (diagonal(iPrev, next(i1, nv), nv, vertices, vertexIndices)) {
      vertexIndices[i1] |= 0x80000000;
    } else {
      vertexIndices[i1] &= 0x0fffffff;
    }
  }
  
  // final triangle elements assignment
  triangleIndices[dst++] = vertexIndices[0] & 0x0fffffff;
  triangleIndices[dst++] = vertexIndices[1] & 0x0fffffff;
  triangleIndices[dst++] = vertexIndices[2] & 0x0fffffff;
  ntris++;
  
  return ntris;
}

int countPolyVerts(List<int> polygons, int polyStartIdx, int maxVerticesPerPoly) {
  for (int i = 0; i < maxVerticesPerPoly; i++) {
    if (polygons[polyStartIdx + i] == meshNullIdx) return i;
  }
  return maxVerticesPerPoly;
}

bool uleft(List<double> firstVertex, List<double> secondVertex, List<double> testVertex) {
  return (
    (secondVertex[0] - firstVertex[0]) * (testVertex[2] - firstVertex[2]) - 
    (testVertex[0] - firstVertex[0]) * (secondVertex[2] - firstVertex[2]) < 0
  );
}

typedef PolyMergeResult = ({double value, int ea, int eb});

PolyMergeResult getPolyMergeValue(
  List<int> polygons,
  int polyAStartIdx,
  int polyBStartIdx,
  List<double> vertices,
  int maxVerticesPerPoly,
) {
  final int numVertsA = countPolyVerts(polygons, polyAStartIdx, maxVerticesPerPoly);
  final int numVertsB = countPolyVerts(polygons, polyBStartIdx, maxVerticesPerPoly);
  if (numVertsA + numVertsB - 2 > maxVerticesPerPoly) {
    return (value: -1.0, ea: -1, eb: -1);
  }
  int ea = -1;
  int eb = -1;
  for (int i = 0; i < numVertsA; i++) {
    int va0 = polygons[polyAStartIdx + i];
    int va1 = polygons[polyAStartIdx + ((i + 1) % numVertsA)];
    if (va0 > va1) { final int t = va0; va0 = va1; va1 = t; }
    for (int j = 0; j < numVertsB; j++) {
      int vb0 = polygons[polyBStartIdx + j];
      int vb1 = polygons[polyBStartIdx + ((j + 1) % numVertsB)];
      if (vb0 > vb1) { 
        final int t = vb0; 
        vb0 = vb1; 
        vb1 = t; 
      }
      if (va0 == vb0 && va1 == vb1) { 
        ea = i; 
        eb = j; 
        break; 
      }
    }
    //if (ea != -1) break;
  }
  if (ea == -1 || eb == -1){ 
    return (value: -1.0, ea: -1, eb: -1);
  }
  
  final int va = polygons[polyAStartIdx + ((ea + numVertsA - 1) % numVertsA)];
  final int vb = polygons[polyAStartIdx + ea];
  final int vc = polygons[polyBStartIdx + ((eb + 2) % numVertsB)];
  if (!uleft([vertices[va * 3], 0, vertices[va * 3 + 2]], [vertices[vb * 3], 0, vertices[vb * 3 + 2]], [vertices[vc * 3], 0, vertices[vc * 3 + 2]])) {
    return (value: -1.0, ea: -1, eb: -1);
  }
  final int va2 = polygons[polyBStartIdx + ((eb + numVertsB - 1) % numVertsB)];
  final int vb2 = polygons[polyBStartIdx + eb];
  final int vc2 = polygons[polyAStartIdx + ((ea + 2) % numVertsA)];
  if (!uleft([vertices[va2 * 3], 0, vertices[va2 * 3 + 2]], [vertices[vb2 * 3], 0, vertices[vb2 * 3 + 2]], [vertices[vc2 * 3], 0, vertices[vc2 * 3 + 2]])) {
    return (value: -1.0, ea: -1, eb: -1);
  }
  final int vaEdge = polygons[polyAStartIdx + ea];
  final int vbEdge = polygons[polyAStartIdx + ((ea + 1) % numVertsA)];
  final double dx = vertices[vaEdge * 3] - vertices[vbEdge * 3];
  final double dy = vertices[vaEdge * 3 + 2] - vertices[vbEdge * 3 + 2];
  return (value: dx * dx + dy * dy, ea: ea, eb: eb);
}

void mergePolyVerts(
  List<int> polygons,
  int polyAStartIdx,
  int polyBStartIdx,
  int edgeIdxA,
  int edgeIdxB,
  int tempStartIdx,
  int maxVerticesPerPoly,
) {
  final int numVertsA = countPolyVerts(polygons, polyAStartIdx, maxVerticesPerPoly);
  final int numVertsB = countPolyVerts(polygons, polyBStartIdx, maxVerticesPerPoly);
  for (int i = 0; i < maxVerticesPerPoly; i++) {
    polygons[tempStartIdx + i] = meshNullIdx;
  }
  int n = 0;
  for (int i = 0; i < numVertsA - 1; i++) {
    polygons[tempStartIdx + n++] = polygons[polyAStartIdx + ((edgeIdxA + 1 + i) % numVertsA)];
  }
  for (int i = 0; i < numVertsB - 1; i++) {
    polygons[tempStartIdx + n++] = polygons[polyBStartIdx + ((edgeIdxB + 1 + i) % numVertsB)];
  }
  for (int i = 0; i < maxVerticesPerPoly; i++) {
    polygons[polyAStartIdx + i] = polygons[tempStartIdx + i];
  }
}

PolyMesh buildPolyMesh(BuildContextState ctx, ContourSet contourSet, int maxVerticesPerPoly) {
  int maxVertsPerCont = 0;
  final List<Contour> contours = contourSet.contours;
  for (int i = 0; i < contours.length; i++) {
    final Contour contour = contours[i];
    if (contour.nVertices < 3) continue;
    maxVertsPerCont = math.max(maxVertsPerCont, contour.nVertices);
  }
  final PolyMesh mesh = PolyMesh(
    vertices: [], 
    polys: [], 
    regions: [],
    flags: [], 
    areas: [], 
    nVertices: 0, 
    nPolys: 0,
    maxVerticesPerPoly: maxVerticesPerPoly,
    bounds: contourSet.bounds.clone(), // three_js_math BoundingBox alignment
    localWidth: contourSet.width, 
    localHeight: contourSet.height,
    cellSize: contourSet.cellSize, 
    cellHeight: contourSet.cellHeight,
    borderSize: contourSet.borderSize, 
    maxEdgeError: contourSet.maxError,
  );

  final List<int> vflags = [];
  final Map<String, List<VertexEntry>> vertexMap = {};
  final List<int> indices = List<int>.filled(maxVertsPerCont, 0);
  final List<int> tris = List<int>.filled(maxVertsPerCont * 3, 0);
  final List<int> polys = List<int>.filled((maxVertsPerCont + 1) * maxVerticesPerPoly, meshNullIdx);
  final int tmpPolyStart = maxVertsPerCont * maxVerticesPerPoly;

  // Continue buildPolyMesh loop context
  for (int i = 0; i < contours.length; i++) {
    final Contour cont = contours[i];
    final int contNVertices = cont.nVertices;
    if (contNVertices < 3) continue;
    for (int j = 0; j < contNVertices; j++) { 
      indices[j] = j; 
    }

    final contVertices = cont.vertices;
    int ntris = triangulate(contNVertices, contVertices, indices, tris);
    if (ntris <= 0) {
      print('Bad triangulation for contour $i');
      ntris = ntris.abs();
    }
    for (int j = 0; j < contNVertices; j++) {
      final double x = contVertices[j * 4];
      final double y = contVertices[j * 4 + 1];
      final double z = contVertices[j * 4 + 2];
      final int flags = contVertices[j * 4 + 3].toInt();
      final List<double> meshVertices = mesh.vertices;
      final int idx = addVertex(x, y, z, meshVertices, vflags, vertexMap);
      indices[j] = idx;
      if ((flags & borderVertex) != 0) { vflags[idx] = 1; }
    }
    // Continue loop context inside buildPolyMesh
    int npolys = 0;
    polys.fillRange(0, maxVertsPerCont * maxVerticesPerPoly, meshNullIdx);
    for (int j = 0; j < ntris; j++) {
      final int t0 = tris[j * 3]; final int t1 = tris[j * 3 + 1]; final int t2 = tris[j * 3 + 2];
      if (t0 != t1 && t0 != t2 && t1 != t2) {
        polys[npolys * maxVerticesPerPoly] = indices[t0];
        polys[npolys * maxVerticesPerPoly + 1] = indices[t1];
        polys[npolys * maxVerticesPerPoly + 2] = indices[t2];
        npolys++;
      }
    }
    if (npolys == 0) continue;
    // Continue loop context inside buildPolyMesh
    if (maxVerticesPerPoly > 3) {
      while (true) {
        double bestMergeVal = 0; int bestPa = 0; int bestPb = 0; int bestEa = 0; int bestEb = 0;
        for (int j = 0; j < npolys - 1; j++) {
          for (int k = j + 1; k < npolys; k++) {
            final int paStart = j * maxVerticesPerPoly; final int pbStart = k * maxVerticesPerPoly;
            final List<double> meshVertices = mesh.vertices;
            final result = getPolyMergeValue(polys, paStart, pbStart, meshVertices, maxVerticesPerPoly);
            if (result.value > bestMergeVal) {
              bestMergeVal = result.value; bestPa = j; bestPb = k; bestEa = result.ea; bestEb = result.eb;
            }
          }
        }
        if (bestMergeVal > 0) {
          final int paStart = bestPa * maxVerticesPerPoly; final int pbStart = bestPb * maxVerticesPerPoly;
          mergePolyVerts(polys, paStart, pbStart, bestEa, bestEb, tmpPolyStart, maxVerticesPerPoly);
          for (int m = 0; m < maxVerticesPerPoly; m++) { 
            polys[pbStart + m] = polys[(npolys - 1) * maxVerticesPerPoly + m]; 
          }
          npolys--;
        } else { break; }
      }
    }
    // Concluding section of buildPolyMesh
    final List<int> meshPolys = mesh.polys;
    final meshRegions = mesh.regions;
    final List<int> meshAreas = mesh.areas;
    for (int j = 0; j < npolys; j++) {
      final int pStart = (mesh.nPolys) * maxVerticesPerPoly;
      final int qStart = j * maxVerticesPerPoly;
      if (meshPolys.length <= pStart + maxVerticesPerPoly) {
        meshPolys.addAll(List<int>.filled((pStart + maxVerticesPerPoly + 1) - meshPolys.length, 0));
      }
      for (int k = 0; k < maxVerticesPerPoly; k++) {
        meshPolys[pStart + k] = polys[qStart + k]; 
      }
      if (meshRegions.length <= mesh.nPolys) { 
        meshRegions.add(0); 
        meshAreas.add(0); 
      }
      meshRegions[mesh.nPolys] = cont.reg;
      meshAreas[mesh.nPolys] = cont.area;
      mesh.nPolys++;
    }
  }
  final List<double> meshVertices = mesh.vertices;
  mesh.nVertices =( meshVertices.length / 3).floor();
  for (int i = 0; i < (mesh.nVertices); i++) {
    if (vflags[i] != 0) {
      if (!canRemoveVertex(mesh, i)){
        continue;
      }
      if (removeVertex(ctx, mesh, i)) {
        final int currentNVerts = mesh.nVertices;
        for (int j = i; j < currentNVerts; j++) { 
          vflags[j] = vflags[j + 1]; 
        }
        i--;
      } else { 
        print('Failed to remove edge vertex $i'); 
      }
    }
  }
  final int finalNPolys = mesh.nPolys;
  final int finalNVerts = mesh.nVertices;
  if (mesh.polys.length > finalNPolys * maxVerticesPerPoly) mesh.polys.removeRange(finalNPolys * maxVerticesPerPoly, mesh.polys.length);
  if (mesh.regions.length > finalNPolys) mesh.regions.removeRange(finalNPolys, mesh.regions.length);
  if (meshVertices.length > finalNVerts * 3) meshVertices.removeRange(finalNVerts * 3, meshVertices.length);
  mesh.flags = List<int>.filled(finalNPolys, 0);
  
  return mesh;
}

/// Checks if a vertex can be safely removed without violating polygon constraints
bool canRemoveVertex(PolyMesh mesh, int remVertexIdx) {
  final int nvp = mesh.maxVerticesPerPoly;
  final int nPolys = mesh.nPolys;
  final List<int> meshPolys = mesh.polys;

  // Count number of polygons to remove
  int numTouchedVerts = 0;
  int numRemainingEdges = 0;

  for (int i = 0; i < nPolys; i++) {
    final int polyStart = i * nvp;
    final int nv = countPolyVerts(meshPolys, polyStart, nvp);
    int numRemoved = 0;
    int numVerts = 0;

    for (int j = 0; j < nv; j++) {
      if (meshPolys[polyStart + j] == remVertexIdx) {
        numTouchedVerts++;
        numRemoved++;
      }
      numVerts++;
    }

    if (numRemoved > 0) {
      numRemainingEdges += numVerts - (numRemoved + 1);
    }
  }

  // There would be too few edges remaining to create a valid polygon
  if (numRemainingEdges <= 2) {
    return false;
  }

  // Find edges which share the removed vertex
  final int maxEdges = numTouchedVerts * 2;
  final List<int> edges = List<int>.filled(maxEdges * 3, 0);
  int nedges = 0;

  for (int i = 0; i < nPolys; i++) {
    final int polyStart = i * nvp;
    final int nv = countPolyVerts(meshPolys, polyStart, nvp);

    // Collect edges which touch the removed vertex
    for (int j = 0, k = nv - 1; j < nv; k = j++) {
      if (meshPolys[polyStart + j] == remVertexIdx || meshPolys[polyStart + k] == remVertexIdx) {
        // Arrange edge so that a = remVertexIdx
        int a = meshPolys[polyStart + j];
        int b = meshPolys[polyStart + k];
        
        if (b == remVertexIdx) {
          final int tmp = a;
          a = b;
          b = tmp;
        }

        // Check if the edge exists
        bool exists = false;
        for (int m = 0; m < nedges; m++) {
          final int e = m * 3;
          if (edges[e + 1] == b) {
            // Exists, increment vertex share count
            edges[e + 2]++;
            exists = true;
            break;
          }
        }

        // Add new edge
        if (!exists) {
          final int e = nedges * 3;
          edges[e] = a;
          edges[e + 1] = b;
          edges[e + 2] = 1;
          nedges++;
        }
      }
    }
  }

  // There should be no more than 2 open edges
  int numOpenEdges = 0;
  for (int i = 0; i < nedges; i++) {
    if (edges[i * 3 + 2] < 2) {
      numOpenEdges++;
    }
  }

  return numOpenEdges <= 2;
}

/// Helper function to shift elements and insert at the front of a list tracking its size by map reference
void pushFront(int v, List<int> arr, Nint an) {
  an.value++;
  for (int i = an.value - 1; i > 0; i--) {
    arr[i] = arr[i - 1];
  }
  arr[0] = v;
}

/// Helper function to append elements to a list tracking its size by map reference
void pushBack(int v, List<int> arr, Nint an) {
  arr[an.value] = v;
  an.value++;
}

/// Unlinks a vertex entry from the poly mesh, builds an ordered perimeter hole, and re-triangulates the region
bool removeVertex(BuildContextState ctx, PolyMesh mesh, int remVertexIdx) {
  final int nvp = mesh.maxVerticesPerPoly;
  final List<int> meshPolys = mesh.polys;
  final List<int> meshRegions = mesh.regions;
  final List<int> meshAreas = mesh.areas;
  final List<double> meshVertices = mesh.vertices;

  int numRemovedVerts = 0;
  for (int i = 0; i < (mesh.nPolys); i++) {
    final int polyStart = i * nvp;
    final int nv = countPolyVerts(meshPolys, polyStart, nvp);
    for (int j = 0; j < nv; j++) {
      if (meshPolys[polyStart + j] == remVertexIdx) { numRemovedVerts++; }
    }
  }

  final List<int> edges = List<int>.filled(numRemovedVerts * nvp * 4, 0);
  int nedges = 0;
  final List<int> hole = List<int>.filled(numRemovedVerts * nvp, 0);
  int nhole = 0;
  final List<int> hreg = List<int>.filled(numRemovedVerts * nvp, 0);
  final List<int> harea = List<int>.filled(numRemovedVerts * nvp, 0);

  for (int i = 0; i < (mesh.nPolys); i++) {
    final int polyStart = i * nvp;
    final int nv = countPolyVerts(meshPolys, polyStart, nvp);
    bool hasRem = false;
    for (int j = 0; j < nv; j++) {
      if (meshPolys[polyStart + j] == remVertexIdx) { hasRem = true; break; }
    }
    if (hasRem) {
      for (int j = 0, k = nv - 1; j < nv; k = j++) {
        if (meshPolys[polyStart + j] != remVertexIdx && meshPolys[polyStart + k] != remVertexIdx) {
          final int e = nedges * 4;
          edges[e] = meshPolys[polyStart + k];
          edges[e + 1] = meshPolys[polyStart + j];
          edges[e + 2] = meshRegions[i];
          edges[e + 3] = meshAreas[i];
          nedges++;
        }
      }
      final int lastPolyStart = ((mesh.nPolys) - 1) * nvp;
      if (polyStart != lastPolyStart) {
        for (int j = 0; j < nvp; j++) { meshPolys[polyStart + j] = meshPolys[lastPolyStart + j]; }
      }
      for (int j = 0; j < nvp; j++) { meshPolys[lastPolyStart + j] = meshNullIdx; }
      meshRegions[i] = meshRegions[(mesh.nPolys) - 1];
      meshAreas[i] = meshAreas[(mesh.nPolys) - 1];
      mesh.nPolys--;
      i--;
    }
  }

  final int meshNVertices = mesh.nVertices;
  for (int i = remVertexIdx; i < meshNVertices - 1; i++) {
    meshVertices[i * 3] = meshVertices[(i + 1) * 3];
    meshVertices[i * 3 + 1] = meshVertices[(i + 1) * 3 + 1];
    meshVertices[i * 3 + 2] = meshVertices[(i + 1) * 3 + 2];
  }
  mesh.nVertices--;

  for (int i = 0; i < (mesh.nPolys); i++) {
    final int polyStart = i * nvp;
    final int nv = countPolyVerts(meshPolys, polyStart, nvp);
    for (int j = 0; j < nv; j++) {
      if (meshPolys[polyStart + j] > remVertexIdx) { meshPolys[polyStart + j]--; }
    }
  }
  for (int i = 0; i < nedges; i++) {
    if (edges[i * 4] > remVertexIdx) edges[i * 4]--;
    if (edges[i * 4 + 1] > remVertexIdx) edges[i * 4 + 1]--;
  }

  if (nedges == 0) return true;

  Nint nholeRef = Nint();
  Nint nhregRef = Nint();
  Nint nhareaRef = Nint();

  pushBack(edges[0], hole, nholeRef);
  pushBack(edges[2], hreg, nhregRef);
  pushBack(edges[3], harea, nhareaRef);
  nhole = nholeRef.value;

  while (nedges > 0) {
    bool match = false;
    for (int i = 0; i < nedges; i++) {
      final int ea = edges[i * 4]; 
      final int eb = edges[i * 4 + 1];
      final int r = edges[i * 4 + 2]; 
      final int a = edges[i * 4 + 3];
      bool add = false;
       
      if (hole[0] == eb) {
        pushFront(ea, hole, nholeRef); 
        pushFront(r, hreg, nhregRef); 
        pushFront(a, harea, nhareaRef);
        add = true;
      } 
      else if (hole[nhole - 1] == ea) {
        pushBack(eb, hole, nholeRef); 
        pushBack(r, hreg, nhregRef); 
        pushBack(a, harea, nhareaRef);
        add = true;
      }
      if (add) {
        final int targetIdx = (nedges - 1) * 4;
        edges[i * 4] = edges[targetIdx]; 
        edges[i * 4 + 1] = edges[targetIdx + 1];
        edges[i * 4 + 2] = edges[targetIdx + 2]; 
        edges[i * 4 + 3] = edges[targetIdx + 3];
        nedges--; 
        match = true; 
        i--;
      }
      nhole = nholeRef.value;
    }
    if (!match){ 
      break;
    }
  }
  // Continue removeVertex context block
  final List<int> tris = List<int>.filled(nhole * 3, 0);
  final List<double> tverts = List<double>.filled(nhole * 4, 0.0);
  final List<int> thole = List<int>.filled(nhole, 0);

  for (int i = 0; i < nhole; i++) {
    final int pi = hole[i];
    tverts[i * 4] = meshVertices[pi * 3];
    tverts[i * 4 + 1] = meshVertices[pi * 3 + 1];
    tverts[i * 4 + 2] = meshVertices[pi * 3 + 2];
    tverts[i * 4 + 3] = 0.0;
    thole[i] = i;
  }

  int ntris = triangulate(nhole, tverts, thole, tris);
  if (ntris < 0) {
    ntris = -ntris;
    print('removeVertex: triangulate() returned bad results');
  }

  final List<int> polys = List<int>.filled((ntris + 1) * nvp, meshNullIdx);
  final List<int> pregs = List<int>.filled(ntris, 0);
  final List<int> pareas = List<int>.filled(ntris, 0);

  int npolys = 0;
  for (int j = 0; j < ntris; j++) {
    final int t0 = tris[j * 3]; final int t1 = tris[j * 3 + 1]; final int t2 = tris[j * 3 + 2];
    if (t0 != t1 && t0 != t2 && t1 != t2) {
      polys[npolys * nvp] = hole[t0];
      polys[npolys * nvp + 1] = hole[t1];
      polys[npolys * nvp + 2] = hole[t2];
      
      if (hreg[t0] != hreg[t1] || hreg[t1] != hreg[t2]) {
        pregs[npolys] = multipleRegs; // Mapped from previously initialized project global constants
      } else {
        pregs[npolys] = hreg[t0];
      }
      pareas[npolys] = harea[t0];
      npolys++;
    }
  }

  if (npolys == 0) return true;

  if (nvp > 3) {
    while (true) {
      double bestMergeVal = 0; 
      int bestPa = 0; 
      int bestPb = 0; 
      int bestEa = 0; 
      int bestEb = 0;

      for (int j = 0; j < npolys - 1; j++) {
        final int pjStart = j * nvp;
        for (int k = j + 1; k < npolys; k++) {
          final int pkStart = k * nvp;
          final result = getPolyMergeValue(polys, pjStart, pkStart, meshVertices, nvp);
          if (result.value > bestMergeVal) {
            bestMergeVal = result.value; 
            bestPa = j; 
            bestPb = k; 
            bestEa = result.ea; 
            bestEb = result.eb;
          }
        }
      }
      if (bestMergeVal > 0) {
        final int paStart = bestPa * nvp; 
        final int pbStart = bestPb * nvp;
        mergePolyVerts(polys, paStart, pbStart, bestEa, bestEb, ntris * nvp, nvp);
        if (pregs[bestPa] != pregs[bestPb]) { 
          pregs[bestPa] = multipleRegs; 
        }

        final int lastStart = (npolys - 1) * nvp;
        if (pbStart != lastStart) {
          for (int m = 0; m < nvp; m++) { 
            polys[pbStart + m] = polys[lastStart + m]; 
          }
        }
        pregs[bestPb] = pregs[npolys - 1];
        pareas[bestPb] = pareas[npolys - 1];
        npolys--;
      } else { break; }
    }
  }

  for (int i = 0; i < npolys; i++) {
    final int meshPolyStart = (mesh.nPolys) * nvp;
    final int polyStart = i * nvp;
    
    if (meshPolys.length <= meshPolyStart + nvp) {
      meshPolys.addAll(List<int>.filled((meshPolyStart + nvp + 1) - meshPolys.length, 0));
    }
    for (int j = 0; j < nvp; j++) { 
      meshPolys[meshPolyStart + j] = meshNullIdx; 
    }
    for (int j = 0; j < nvp; j++) { 
      meshPolys[meshPolyStart + j] = polys[polyStart + j]; 
    }
    
    if (meshRegions.length <= (mesh.nPolys)) {
      meshRegions.add(0); 
      meshAreas.add(0);
    }

    meshRegions[mesh.nPolys] = pregs[i];
    meshAreas[mesh.nPolys] = pareas[i];
    mesh.nPolys++;
  }
  return true;
}
