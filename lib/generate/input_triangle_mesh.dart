import 'dart:math' as math;
import 'package:three_js_math/three_js_math.dart';
import './index.dart';

// Assuming these types and constants are defined in your 'common.dart'
// and math library equivalents.
// import 'common.dart'; 
// import 'mathcat.dart'; // Replace with your actual vector math imports

// Replicating mathcat buffer utilities
class Vector3Utils {
  static Vector3 create() => Vector3();
  
  static void subtract(Vector3 out, Vector3 a, Vector3 b) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
  }
  
  static void cross(Vector3 out, Vector3 a, Vector3 b) {
    final ax = a[0], ay = a[1], az = a[2];
    final bx = b[0], by = b[1], bz = b[2];
    out[0] = ay * bz - az * by;
    out[1] = az * bx - ax * bz;
    out[2] = ax * by - ay * bx;
  }
  
  static void normalize(Vector3 out, Vector3 v) {
    final x = v[0], y = v[1], z = v[2];
    double len = x * x + y * y + z * z;
    if (len > 0) {
      len = 1.0 / math.sqrt(len);
      out[0] = v[0] * len;
      out[1] = v[1] * len;
      out[2] = v[2] * len;
    }
  }

  static Vector3 fromBuffer(Vector3 out, List<num> buffer, int offset) {
    out[0] = buffer[offset].toDouble();
    out[1] = buffer[offset + 1].toDouble();
    out[2] = buffer[offset + 2].toDouble();
    return out;
  }
}

// File-level private scratch variables to avoid GC pressure
final Vector3 _edge0 = Vector3Utils.create();
final Vector3 _edge1 = Vector3Utils.create();
final Vector3 _triangleNormal = Vector3Utils.create();
final Vector3 _v0 = Vector3Utils.create();
final Vector3 _v1 = Vector3Utils.create();
final Vector3 _v2 = Vector3Utils.create();

/// Calculates the normal vector of a triangle
void calcTriNormal(Vector3 inV0, Vector3 inV1, Vector3 inV2, Vector3 outFaceNormal) {
  // Calculate edge vectors: e0 = v1 - v0, e1 = v2 - v0
  Vector3Utils.subtract(_edge0, inV1, inV0);
  Vector3Utils.subtract(_edge1, inV2, inV0);
  
  // Calculate cross product: faceNormal = e0 × e1
  Vector3Utils.cross(outFaceNormal, _edge0, _edge1);
  
  // Normalize the result
  Vector3Utils.normalize(outFaceNormal, outFaceNormal);
}

/// Marks triangles as walkable based on their slope angle
void markWalkableTriangles(
  List<num> inVertices,
  List<int> inIndices,
  List<int> outTriAreaIds, [
  double walkableSlopeAngle = 45.0,
]) {
  // Convert walkable slope angle to threshold using cosine
  final walkableThr = math.cos((walkableSlopeAngle / 180.0) * math.pi);
  final numTris = inIndices.length ~/ 3; // Using integer division

  for (int i = 0; i < numTris; ++i) {
    final triStartIndex = i * 3;
    final i0 = inIndices[triStartIndex];
    final i1 = inIndices[triStartIndex + 1];
    final i2 = inIndices[triStartIndex + 2];

    final v0 = Vector3Utils.fromBuffer(_v0, inVertices, i0 * 3);
    final v1 = Vector3Utils.fromBuffer(_v1, inVertices, i1 * 3);
    final v2 = Vector3Utils.fromBuffer(_v2, inVertices, i2 * 3);

    calcTriNormal(v0, v1, v2, _triangleNormal);

    if (_triangleNormal[1] > walkableThr) {
      outTriAreaIds[i] = walkableArea;
    }
  }
}

/// Clears (sets to NULL_AREA) triangles whose slope exceeds the walkable limit.
void clearUnwalkableTriangles(
  List<num> inVertices,
  List<int> inIndices,
  List<int> inOutTriAreaIds, [
  double walkableSlopeAngle = 45.0,
]) {
  final walkableThr = math.cos((walkableSlopeAngle / 180.0) * math.pi);
  final numTris = inIndices.length ~/ 3;

  for (int i = 0; i < numTris; ++i) {
    final triStartIndex = i * 3;
    final i0 = inIndices[triStartIndex];
    final i1 = inIndices[triStartIndex + 1];
    final i2 = inIndices[triStartIndex + 2];

    final v0 = Vector3Utils.fromBuffer(_v0, inVertices, i0 * 3);
    final v1 = Vector3Utils.fromBuffer(_v1, inVertices, i1 * 3);
    final v2 = Vector3Utils.fromBuffer(_v2, inVertices, i2 * 3);

    calcTriNormal(v0, v1, v2, _triangleNormal);

    if (_triangleNormal[1] <= walkableThr) {
      inOutTriAreaIds[i] = nullArea;
    }
  }
}

/// Calculates global bounds of the provided mesh structure
BoundingBox calculateMeshBounds(BoundingBox outBounds, List<num> inVertices, List<int> inIndices) {
  final safeMax = double.infinity;
  outBounds.min.x = safeMax;
  outBounds.min.y = safeMax;
  outBounds.min.z = safeMax;
  outBounds.max.x = -safeMax;
  outBounds.max.y = -safeMax;
  outBounds.max.z = -safeMax;

  final numTris = inIndices.length ~/ 3;

  for (int i = 0; i < numTris; ++i) {
    final triStartIndex = i * 3;
    for (int j = 0; j < 3; ++j) {
      final index = inIndices[triStartIndex + j];
      final x = inVertices[index * 3].toDouble();
      final y = inVertices[index * 3 + 1].toDouble();
      final z = inVertices[index * 3 + 2].toDouble();

      outBounds.min.x = math.min(outBounds.min.x, x);
      outBounds.min.y = math.min(outBounds.min.y, y);
      outBounds.min.z = math.min(outBounds.min.z, z);
      outBounds.max.x = math.max(outBounds.max.x, x);
      outBounds.max.y = math.max(outBounds.max.y, y);
      outBounds.max.z = math.max(outBounds.max.z, z);
    }
  }

  return outBounds;
}
