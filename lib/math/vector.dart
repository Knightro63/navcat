import 'package:three_js_math/three_js_math.dart';
import 'dart:math' as math;

extension Vec3Cat on Vector3 {
  /// Returns true if all three floating-point spatial components are finite.
  bool isFinite() {
    return x.isFinite && y.isFinite && z.isFinite;
  }

  void min2(Vector3 a, Vector3 b) {
    x = math.min(a.x, b.x);
    y = math.min(a.y, b.y);
    z = math.min(a.z, b.z);
  }

  void max2(Vector3 a, Vector3 b) {
    x = math.max(a.x, b.x);
    y = math.max(a.y, b.y);
    z = math.max(a.z, b.z);
  }
}

extension Vector2FiniteExtension on Vector2 {
  /// Returns true if all three floating-point spatial components are finite.
  bool isFinite() {
    return x.isFinite && y.isFinite;
  }
}