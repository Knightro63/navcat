import 'dart:math' as math;
import 'dart:typed_data';
import 'package:three_js_math/three_js_math.dart';

const double DT_PI = math.pi;
const int DT_MAX_PATTERN_DIVS = 32; // Max number of adaptive divs
const int DT_MAX_PATTERN_RINGS = 4; // Max number of adaptive rings

class ObstacleCircle {
  /// Position of the obstacle
  final Vector3 p;
  /// Velocity of the obstacle
  final Vector3 vel;
  /// Desired velocity of the obstacle
  final Vector3 dvel;
  /// Radius of the obstacle
  double rad;
  /// Direction to obstacle center (used for side selection during sampling)
  final Vector3 dp;
  /// Normal pointing away from velocity (used for side selection during sampling)
  final Vector3 np;

  ObstacleCircle({
    Vector3? p,
    Vector3? vel,
    Vector3? dvel,
    this.rad = 0.0,
    Vector3? dp,
    Vector3? np,
  })  : this.p = p ?? Vector3(),
        this.vel = vel ?? Vector3(),
        this.dvel = dvel ?? Vector3(),
        this.dp = dp ?? Vector3(),
        this.np = np ?? Vector3();
}

class ObstacleSegment {
  /// End points of the obstacle segment
  final Vector3 p;
  final Vector3 q;
  /// True if agent is very close to segment
  bool touch;

  ObstacleSegment({
    Vector3? p,
    Vector3? q,
    this.touch = false,
  })  : this.p = p ?? Vector3(),
        this.q = q ?? Vector3();
}

class ObstacleAvoidanceParams {
  double velBias;
  double weightDesVel;
  double weightCurVel;
  double weightSide;
  double weightToi;
  double horizTime;
  int gridSize;          // for grid sampling
  int adaptiveDivs;      // for adaptive sampling
  int adaptiveRings;     // for adaptive sampling
  int adaptiveDepth;     // for adaptive sampling

  ObstacleAvoidanceParams({
    this.velBias = 0.4,
    this.weightDesVel = 2.0,
    this.weightCurVel = 0.75,
    this.weightSide = 0.75,
    this.weightToi = 2.5,
    this.horizTime = 2.5,
    this.gridSize = 33,
    this.adaptiveDivs = 7,
    this.adaptiveRings = 2,
    this.adaptiveDepth = 5,
  });
}

class ObstacleAvoidanceSample {
  final Vector3 vel;
  double ssize;
  double pen;
  double vpen;
  double vcpen;
  double spen;
  double tpen;

  ObstacleAvoidanceSample({
    Vector3? vel,
    this.ssize = 0.0,
    this.pen = 0.0,
    this.vpen = 0.0,
    this.vcpen = 0.0,
    this.spen = 0.0,
    this.tpen = 0.0,
  }) : this.vel = vel ?? Vector3();
}

class ObstacleAvoidanceDebugData {
  final List<ObstacleAvoidanceSample> samples;

  ObstacleAvoidanceDebugData({List<ObstacleAvoidanceSample>? samples})
      : this.samples = samples ?? [];

  /// Resets debug data.
  void reset() {
    samples.clear();
  }
}

class ObstacleAvoidanceQuery {
  final List<ObstacleCircle> circles;
  final List<ObstacleSegment> segments;
  final int maxCircles;
  final int maxSegments;
  int circleCount;
  int segmentCount;

  // Internal state for sampling
  final ObstacleAvoidanceParams params;
  double invHorizTime;
  double vmax;
  double invVmax;

  // Pre-allocated pattern array for adaptive sampling
  final Float32List pattern;

  ObstacleAvoidanceQuery({
    required this.maxCircles,
    required this.maxSegments,
  })  : this.circles = List<ObstacleCircle>.generate(maxCircles, (_) => ObstacleCircle()),
        this.segments = List<ObstacleSegment>.generate(maxSegments, (_) => ObstacleSegment()),
        this.circleCount = 0,
        this.segmentCount = 0,
        this.params = ObstacleAvoidanceParams(),
        this.invHorizTime = 0.0,
        this.vmax = 0.0,
        this.invVmax = 0.0,
        this.pattern = Float32List((DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2);

  /// Resets the obstacle avoidance query counters.
  void reset() {
    circleCount = 0;
    segmentCount = 0;
  }

  /// Adds a circular obstacle to the query.
  void addCircleObstacle(Vector3 pos, double rad, Vector3 vel, Vector3 dvel) {
    if (circleCount >= maxCircles) return;

    final circle = circles[circleCount];
    // Copy data to pre-allocated object
    circle.p.setFrom(pos);
    circle.vel.setFrom(vel);
    circle.dvel.setFrom(dvel);
    circle.rad = rad;

    // Reset computed values
    circle.dp.setValues(0, 0, 0);
    circle.np.setValues(0, 0, 0);
    circleCount++;
  }

  /// Adds a segment obstacle to the query.
  void addSegmentObstacle(Vector3 p, Vector3 q) {
    if (segmentCount >= maxSegments) return;

    final segment = segments[segmentCount];
    // Copy data to pre-allocated object
    segment.p.setFrom(p);
    segment.q.setFrom(q);
    segment.touch = false;
    segmentCount++;
  }
}

/**
 * Helper function to calculate 2D triangle area on XZ plane.
 */
double triArea2D(Vector3 a, Vector3 b, Vector3 c) {
  final abx = b.x - a.x;
  final abz = b.z - a.z;
  final acx = c.x - a.x;
  final acz = c.z - a.z;
  return acx * abz - abx * acz;
}

/**
 * Helper function to calculate 2D dot product on XZ plane.
 */
double vdot2D(Vector3 a, Vector3 b) => (a.x * b.x) + (a.z * b.z);

/**
 * Helper function to calculate 2D perpendicular dot product on XZ plane.
 */
double vperp2D(Vector3 a, Vector3 b) => (a.x * b.z) - (a.z * b.x);

/**
 * Helper function to calculate 2D distance on XZ plane.
 */
double vdist2D(Vector3 a, Vector3 b) {
  final dx = b.x - a.x;
  final dz = b.z - a.z;
  return math.sqrt(dx * dx + dz * dz);
}

/**
 * Helper function to calculate squared distance from point to segment in 2D (XZ plane).
 */
double distancePtSegSqr2D(Vector3 pt, Vector3 p, Vector3 q) {
  final pqx = q.x - p.x;
  final pqz = q.z - p.z;
  final dx = pt.x - p.x;
  final dz = pt.z - p.z;
  final d = (pqx * pqx) + (pqz * pqz);
  
  double t = (pqx * dx) + (pqz * dz);
  if (d > 0) t /= d;
  if (t < 0) {
    t = 0;
  } else if (t > 1) {
    t = 1;
  }

  final nearestX = p.x + (t * pqx);
  final nearestZ = p.z + (t * pqz);
  final distX = pt.x - nearestX;
  final distZ = pt.z - nearestZ;
  
  return (distX * distX) + (distZ * distZ);
}

// Internal temporary vector variable allocation state for sweep tests
final Vector3 _sweepCircleCircleS = Vector3();

/// Reusable container tracking sweep test intersection outputs.
class SweepResult {
  bool hit = false;
  double tmin = 0.0;
  double tmax = 0.0;
}

/// Reusable container tracking ray-segment intersection outputs.
class IntersectionResult {
  bool hit = false;
  double t = 0.0;
}

/**
 * Sweep test between two circles.
 */
void sweepCircleCircle(
  Vector3 c0,
  double r0,
  Vector3 v,
  Vector3 c1,
  double r1,
  SweepResult out,
) {
  const double EPS = 0.0001;
  final double sx = c1.x - c0.x;
  final double sz = c1.z - c0.z;
  
  // Reusing internal global scratchpad from previous turn definition
  _sweepCircleCircleS.setValues(sx, 0, sz);
  
  final double r = r0 + r1;
  final double sSqr = (sx * sx) + (sz * sz);
  final double c = sSqr - (r * r);
  
  final double a = (v.x * v.x) + (v.z * v.z);
  if (a < EPS) {
    out.hit = false;
    out.tmin = 0.0;
    out.tmax = 0.0;
    return;
  }

  final double b = (v.x * sx) + (v.z * sz);
  final double d = (b * b) - (a * c);
  if (d < 0.0) {
    out.hit = false;
    out.tmin = 0.0;
    out.tmax = 0.0;
    return;
  }

  final double invA = 1.0 / a;
  final double rd = math.sqrt(d);
  out.hit = true;
  out.tmin = (b - rd) * invA;
  out.tmax = (b + rd) * invA;
}

final Vector3 _intersectRaySegmentV = Vector3();
final Vector3 _intersectRaySegmentW = Vector3();

/**
 * Ray-segment intersection test.
 */
void intersectRaySegment(
  Vector3 ap,
  Vector3 u,
  Vector3 bp,
  Vector3 bq,
  IntersectionResult out,
) {
  final v = _intersectRaySegmentV..setValues(bq.x - bp.x, bq.y - bp.y, bq.z - bp.z);
  final w = _intersectRaySegmentW..setValues(ap.x - bp.x, ap.y - bp.y, ap.z - bp.z);

  final double d = vperp2D(u, v);
  if (d.abs() < 1e-6) {
    out.hit = false;
    out.t = 0.0;
    return;
  }

  final double invD = 1.0 / d;
  final double t = vperp2D(v, w) * invD;
  if (t < 0 || t > 1) {
    out.hit = false;
    out.t = 0.0;
    return;
  }

  final double s = vperp2D(u, w) * invD;
  if (s < 0 || s > 1) {
    out.hit = false;
    out.t = 0.0;
    return;
  }

  out.hit = true;
  out.t = t;
}

final Vector3 _prepareObstaclesOrig = Vector3();
final Vector3 _prepareObstaclesDv = Vector3();

/**
 * Prepares obstacles for sampling by calculating side information.
 */
void prepareObstacles(ObstacleAvoidanceQuery query, Vector3 pos, Vector3 dvel) {
  // Prepare circular obstacles
  for (int i = 0; i < query.circleCount; i++) {
    final cir = query.circles[i];
    final pa = pos;
    final pb = cir.p;

    final orig = _prepareObstaclesOrig.setValues( 0, 0, 0);
    cir.dp.sub2(pb, pa);
    cir.dp.normalize();
    
    final dv = _prepareObstaclesDv.sub2(cir.dvel, dvel);

    final double a = triArea2D(orig, cir.dp, dv);

    if (a < 0.01) {
      cir.np.setValues(-cir.dp.z, 0, cir.dp.x);
    } else {
      cir.np.setValues(cir.dp.z, 0, -cir.dp.x);
    }
  }

  // Prepare segment obstacles
  for (int i = 0; i < query.segmentCount; i++) {
    final seg = query.segments[i];
    const double r = 0.01;
    final double distSqr = distancePtSegSqr2D(pos, seg.p, seg.q);
    seg.touch = distSqr < (r * r);
  }
}

/**
 * Copies parameters to avoid object allocation.
 */
void copyParams(ObstacleAvoidanceParams dest, ObstacleAvoidanceParams src) {
  dest.velBias = src.velBias;
  dest.weightDesVel = src.weightDesVel;
  dest.weightCurVel = src.weightCurVel;
  dest.weightSide = src.weightSide;
  dest.weightToi = src.weightToi;
  dest.horizTime = src.horizTime;
  dest.gridSize = src.gridSize;
  dest.adaptiveDivs = src.adaptiveDivs;
  dest.adaptiveRings = src.adaptiveRings;
  dest.adaptiveDepth = src.adaptiveDepth;
}

// Zero-allocation reusable variable vectors for sampling functions
final Vector3 _vab = Vector3();
final Vector3 _sdir = Vector3();
final Vector3 _snorm = Vector3();
final SweepResult _sweepResult = SweepResult();
final IntersectionResult _intersectionResult = IntersectionResult();

/**
 * Process a velocity sample and calculate its penalty.
 */
double processSample(
  ObstacleAvoidanceQuery query,
  Vector3 vcand,
  double cs,
  Vector3 pos,
  double rad,
  Vector3 vel,
  Vector3 dvel,
  double minPenalty, [
  ObstacleAvoidanceDebugData? debug,
]) {
  final params = query.params;

  // Penalty for straying away from desired and current velocities
  final double vpen = params.weightDesVel * (vdist2D(vcand, dvel) * query.invVmax);
  final double vcpen = params.weightCurVel * (vdist2D(vcand, vel) * query.invVmax);

  // Find threshold hit time to bail out based on early out penalty
  final double minPen = minPenalty - vpen - vcpen;
  final double tThreshold = ((params.weightToi / minPen) - 0.1) * params.horizTime;

  // Dart equivalent checks matching Number.EPSILON thresholds
  if (tThreshold - params.horizTime > -double.minPositive) {
    return minPenalty; // Already too much
  }

  // Find min time of impact and exit amongst all obstacles
  double tmin = params.horizTime;
  double side = 0.0;
  int nside = 0;

  // Check circular obstacles
  for (int i = 0; i < query.circleCount; i++) {
    final cir = query.circles[i];

    // RVO calculations matching manual multi-subscript additions
    _vab.setValues(
      (vcand.x * 2.0) - vel.x - cir.vel.x,
      (vcand.y * 2.0) - vel.y - cir.vel.y,
      (vcand.z * 2.0) - vel.z - cir.vel.z,
    );

    // Side bias
    final double vdot = vdot2D(cir.dp, _vab) * 0.5 + 0.5;
    final double vperp = vdot2D(cir.np, _vab) * 2.0;
    final double minVal = math.min(vdot, vperp);
    
    side += math.max(0.0, math.min(1.0, minVal));
    nside++;

    sweepCircleCircle(pos, rad, _vab, cir.p, cir.rad, _sweepResult);
    if (!_sweepResult.hit) continue;

    double htmin = _sweepResult.tmin;
    final double htmax = _sweepResult.tmax;

    // Handle overlapping obstacles
    if (htmin < 0.0 && htmax > 0.0) {
      htmin = -htmin * 0.5; // Avoid more when overlapped
    }

    if (htmin >= 0.0) {
      if (htmin < tmin) {
        tmin = htmin;
        if (tmin < tThreshold) {
          return minPenalty;
        }
      }
    }
  }

  // Check segment obstacles
  for (int i = 0; i < query.segmentCount; i++) {
    final seg = query.segments[i];
    double htmin = 0.0;

    if (seg.touch) {
      // Special case when agent is very close to segment
      _sdir.setValues(seg.q.x - seg.p.x, seg.q.y - seg.p.y, seg.q.z - seg.p.z);
      _snorm.setValues(-_sdir.z, _sdir.y, _sdir.x);

      // If the velocity is pointing towards the segment, no collision.
      if (vdot2D(_snorm, vcand) < 0.0) continue;
      htmin = 0.0; // Immediate collision
    } else {
      intersectRaySegment(pos, vcand, seg.p, seg.q, _intersectionResult);
      if (!_intersectionResult.hit) continue;
      htmin = _intersectionResult.t;
    }

    // Avoid less when facing walls
    htmin *= 2.0;

    // Track nearest obstacle
    if (htmin < tmin) {
      tmin = htmin;
      if (tmin < tThreshold) {
        return minPenalty;
      }
    }
  }

  // Normalize side bias
  if (nside > 0) {
    side /= nside;
  }

  final double spen = params.weightSide * side;
  final double tpen = params.weightToi * (1.0 / (0.1 + (tmin * query.invHorizTime)));
  final double penalty = vpen + vcpen + spen + tpen;

  // Store debug info cleanly using optional parameters instead of null checks
  if (debug != null) {
    debug.samples.add(ObstacleAvoidanceSample(
      vel: vcand.clone(),
      ssize: cs,
      pen: penalty,
      vpen: vpen,
      vcpen: vcpen,
      spen: spen,
      tpen: tpen,
    ));
  }

  return penalty;
}

class GridSamplingResult {
  final Vector3 nvel;
  final int samples;

  GridSamplingResult({
    required this.nvel,
    required this.samples,
  });
}

// Reusable scratch vector to prevent runtime allocations
final Vector3 _sampleVelocityGridVcand = Vector3();

/**
 * Sample velocity using grid-based approach.
 */
GridSamplingResult sampleVelocityGrid(
  ObstacleAvoidanceQuery query,
  Vector3 pos,
  double rad,
  double vmax,
  Vector3 vel,
  Vector3 dvel,
  ObstacleAvoidanceParams params, [
  ObstacleAvoidanceDebugData? debug,
]) {
  prepareObstacles(query, pos, dvel);
  copyParams(query.params, params);
  
  query.invHorizTime = 1.0 / query.params.horizTime;
  query.vmax = vmax;
  query.invVmax = vmax > 0 ? 1.0 / vmax : double.maxFinite;

  final Vector3 nvel = Vector3(0, 0, 0);
  if (debug != null) {
    debug.reset();
  }

  final double cvx = dvel.x * params.velBias;
  final double cvz = dvel.z * params.velBias;
  final double cs = (vmax * 2.0 * (1.0 - params.velBias)) / (params.gridSize - 1);
  final double half = ((params.gridSize - 1) * cs) * 0.5;
  
  double minPenalty = double.maxFinite;
  int ns = 0;

  // Pre-compute vmax squared for bounds checking
  final double vmaxPlusHalfCs = vmax + (cs / 2.0);
  final double vmaxSqr = vmaxPlusHalfCs * vmaxPlusHalfCs;

  for (int y = 0; y < params.gridSize; ++y) {
    for (int x = 0; x < params.gridSize; ++x) {
      final vcand = _sampleVelocityGridVcand;
      vcand.x = cvx + (x * cs) - half;
      vcand.y = 0;
      vcand.z = cvz + (y * cs) - half;

      if ((vcand.x * vcand.x) + (vcand.z * vcand.z) > vmaxSqr) {
        continue;
      }

      final double penalty = processSample(
        query, vcand, cs, pos, rad, vel, dvel, minPenalty, debug,
      );
      ns++;

      if (penalty < minPenalty) {
        minPenalty = penalty;
        nvel.setFrom(vcand);
      }
    }
  }

  return GridSamplingResult(nvel: nvel, samples: ns);
}

/**
 * Normalize a 2D vector (ignoring Y component).
 */
void normalize2D(Vector3 v) {
  final double d = math.sqrt((v.x * v.x) + (v.z * v.z));
  if (d == 0.0) return;
  final double invD = 1.0 / d;
  v.x *= invD;
  v.z *= invD;
}

/**
 * Rotate a 2D vector (ignoring Y component).
 */
void rotate2D(Vector3 dest, Vector3 v, double ang) {
  final double c = math.cos(ang);
  final double s = math.sin(ang);
  dest.x = (v.x * c) - (v.z * s);
  dest.z = (v.x * s) + (v.z * c);
  dest.y = v.y;
}

// Adaptive sampling reusable internal state instances
final Vector3 _sampleVelocityAdaptiveDdir = Vector3();
final Vector3 _sampleVelocityAdaptiveDdir2 = Vector3();
final Vector3 _sampleVelocityAdaptiveRes = Vector3();
final Vector3 _sampleVelocityAdaptiveBvel = Vector3();
final Vector3 _sampleVelocityAdaptiveVcand = Vector3();

/**
 * Sample velocity using adaptive approach.
 */
int sampleVelocityAdaptive(
  ObstacleAvoidanceQuery query,
  Vector3 pos,
  double rad,
  double vmax,
  Vector3 vel,
  Vector3 dvel,
  ObstacleAvoidanceParams params,
  Vector3 outVelocity, [
  ObstacleAvoidanceDebugData? debug,
]) {
  prepareObstacles(query, pos, dvel);
  copyParams(query.params, params);
  
  query.invHorizTime = 1.0 / query.params.horizTime;
  query.vmax = vmax;
  query.invVmax = vmax > 0 ? 1.0 / vmax : double.maxFinite;

  if (debug != null) {
    debug.reset();
  }

  // Build sampling pattern aligned to desired velocity
  final pat = query.pattern;
  int npat = 0;
  
  final int ndivs = math.max(1, math.min(query.params.adaptiveDivs, DT_MAX_PATTERN_DIVS));
  final int nrings = math.max(1, math.min(query.params.adaptiveRings, DT_MAX_PATTERN_RINGS));
  final int depth = query.params.adaptiveDepth;
  
  final double da = (1.0 / ndivs) * DT_PI * 2.0;
  final double ca = math.cos(da);
  final double sa = math.sin(da);

  // Desired direction - use pre-allocated vectors to avoid cloning
  final ddir = _sampleVelocityAdaptiveDdir..setFrom(dvel);
  normalize2D(ddir);
  
  final ddir2 = _sampleVelocityAdaptiveDdir2;
  rotate2D(ddir2, ddir, da * 0.5); // Rotated by da/2

  // Always add sample at zero
  pat[npat * 2] = 0.0;
  pat[npat * 2 + 1] = 0.0;
  npat++;

  for (int j = 0; j < nrings; ++j) {
    final double r = (nrings - j) / nrings;
    final Vector3 baseDir = (j % 2 == 0) ? ddir : ddir2;
    
    pat[npat * 2] = baseDir.x * r;
    pat[npat * 2 + 1] = baseDir.z * r;
    
    int last1 = npat * 2; // Points to current element
    int last2 = last1;    // Both point to same location initially
    npat++;

    for (int i = 1; i < ndivs - 1; i += 2) {
      // Get next point on the "right" (rotate CW)
      pat[npat * 2] = (pat[last1] * ca) + (pat[last1 + 1] * sa);
      pat[npat * 2 + 1] = (-pat[last1] * sa) + (pat[last1 + 1] * ca);
      
      // Get next point on the "left" (rotate CCW)
      pat[(npat * 2) + 2] = (pat[last2] * ca) - (pat[last2 + 1] * sa);
      pat[(npat * 2) + 3] = (pat[last2] * sa) + (pat[last2 + 1] * ca);
      
      last1 = npat * 2; // Point to current "right" element
      last2 = last1 + 2; // Point to current "left" element
      npat += 2;
    }

    if ((ndivs & 1) == 0) {
      pat[npat * 2] = (pat[last2] * ca) - (pat[last2 + 1] * sa);
      pat[npat * 2 + 1] = (pat[last2] * sa) + (pat[last2 + 1] * ca);
      npat++;
    }
  }

  // Start sampling
  double cr = vmax * (1.0 - query.params.velBias);
  final res = _sampleVelocityAdaptiveRes;
  res.x = dvel.x * query.params.velBias;
  res.y = 0;
  res.z = dvel.z * query.params.velBias;
  
  int ns = 0;
  
  // Pre-compute vmax squared for bounds checking
  final double vmaxPlusEpsilon = vmax + 0.001;
  final double vmaxSqr = vmaxPlusEpsilon * vmaxPlusEpsilon;

  for (int k = 0; k < depth; ++k) {
    double minPenalty = double.maxFinite;
    final bvel = _sampleVelocityAdaptiveBvel..setValues(0, 0, 0);
    
    // Cache cr / 10 for this depth iteration
    final double crOverTen = cr * 0.1;

    for (int i = 0; i < npat; ++i) {
      final vcand = _sampleVelocityAdaptiveVcand;
      vcand.x = res.x + (pat[i * 2] * cr);
      vcand.y = 0;
      vcand.z = res.z + (pat[i * 2 + 1] * cr);

      if ((vcand.x * vcand.x) + (vcand.z * vcand.z) > vmaxSqr) {
        continue;
      }

      final double penalty = processSample(
        query, vcand, crOverTen, pos, rad, vel, dvel, minPenalty, debug,
      );
      ns++;

      if (penalty < minPenalty) {
        minPenalty = penalty;
        bvel.setFrom(vcand);
      }
    }
    
    res.setFrom(bvel);
    cr *= 0.5;
  }

  outVelocity.setFrom(res);
  return ns;
}
