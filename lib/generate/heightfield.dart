import 'dart:math' as math;
import 'package:navcat/generate/index.dart';
import 'package:three_js_math/three_js_math.dart';

 class HeightfieldSpan {
  /// the lower limit of the span 
  double min;
  /// the upper limit of the span 
  double max;
  /// the area id assigned to the span 
  int area;
  /// the next heightfield span 
  HeightfieldSpan? next;

  HeightfieldSpan({required this.min, required this.max, required this.area, this.next});
}

 class Heightfield {
  /// the width of the heightfield (along x axis in cell units) 
  int width;
  /// the height of the heightfield (along z axis in cell units) 
  int height;
  /// the bounds in world space 
  BoundingBox bounds;
  /// the vertical size of each cell (minimum increment along y) 
  double cellHeight;
  /// the vertical size of each cell (minimum increment along x and z) 
  double cellSize;
  /// the heightfield of spans, (width*height) 
  List<HeightfieldSpan?> spans;

  Heightfield({required this.width, required this.height, required this.bounds, required this.cellHeight, required this.cellSize, required this.spans});
}

const spanMaxHeight = 0x1fff; // 8191
const maxHeightfieldHeight = 0xffff;

Vector2 calculateGridSize(Vector2 outGridSize, BoundingBox bounds, double cellSize) {
  // Safety guard 1: Check for zero or negative cell sizes
  if (cellSize <= 0.0) {
    outGridSize.setValues(0.0, 0.0);
    return outGridSize;
  }

  final minBounds = bounds.min;
  final maxBounds = bounds.max;

  // Safety guard 2: Check if bounds are infinite or NaN before calculations
  if (
    !minBounds.x.isFinite || 
    !maxBounds.x.isFinite || 
    !minBounds.z.isFinite || 
    !maxBounds.z.isFinite
  ) {
    outGridSize.setValues(0.0, 0.0);
    return outGridSize;
  }

  // Perform calculations safely
  final double widthRaw = (maxBounds.x - minBounds.x) / cellSize + 0.5;
  final double heightRaw = (maxBounds.z - minBounds.z) / cellSize + 0.5;

  // Safety guard 3: Ultimate check right before calling .floor()
  if (!widthRaw.isFinite || !heightRaw.isFinite) {
    outGridSize.setValues(0.0, 0.0);
    return outGridSize;
  }

  outGridSize.x = widthRaw.floor().toDouble();
  outGridSize.y = heightRaw.floor().toDouble();

  return outGridSize;
}

Heightfield createHeightfield(
  double width,
  double height,
  BoundingBox bounds,
  double cellSize,
  double cellHeight,
) {
  final int numSpans = (width * height).toInt();
  final spans = List<HeightfieldSpan?>.filled(numSpans, null);

  return Heightfield(
    width: width.toInt(),
    height: height.toInt(),
    spans: spans,
    bounds: bounds,
    cellHeight: cellHeight,
    cellSize: cellSize,
  );
}

/// Adds a span to the heightfield. If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
bool addHeightfieldSpan(Heightfield heightfield, int x, int z, double min, double max, int areaID, double flagMergeThreshold) {
  // Create the new span
  final newSpan = HeightfieldSpan(min: min, max: max, area: areaID, next: null);

  final int columnIndex = x + z * heightfield.width.toInt();
  HeightfieldSpan? previousSpan;
  HeightfieldSpan? currentSpan = heightfield.spans[columnIndex];

  // Insert the new span, possibly merging it with existing spans
  while (currentSpan != null) {
    if (currentSpan.min > newSpan.max) {
      // Current span is completely after the new span, break
      break;
    }

    if (currentSpan.max < newSpan.min) {
      // Current span is completely before the new span. Keep going
      previousSpan = currentSpan;
      currentSpan = currentSpan.next;
    } 
    else {
      // The new span overlaps with an existing span. Merge them
      if (currentSpan.min < newSpan.min) {
        newSpan.min = currentSpan.min;
      }
      if (currentSpan.max > newSpan.max) {
        newSpan.max = currentSpan.max;
      }

      // Merge flags
      if ((newSpan.max - currentSpan.max).abs() <= flagMergeThreshold) {
        // Higher area ID numbers indicate higher resolution priority
        newSpan.area = math.max(newSpan.area, currentSpan.area);
      }

      // Remove the current span since it's now merged with newSpan
      final next = currentSpan.next;
      if (previousSpan != null) {
        previousSpan.next = next;
      } 
      else {
        heightfield.spans[columnIndex] = next;
      }
      currentSpan = next;
    }
  }

  // Insert new span after prev
  if (previousSpan != null) {
    newSpan.next = previousSpan.next;
    previousSpan.next = newSpan;
  } 
  else {
    // This span should go before the others in the list
    newSpan.next = heightfield.spans[columnIndex];
    heightfield.spans[columnIndex] = newSpan;
  }

  return true;
}

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
void dividePoly(
  Map<String, int> out,
  List<double> inVerts,
  int inVertsCount,
  List<double> outVerts1,
  List<double> outVerts2,
  double axisOffset,
  int axis,
){
  // How far positive or negative away from the separating axis is each vertex
  final inVertAxisDelta = _inVertAxisDelta;
  for (int inVert = 0; inVert < inVertsCount; ++inVert) {
    inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
  }

  int poly1Vert = 0;
  int poly2Vert = 0;

  for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA) {
    // If the two vertices are on the same side of the separating axis
    bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

    if (!sameSide) {
      final s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
      outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
      outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
      outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;

      // Copy to second polygon
      outVerts2[poly2Vert * 3 + 0] = outVerts1[poly1Vert * 3 + 0];
      outVerts2[poly2Vert * 3 + 1] = outVerts1[poly1Vert * 3 + 1];
      outVerts2[poly2Vert * 3 + 2] = outVerts1[poly1Vert * 3 + 2];

      poly1Vert++;
      poly2Vert++;

      // Add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
      // since these were already added above
      if (inVertAxisDelta[inVertA] > 0) {
        outVerts1[poly1Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
        outVerts1[poly1Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
        outVerts1[poly1Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
        poly1Vert++;
      } else if (inVertAxisDelta[inVertA] < 0) {
        outVerts2[poly2Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
        outVerts2[poly2Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
        outVerts2[poly2Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
        poly2Vert++;
      }
    } 
    else {
      // Add the inVertA point to the right polygon. Addition is done even for points on the dividing line
      if (inVertAxisDelta[inVertA] >= 0) {
        outVerts1[poly1Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
        outVerts1[poly1Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
        outVerts1[poly1Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
        poly1Vert++;
        if (inVertAxisDelta[inVertA] != 0) {
          continue;
        }
      }
      outVerts2[poly2Vert * 3 + 0] = inVerts[inVertA * 3 + 0];
      outVerts2[poly2Vert * 3 + 1] = inVerts[inVertA * 3 + 1];
      outVerts2[poly2Vert * 3 + 2] = inVerts[inVertA * 3 + 2];
      poly2Vert++;
    }
  }

  out['nv1'] = poly1Vert;
  out['nv2'] = poly2Vert;
}

final _triangleBounds = BoundingBox();
final _rasterizeTriMin = Vector3();
final _rasterizeTriMax = Vector3();

final _inVerts = List<double>.filled(7 * 3, 0.0);
final _inRow = List<double>.filled(7 * 3, 0.0);
final _p1 = List<double>.filled(7 * 3, 0.0);
final _p2 = List<double>.filled(7 * 3, 0.0);

final _inVertAxisDelta = List<double>.filled(12, 0.0);
final _dividePolyResult = <String, int>{ 'nv1': 0, 'nv2': 0 };

final _v0 = Vector3.zero();
final _v1 = Vector3.zero();
final _v2 = Vector3.zero();

/// Rasterize a single triangle to the heightfield
bool rasterizeTriangle(
    Vector3 v0,
    Vector3 v1,
    Vector3 v2,
    int areaID,
    Heightfield heightfield,
    double flagMergeThreshold,
) {
  _rasterizeTriMin.setFrom(v0);
  _rasterizeTriMin.min(v1);
  _rasterizeTriMin.min(v2);

  _rasterizeTriMax.setFrom(v0);
  _rasterizeTriMax.max(v1);
  _rasterizeTriMax.max(v2);

  _triangleBounds.set(_rasterizeTriMin, _rasterizeTriMax);

  // If the triangle does not touch the bounding box of the heightfield, skip the triangle
  if (!_triangleBounds.intersectsBox(heightfield.bounds)) {
    return true;
  }

  final heightfieldBoundsMin = heightfield.bounds.min;
  final heightfieldBoundsMax = heightfield.bounds.max;

  final w = heightfield.width.toInt();
  final h = heightfield.height.toInt();
  final by = heightfieldBoundsMax.y - heightfieldBoundsMin.y;
  final cellSize = heightfield.cellSize;
  final cellHeight = heightfield.cellHeight;
  final inverseCellSize = 1.0 / cellSize;
  final inverseCellHeight = 1.0 / cellHeight;

  // Calculate the footprint of the triangle on the grid's z-axis
  int z0 = ((_rasterizeTriMin.z - heightfieldBoundsMin.z) * inverseCellSize).floor();
  int z1 = ((_rasterizeTriMax.z - heightfieldBoundsMin.z) * inverseCellSize).floor();

  // Use -1 rather than 0 to cut the polygon properly at the start of the tile
  z0 = MathUtils.clamp(z0, -1, h - 1);
  z1 = MathUtils.clamp(z1, 0, h - 1);

  // Clip the triangle into all grid cells it touches
  List<double> inVerts = _inVerts;
  List<double> inRow = _inRow;
  List<double> p1 = _p1;
  List<double> p2 = _p2;

  // Copy triangle vertices
  inVerts[0] = v0.x;
  inVerts[1] = v0.y;
  inVerts[2] = v0.z;
  inVerts[3] = v1.x;
  inVerts[4] = v1.y;
  inVerts[5] = v1.z;
  inVerts[6] = v2.x;
  inVerts[7] = v2.y;
  inVerts[8] = v2.z;

  int nvIn = 3;

  for (int z = z0; z <= z1; ++z) {
    // Clip polygon to row. Store the remaining polygon as well
    final cellZ = heightfieldBoundsMin.z + z * cellSize;
    dividePoly(_dividePolyResult, inVerts, nvIn, inRow, p1, cellZ + cellSize, axisZ);
    final nvRow = _dividePolyResult['nv1']!;
    final nvIn2 = _dividePolyResult['nv2']!;

    // Swap arrays
    final temp = inVerts;
    inVerts = p1;
    p1 = temp;
    nvIn = nvIn2;

    if (nvRow < 3) {
      continue;
    }
    if (z < 0) {
      continue;
    }

    // Find X-axis bounds of the row
    double minX = inRow[0];
    double maxX = inRow[0];
    for (int vert = 1; vert < nvRow; ++vert) {
      if (minX > inRow[vert * 3]) {
        minX = inRow[vert * 3];
      }
      if (maxX < inRow[vert * 3]) {
        maxX = inRow[vert * 3];
      }
    }

    int x0 = ((minX - heightfieldBoundsMin.x) * inverseCellSize).floor();
    int x1 = ((maxX - heightfieldBoundsMin.x) * inverseCellSize).floor();
    if (x1 < 0 || x0 >= w) {
      continue;
    }
    x0 = MathUtils.clamp(x0, -1, w - 1);
    x1 = MathUtils.clamp(x1, 0, w - 1);

    int nv2 = nvRow;

    for (int x = x0; x <= x1; ++x) {
      // Clip polygon to column. Store the remaining polygon as well
      final cx = heightfieldBoundsMin.x + x * cellSize;
      dividePoly(_dividePolyResult, inRow, nv2, p1, p2, cx + cellSize, axisX);
      final nv = _dividePolyResult['nv1']!;
      final nv2New = _dividePolyResult['nv2']!;

      // Swap arrays
      final temp = inRow;
      inRow = p2;
      p2 = temp;
      nv2 = nv2New;

      if (nv < 3) {
        continue;
      }
      if (x < 0) {
        continue;
      }

      // Calculate min and max of the span
      double spanMin = p1[1];
      double spanMax = p1[1];
      for (int vert = 1; vert < nv; ++vert) {
        spanMin = math.min(spanMin, p1[vert * 3 + 1]);
        spanMax = math.max(spanMax, p1[vert * 3 + 1]);
      }
      spanMin -= heightfieldBoundsMin[1];
      spanMax -= heightfieldBoundsMin[1];

      // Skip the span if it's completely outside the heightfield bounding box
      if (spanMax < 0.0) {
        continue;
      }
      if (spanMin > by) {
        continue;
      }

      // Clamp the span to the heightfield bounding box
      if (spanMin < 0.0) {
        spanMin = 0;
      }
      if (spanMax > by) {
        spanMax = by;
      }

      // Snap the span to the heightfield height grid
      final spanMinCellIndex = MathUtils.clamp((spanMin * inverseCellHeight).floor(), 0, spanMaxHeight);
      final spanMaxCellIndex = MathUtils.clamp((spanMax * inverseCellHeight).ceil(), spanMinCellIndex + 1, spanMaxHeight);

      if (!addHeightfieldSpan(heightfield, x, z, spanMinCellIndex.toDouble(), spanMaxCellIndex.toDouble(), areaID, flagMergeThreshold)) {
        return false;
      }
    }
  }

  return true;
}

bool rasterizeTriangles(
    BuildContextState ctx,
    Heightfield heightfield,
    List<double> vertices,
    List<int> indices,
    List<int> triAreaIds,
    [double flagMergeThreshold = 1,]
) {
  final numTris = indices.length ~/ 3;

  for (int triIndex = 0; triIndex < numTris; ++triIndex) {
    final i0 = indices[triIndex * 3 + 0];
    final i1 = indices[triIndex * 3 + 1];
    final i2 = indices[triIndex * 3 + 2];

    final v0 = _v0.fromArray(vertices, i0 * 3);
    final v1 = _v1.fromArray(vertices, i1 * 3);
    final v2 = _v2.fromArray(vertices, i2 * 3);

    final areaId = triAreaIds[triIndex];

    if (!rasterizeTriangle(v0, v1, v2, areaId, heightfield, flagMergeThreshold)) {
      print('Failed to rasterize triangle');
      return false;
    }
  }

  return true;
}

void filterLowHangingWalkableObstacles(Heightfield heightfield, double walkableClimb) {
  final xSize = heightfield.width;
  final zSize = heightfield.height;

  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      HeightfieldSpan? previousSpan ;
      bool previousWasWalkable = false;
      int previousAreaID = nullArea;

      // For each span in the column...
      final columnIndex = x + z * xSize.toInt();
      HeightfieldSpan? span = heightfield.spans[columnIndex];

      while (span != null) {
        final walkable = span.area != nullArea;

        // If current span is not walkable, but there is walkable span just below it and the height difference
        // is small enough for the agent to walk over, mark the current span as walkable too.
        if (!walkable && previousWasWalkable && previousSpan != null && span.max - previousSpan.max <= walkableClimb) {
          span.area = previousAreaID;
        }

        // Copy the original walkable value regardless of whether we changed it.
        // This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
        previousWasWalkable = walkable;
        previousAreaID = span.area;
        previousSpan = span;
        span = span.next;
      }
    }
  }
}

void filterLedgeSpans(Heightfield heightfield, double walkableHeight, double walkableClimb) {
  final xSize = heightfield.width;
  final zSize = heightfield.height;

  // Mark spans that are adjacent to a ledge as unwalkable
  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final columnIndex = x + z * xSize.toInt();
      HeightfieldSpan? span = heightfield.spans[columnIndex];

      while (span != null) {
        // Skip non-walkable spans
        if (span.area == nullArea) {
          span = span.next;
          continue;
        }

        final floor = span.max;
        final ceiling = span.next != null? span.next!.min : maxHeightfieldHeight;

        // The difference between this walkable area and the lowest neighbor walkable area.
        // This is the difference between the current span and all neighbor spans that have
        // enough space for an agent to move between, but not accounting at all for surface slope.
        double lowestNeighborFloorDifference = maxHeightfieldHeight.toDouble();

        // Min and max height of accessible neighbours.
        double lowestTraversableNeighborFloor = span.max;
        double highestTraversableNeighborFloor = span.max;

        for (int direction = 0; direction < 4; ++direction) {
          final neighborX = x + getDirOffsetX(direction);
          final neighborZ = z + getDirOffsetY(direction);

          // Skip neighbours which are out of bounds.
          if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) {
            lowestNeighborFloorDifference = -walkableClimb - 1;
            break;
          }

          final neighborColumnIndex = neighborX + neighborZ * xSize.toInt();
          HeightfieldSpan? neighborSpan = heightfield.spans[neighborColumnIndex];

          // The most we can step down to the neighbor is the walkableClimb distance.
          // Start with the area under the neighbor span
          double neighborCeiling = neighborSpan != null? neighborSpan.min : maxHeightfieldHeight.toDouble();

          // Skip neighbour if the gap between the spans is too small.
          if (math.min(ceiling, neighborCeiling) - floor >= walkableHeight) {
            lowestNeighborFloorDifference = -walkableClimb - 1;
            break;
          }

          // For each span in the neighboring column...
          while (neighborSpan != null) {
            final neighborFloor = neighborSpan.max;
            neighborCeiling = neighborSpan.next != null? neighborSpan.next!.min : maxHeightfieldHeight.toDouble();

            // Only consider neighboring areas that have enough overlap to be potentially traversable.
            if (math.min(ceiling, neighborCeiling) - math.max(floor, neighborFloor) < walkableHeight) {
              // No space to traverse between them.
              neighborSpan = neighborSpan.next;
              continue;
            }

            final neighborFloorDifference = neighborFloor - floor;
            lowestNeighborFloorDifference = math.min(lowestNeighborFloorDifference, neighborFloorDifference);

            // Find min/max accessible neighbor height.
            // Only consider neighbors that are at most walkableClimb away.
            if (neighborFloorDifference.abs() <= walkableClimb) {
              // There is space to move to the neighbor cell and the slope isn't too much.
              lowestTraversableNeighborFloor = math.min(lowestTraversableNeighborFloor, neighborFloor);
              highestTraversableNeighborFloor = math.max(highestTraversableNeighborFloor, neighborFloor);
            } else if (neighborFloorDifference < -walkableClimb) {
              // We already know this will be considered a ledge span so we can early-out
              break;
            }

            neighborSpan = neighborSpan.next;
          }
        }

        // The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
        // That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
        if (lowestNeighborFloorDifference < -walkableClimb) {
          span.area = nullArea;
        }
        // If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
        else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb) {
          span.area = nullArea;
        }

        span = span.next;
      }
    }
  }
}

void filterWalkableLowHeightSpans(Heightfield heightfield, double walkableHeight) {
  final xSize = heightfield.width;
  final zSize = heightfield.height;

  // Remove walkable flag from spans which do not have enough
  // space above them for the agent to stand there.
  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final columnIndex = x + z * xSize.toInt();
      HeightfieldSpan? span = heightfield.spans[columnIndex];

      while (span != null) {
        final floor = span.max;
        final ceiling = span.next != null? span.next!.min : maxHeightfieldHeight.toDouble();

        if (ceiling - floor < walkableHeight) {
          span.area = nullArea;
        }

        span = span.next;
      }
    }
  }
}
