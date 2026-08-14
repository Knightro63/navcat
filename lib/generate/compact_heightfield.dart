import 'dart:typed_data';
import 'dart:math' as math;
import 'package:navcat/generate/build_context.dart';
import 'package:navcat/generate/common.dart';
import 'package:navcat/generate/heightfield.dart';
import 'package:navcat/geometry.dart';
import 'package:three_js_math/three_js_math.dart';

class CompactHeightfieldSpan {
  /// the lower extent of the span. measured from the heightfields base.
  int y;
  /// the id of the region the span belongs to, or zero if not in a region
  int region;
  /// packed neighbour connection data
  int con;
  /// the height of the span, measured from y
  int h;

  CompactHeightfieldSpan({
    this.y = 0,
    this.region = 0,
    this.con = 0,
    this.h = 0,
  });
}

class CompactHeightfieldCell {
  /// index to the first span in the column
  int index;
  /// number of spans in the column
  int count;

  CompactHeightfieldCell({
    this.index = 0,
    this.count = 0,
  });
}

BoundingBox box3 = BoundingBox();

class CompactHeightfield {
  /// the width of the heightfield (along x axis in cell units) 
  int width;
  /// the height of the heightfield (along z axis in cell units) 
  int height;
  /// the number of spans in the heightfield 
  int spanCount;
  /// the walkable height used during the build of the heightfield 
  double walkableHeightVoxels;
  /// the walkable climb used during the build of the heightfield 
  double walkableClimbVoxels;
  /// the AABB border size used during the build of the heightfield 
  double borderSize;
  /// the maxiumum distance value of any span within the heightfield 
  double maxDistance;
  /// the maximum region id of any span within the heightfield 
  int maxRegions;
  /// the heightfield bounds in world space 
  late final BoundingBox bounds;
  /// the size of each cell 
  double cellSize;
  /// the height of each cell 
  double cellHeight;
  /// array of cells, size = width*height 
  late List<CompactHeightfieldCell> cells;
  /// array of spans, size = spanCount 
  late List<CompactHeightfieldSpan> spans;
  /// array containing area id data, size = spanCount 
  late List<int> areas;
  /// array containing distance field data, size = spanCount 
  List<double>? distances;

  CompactHeightfield({
    this.width = 0,
    this.height = 0,
    this.spanCount = 0,
    this.walkableHeightVoxels = 0,
    this.walkableClimbVoxels = 0,
    this.borderSize = 0,
    this.maxDistance = 0,
    this.maxRegions = 0,
    BoundingBox? bounds,
    this.cellSize = 0,
    this.cellHeight = 0,
    List<CompactHeightfieldCell>? cells,
    List<CompactHeightfieldSpan>? spans,
    List<int>? areas,
    this.distances,
  }){
    this.bounds = bounds ?? BoundingBox();
    this.cells = cells ?? [];
    this.spans = spans ?? [];
    this.areas = areas ?? [];
  }
}

/// Helper function to set connection data in a span
void setCon(CompactHeightfieldSpan span, int dir, int layerIndex) {
  final shift = dir * 6; // 6 bits per direction
  final mask = 0x3f << shift; // 6-bit mask
  span.con = ((span.con & ~mask) | ((layerIndex & 0x3f) << shift)) & 0xFFFFFFFF;
}

/// Helper function to get connection data from a span
int getCon(CompactHeightfieldSpan span, int dir) {
  final shift = dir * 6; // 6 bits per direction
  return (span.con >> shift) & 0x3f;
}

/// Count the number of walkable spans in the heightfield
int getHeightFieldSpanCount(Heightfield heightfield) {
  final numCols = heightfield.width * heightfield.height;
  int spanCount = 0;

  for (int columnIndex = 0; columnIndex < numCols; ++columnIndex) {
    HeightfieldSpan? span = heightfield.spans[columnIndex];
    while (span != null) {
      if (span.area != nullArea) {
        spanCount++;
      }
      span = span.next;
    }
  }

  return spanCount;
}

/// Build a compact heightfield from a heightfield
CompactHeightfield buildCompactHeightfield(
  BuildContextState ctx,
  double walkableHeightVoxels,
  double walkableClimbVoxels,
  Heightfield heightfield,
) {
  final xSize = heightfield.width;
  final zSize = heightfield.height;
  final spanCount = getHeightFieldSpanCount(heightfield);

  final List<CompactHeightfieldCell> cells = [];
  for (int i = 0; i < xSize * zSize; i++) {
    cells.add(CompactHeightfieldCell());
  }

  final List<CompactHeightfieldSpan> spans = [];
  for (int i = 0; i < spanCount; i++) {
    spans.add(CompactHeightfieldSpan());
  }

  final compactHeightfield = CompactHeightfield(
    width: xSize,
    height: zSize,
    spanCount: spanCount,
    walkableHeightVoxels: walkableHeightVoxels,
    walkableClimbVoxels: walkableClimbVoxels,
    borderSize: 0,
    maxDistance: 0,
    maxRegions: 0,
    bounds: heightfield.bounds.clone(),
    cellSize: heightfield.cellSize,
    cellHeight: heightfield.cellHeight,
    cells: cells,
    spans: spans,
    areas: List.filled(spanCount, nullArea),
    distances: List.filled(spanCount, 0),
  );

  // adjust upper bound to account for walkable height
  compactHeightfield.bounds.max.y += walkableHeightVoxels * heightfield.cellHeight;

  // fill in cells and spans
  int currentCellIndex = 0;
  final numColumns = xSize * zSize;

  for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex) {
    HeightfieldSpan? span = heightfield.spans[columnIndex];

    // if there are no spans at this cell, just leave the data to index=0, count=0.
    if (span == null) {
      continue;
    }

    final cell = compactHeightfield.cells[columnIndex];
    cell.index = currentCellIndex;
    cell.count = 0;

    while (span != null) {
      if (span.area != nullArea) {
        final bot = span.max.toInt();
        final top = (span.next != null ? span.next!.min : maxHeight).toInt();

        compactHeightfield.spans[currentCellIndex].y = math.min(math.max(bot, 0), 0xffff);
        compactHeightfield.spans[currentCellIndex].h = math.min(math.max(top - bot, 0), 0xff);
        compactHeightfield.areas[currentCellIndex] = span.area;

        currentCellIndex++;
        cell.count++;
      }
      span = span.next;
    }
  }

  // find neighbour connections
  int maxLayerIndex = 0;
  final zStride = xSize;

  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];

      for (int i = cell.index; i < cell.index + cell.count; ++i) {
        final span = compactHeightfield.spans[i];

        for (int dir = 0; dir < 4; ++dir) {
          setCon(span, dir, notConnected);

          final neighborX = x + dirOffsets[dir][0];
          final neighborZ = z + dirOffsets[dir][1];

          // first check that the neighbour cell is in bounds.
          if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) {
            continue;
          }

          // iterate over all neighbour spans and check if any of them is
          // accessible from current cell.
          final neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];

          for (int k = neighborCell.index; k < neighborCell.index + neighborCell.count; ++k) {
            final neighborSpan = compactHeightfield.spans[k];
            final bot = math.max(span.y, neighborSpan.y);
            final top = math.min(span.y + span.h, neighborSpan.y + neighborSpan.h);

            // check that the gap between the spans is walkable,
            // and that the climb height between the gaps is not too high.
            if (top - bot >= walkableHeightVoxels && (neighborSpan.y - span.y).abs() <= walkableClimbVoxels) {
              // Mark direction as walkable.
              final layerIndex = k - neighborCell.index;
              if (layerIndex < 0 || layerIndex > maxLayers) {
                maxLayerIndex = math.max(maxLayerIndex, layerIndex);
                continue;
              }
              setCon(span, dir, layerIndex);
              break;
            }
          }
        }
      }
    }
  }

  if (maxLayerIndex > maxLayers) {
    //console.warning(ctx, 'buildCompactHeightfield: Heightfield has too many layers $maxLayerIndex (max: $maxLayers)');
  }

  return compactHeightfield;
}

final int maxDistance = 255;

///
/// Computes a distance field for the compact heightfield.
/// Each span gets a distance value representing how far it is from any boundary or obstacle.
/// @returns A Uint8Array containing distance values for each span
///
Uint8List computeDistanceToBoundary(CompactHeightfield compactHeightfield) {
  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // for readability

  // initialize distance array
  final distanceToBoundary = Uint8List(compactHeightfield.spanCount);
  distanceToBoundary.fillRange(0, compactHeightfield.spanCount, maxDistance);

  // mark boundary cells
  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      for (int spanIndex = cell.index; spanIndex < cell.index + cell.count; ++spanIndex) {
        if (compactHeightfield.areas[spanIndex] == nullArea) {
          distanceToBoundary[spanIndex] = 0;
          continue;
        }

        final span = compactHeightfield.spans[spanIndex];

        // check that there is a non-null adjacent span in each of the 4 cardinal directions
        int neighborCount = 0;
        for (int direction = 0; direction < 4; ++direction) {
          final neighborConnection = getCon(span, direction);
          if (neighborConnection == notConnected) {
            break;
          }

          final neighborX = x + dirOffsets[direction][0];
          final neighborZ = z + dirOffsets[direction][1];
          final neighborSpanIndex =
              compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;

          if (compactHeightfield.areas[neighborSpanIndex] == nullArea) {
            break;
          }
          neighborCount++;
        }

        // at least one missing neighbour, so this is a boundary cell
        if (neighborCount != 4) {
          distanceToBoundary[spanIndex] = 0;
        }
      }
    }
  }

  // pass 1: Forward pass (top-left to bottom-right)
  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        if (getCon(span, 0) != notConnected) {
          // (-1,0) - West neighbor
          final aX = x + dirOffsets[0][0];
          final aY = z + dirOffsets[0][1];
          final aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 0);
          final aSpan = compactHeightfield.spans[aIndex];
          int newDistance = math.min(distanceToBoundary[aIndex] + 2, maxDistance);
          if (newDistance < distanceToBoundary[spanIndex]) {
            distanceToBoundary[spanIndex] = newDistance;
          }

          // (-1,-1) - Northwest diagonal
          if (getCon(aSpan, 3) != notConnected) {
            final bX = aX + dirOffsets[3][0];
            final bY = aY + dirOffsets[3][1];
            final bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 3);
            newDistance = math.min(distanceToBoundary[bIndex] + 3, maxDistance);
            if (newDistance < distanceToBoundary[spanIndex]) {
              distanceToBoundary[spanIndex] = newDistance;
            }
          }
        }

        if (getCon(span, 3) != notConnected) {
          // (0,-1) - North neighbor
          final aX = x + dirOffsets[3][0];
          final aY = z + dirOffsets[3][1];
          final aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 3);
          final aSpan = compactHeightfield.spans[aIndex];
          int newDistance = math.min(distanceToBoundary[aIndex] + 2, maxDistance);
          if (newDistance < distanceToBoundary[spanIndex]) {
            distanceToBoundary[spanIndex] = newDistance;
          }

          // (1,-1) - Northeast diagonal
          if (getCon(aSpan, 2) != notConnected) {
            final bX = aX + dirOffsets[2][0];
            final bY = aY + dirOffsets[2][1];
            final bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 2);
            newDistance = math.min(distanceToBoundary[bIndex] + 3, maxDistance);
            if (newDistance < distanceToBoundary[spanIndex]) {
              distanceToBoundary[spanIndex] = newDistance;
            }
          }
        }
      }
    }
  }

  // pass 2: Backward pass (bottom-right to top-left)
  for (int z = zSize - 1; z >= 0; --z) {
    for (int x = xSize - 1; x >= 0; --x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        if (getCon(span, 2) != notConnected) {
          // (1,0) - East neighbor
          final aX = x + dirOffsets[2][0];
          final aY = z + dirOffsets[2][1];
          final aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 2);
          final aSpan = compactHeightfield.spans[aIndex];
          final newDistance = math.min(distanceToBoundary[aIndex] + 2, maxDistance);
          if (newDistance < distanceToBoundary[spanIndex]) {
            distanceToBoundary[spanIndex] = newDistance;
          }

          // (1,1) - Southeast diagonal
          if (getCon(aSpan, 1) != notConnected) {
            final bX = aX + dirOffsets[1][0];
            final bY = aY + dirOffsets[1][1];
            final bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 1);
            final newDistance = math.min(distanceToBoundary[bIndex] + 3, maxDistance);
            if (newDistance < distanceToBoundary[spanIndex]) {
              distanceToBoundary[spanIndex] = newDistance;
            }
          }
        }

        if (getCon(span, 1) != notConnected) {
          // (0,1) - South neighbor
          final aX = x + dirOffsets[1][0];
          final aY = z + dirOffsets[1][1];
          final aIndex = compactHeightfield.cells[aX + aY * xSize].index + getCon(span, 1);
          final aSpan = compactHeightfield.spans[aIndex];
          int newDistance = math.min(distanceToBoundary[aIndex] + 2, maxDistance);
          if (newDistance < distanceToBoundary[spanIndex]) {
            distanceToBoundary[spanIndex] = newDistance;
          }

          // (-1,1) - Southwest diagonal
          if (getCon(aSpan, 0) != notConnected) {
            final bX = aX + dirOffsets[0][0];
            final bY = aY + dirOffsets[0][1];
            final bIndex = compactHeightfield.cells[bX + bY * xSize].index + getCon(aSpan, 0);
            final newDistance = math.min(distanceToBoundary[bIndex] + 3, maxDistance);
            if (newDistance < distanceToBoundary[spanIndex]) {
              distanceToBoundary[spanIndex] = newDistance;
            }
          }
        }
      }
    }
  }

  return distanceToBoundary;
}

void erodeWalkableArea(double walkableRadiusVoxels, CompactHeightfield compactHeightfield) {
  final distanceToBoundary = computeDistanceToBoundary(compactHeightfield);

  // erode areas that are too close to boundaries
  final minBoundaryDistance = walkableRadiusVoxels * 2;
  for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex) {
    if (distanceToBoundary[spanIndex] < minBoundaryDistance) {
      compactHeightfield.areas[spanIndex] = nullArea;
    }
  }
}

/// Erodes the walkable area for a base agent radius and marks restricted areas for larger agents based on given walkable radius thresholds.
///
/// Note that this function requires careful tuning of the build parameters to get a good result:
/// - The cellSize needs to be small enough to accurately represent narrow passages. Generally you need to use smaller cellSizes than you otherwise would for single agent navmesh builds.
/// - The thresholds should not be so small that the resulting regions are too small to successfully build good navmesh polygons for. Values like 1-2 voxels will likely lead to poor results.
/// - You may get a better result using "buildRegionsMonotone" over "buildRegions" as this will better handle the many small clusters of areas that may be created from smaller thresholds.
///
/// A typical workflow for using this utility to implement multi-agent support:
/// 1. Call erodeAndMarkWalkableAreas with your smallest agent radius and list of restricted areas
/// 2. Continue with buildDistanceField, buildRegionsMonotone, etc.
/// 3. Configure query filters so large agents exclude the narrow/restricted area IDs
///
/// [baseWalkableRadiusVoxels] the smallest agent radius in voxels (used for erosion)
/// [thresholds] array of area ids and their corresponding walkable radius in voxels.
/// [compactHeightfield] the compact heightfield to process
void erodeAndMarkWalkableAreas(
    int baseWalkableRadiusVoxels,
    List<Map<String, int>> thresholds,
    CompactHeightfield compactHeightfield,
){
  // compute distance field once for both operations
  final distanceToBoundary = computeDistanceToBoundary(compactHeightfield);

  // sort thresholds by radius (smallest first) - we want to mark narrowest corridors first
  final sortedThresholds = [...thresholds]..sort((a, b) => a['walkableRadiusVoxels']! - b['walkableRadiusVoxels']!);

  final baseMinDistance = baseWalkableRadiusVoxels * 2;

  // process each span
  for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex) {
    final distance = distanceToBoundary[spanIndex];

    // first, check if this span should be eroded (removed) based on base agent radius
    if (distance < baseMinDistance) {
      compactHeightfield.areas[spanIndex] = nullArea;
      continue;
    }

    // span survived base erosion, now check if it should be marked
    for (final config in sortedThresholds) {
      final minDistance = config['walkableRadiusVoxels']! * 2;

      if (distance < minDistance) {
        // this span is too narrow for this agent size
        // mark it with the area id
        compactHeightfield.areas[spanIndex] = config['areaId']!;
        break; // once marked, we're done with this span
      }
    }

    // if the span wasn't eroded or marked, it remains in its current area
  }
}

/// Marks spans in the heightfield that intersect the specified box area with the given area ID.
void markBoxArea(
  BoundingBox bounds,
  int areaId,
  CompactHeightfield compactHeightfield,
){
  final boxMinBounds = bounds.min;
  final boxMaxBounds = bounds.max;

  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // For readability

  // Find the footprint of the box area in grid cell coordinates.
  final double tempMinX = (boxMinBounds.x - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize;
  int minX = tempMinX.floor();

  final double tempMinY = (boxMinBounds.y - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight;
  final int minY = tempMinY.floor();

  final double tempMinZ = (boxMinBounds.z - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize;
  int minZ = tempMinZ.floor();

  final double tempMaxX = (boxMaxBounds.x - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize;
  int maxX = tempMaxX.floor();

  final double tempMaxY = (boxMaxBounds.y - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight;
  final int maxY = tempMaxY.floor();

  final double tempMaxZ = (boxMaxBounds.z - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize;
  int maxZ = tempMaxZ.floor();

  // Early-out if the box is outside the bounds of the grid.
  if (maxX < 0) return;
  if (minX >= xSize) return;
  if (maxZ < 0) return;
  if (minZ >= zSize) return;

  // Clamp relevant bound coordinates to the grid.
  if (minX < 0) minX = 0;
  if (maxX >= xSize) maxX = xSize - 1;
  if (minZ < 0) minZ = 0;
  if (maxZ >= zSize) maxZ = zSize - 1;

  // Mark relevant cells.
  for (int z = minZ; z <= maxZ; ++z) {
    for (int x = minX; x <= maxX; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        // Skip if the span is outside the box extents.
        if (span.y < minY || span.y > maxY) {
          continue;
        }

        // Skip if the span has been removed.
        if (compactHeightfield.areas[spanIndex] == nullArea) {
          continue;
        }

        // Mark the span.
        compactHeightfield.areas[spanIndex] = areaId;
      }
    }
  }
}

/// Marks spans in the heightfield that intersect the specified rotated box area with the given area ID.
/// @param center - The center point of the box in world space [x, y, z]
/// @param halfExtents - Half extents of the box along each axis [x, y, z]
/// @param angleRadians - Rotation angle in radians around the Y axis
/// @param areaId - The area ID to assign to intersecting spans
/// @param compactHeightfield - The compact heightfield to mark
void markRotatedBoxArea (
  Vector3 center,
  Vector3 halfExtents,
  double angleRadians,
  int areaId,
  CompactHeightfield compactHeightfield,
){
  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // for readability

  // precompute sin and cos for rotation
  final cosAngle = math.cos(angleRadians);
  final sinAngle = math.sin(angleRadians);

  // compute the 4 corners of the rotated box in the XZ plane and find AABB
  // the corners in local space are at (±halfExtents[0], ±halfExtents[2])
  final hx = halfExtents[0];
  final hz = halfExtents[2];

  // corner 1: (-hx, -hz)
  double worldX = center[0] + cosAngle * -hx - sinAngle * -hz;
  double worldZ = center[2] + sinAngle * -hx + cosAngle * -hz;
  double minWorldX = worldX;
  double maxWorldX = worldX;
  double minWorldZ = worldZ;
  double maxWorldZ = worldZ;

  // corner 2: (hx, -hz)
  worldX = center[0] + cosAngle * hx - sinAngle * -hz;
  worldZ = center[2] + sinAngle * hx + cosAngle * -hz;
  minWorldX = math.min(minWorldX, worldX);
  maxWorldX = math.max(maxWorldX, worldX);
  minWorldZ = math.min(minWorldZ, worldZ);
  maxWorldZ = math.max(maxWorldZ, worldZ);

  // corner 3: (hx, hz)
  worldX = center[0] + cosAngle * hx - sinAngle * hz;
  worldZ = center[2] + sinAngle * hx + cosAngle * hz;
  minWorldX = math.min(minWorldX, worldX);
  maxWorldX = math.max(maxWorldX, worldX);
  minWorldZ = math.min(minWorldZ, worldZ);
  maxWorldZ = math.max(maxWorldZ, worldZ);

  // corner 4: (-hx, hz)
  worldX = center[0] + cosAngle * -hx - sinAngle * hz;
  worldZ = center[2] + sinAngle * -hx + cosAngle * hz;
  minWorldX = math.min(minWorldX, worldX);
  maxWorldX = math.max(maxWorldX, worldX);
  minWorldZ = math.min(minWorldZ, worldZ);
  maxWorldZ = math.max(maxWorldZ, worldZ);

  // compute Y extents in world space
  final minWorldY = center[1] - halfExtents[1];
  final maxWorldY = center[1] + halfExtents[1];

  // convert AABB to grid coordinates
  int minX = ((minWorldX - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int minY = ((minWorldY - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int minZ = ((minWorldZ - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();
  int maxX = ((maxWorldX - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int maxY = ((maxWorldY - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int maxZ = ((maxWorldZ - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();

  // early-out if the rotated box AABB is outside the grid bounds
  if (maxX < 0) return;
  if (minX >= xSize) return;
  if (maxZ < 0) return;
  if (minZ >= zSize) return;

  // clamp to grid bounds
  if (minX < 0) minX = 0;
  if (maxX >= xSize) maxX = xSize - 1;
  if (minZ < 0) minZ = 0;
  if (maxZ >= zSize) maxZ = zSize - 1;

  // iterate through cells in the AABB
  for (int z = minZ; z <= maxZ; ++z) {
    for (int x = minX; x <= maxX; ++x) {
      // calculate cell center in world space
      final cellWorldX = compactHeightfield.bounds.min.x + (x + 0.5) * compactHeightfield.cellSize;
      final cellWorldZ = compactHeightfield.bounds.min.z + (z + 0.5) * compactHeightfield.cellSize;

      // transform cell center to box's local coordinate system
      // first translate to box origin
      final dx = cellWorldX - center[0];
      final dz = cellWorldZ - center[2];

      // then apply inverse rotation (rotation by -angleRadians)
      // inverse rotation matrix for Y-axis: [cos(θ), -sin(θ); sin(θ), cos(θ)]
      final localX = cosAngle * dx - sinAngle * dz;
      final localZ = sinAngle * dx + cosAngle * dz;

      // check if the point is inside the box in local space
      if (localX.abs() > halfExtents[0] || localZ.abs() > halfExtents[2]) {
        continue;
      }

      // cell is inside the rotated box, mark its spans
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        // skip if the span is outside the Y extents
        if (span.y < minY || span.y > maxY) {
          continue;
        }

        // skip if the span has been removed
        if (compactHeightfield.areas[spanIndex] == nullArea) {
          continue;
        }

        // mark the span
        compactHeightfield.areas[spanIndex] = areaId;
      }
    }
  }
}

final _markConvexPolyArea_point = Vector3();

/// Marks spans in the heightfield that intersect the specified convex polygon area with the given area ID.
void markConvexPolyArea (
  List<double> verts,
  double minY,
  double maxY,
  int areaId,
  CompactHeightfield compactHeightfield,
) {
  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // for readability

  // compute the bounding box of the polygon
  final bmin = [verts[0], minY, verts[2]];
  final bmax = [verts[0], maxY, verts[2]];

  final numVerts = verts.length ~/ 3;
  for (int i = 1; i < numVerts; ++i) {
      final vertIndex = i * 3;
      bmin[0] = math.min(bmin[0], verts[vertIndex]);
      bmin[2] = math.min(bmin[2], verts[vertIndex + 2]);
      bmax[0] = math.max(bmax[0], verts[vertIndex]);
      bmax[2] = math.max(bmax[2], verts[vertIndex + 2]);
  }

  // compute the grid footprint of the polygon
  int minx = ((bmin[0] - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int miny = ((bmin[1] - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int minz = ((bmin[2] - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();
  int maxx = ((bmax[0] - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int maxy = ((bmax[1] - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int maxz = ((bmax[2] - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();

  // early-out if the polygon lies entirely outside the grid.
  if (maxx < 0) return;
  if (minx >= xSize) return;
  if (maxz < 0) return;
  if (minz >= zSize) return;

  // clamp the polygon footprint to the grid
  if (minx < 0) minx = 0;
  if (maxx >= xSize) maxx = xSize - 1;
  if (minz < 0) minz = 0;
  if (maxz >= zSize) maxz = zSize - 1;

  // TODO: optimize.
  for (int z = minz; z <= maxz; ++z) {
    for (int x = minx; x <= maxx; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        // skip if span is removed.
        if (compactHeightfield.areas[spanIndex] == nullArea) {
          continue;
        }

        // skip if y extents don't overlap.
        if (span.y < miny || span.y > maxy) {
          continue;
        }
  
        final point = _markConvexPolyArea_point.setValues(
          compactHeightfield.bounds.min.x + (x + 0.5) * compactHeightfield.cellSize,
          0,
          compactHeightfield.bounds.min.z + (z + 0.5) * compactHeightfield.cellSize,
        );

        if (pointInPoly(point, verts, numVerts)) {
          compactHeightfield.areas[spanIndex] = areaId;
        }
      }
    }
  }
}

/// Marks spans in the heightfield that intersect the specified cylinder area with the given area ID.
void markCylinderArea(
  List<double> position,
  double radius,
  double height,
  int areaId,
  CompactHeightfield compactHeightfield,
){
  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // for readability

  // compute the bounding box of the cylinder
  final cylinderBBMin = [position[0] - radius, position[1], position[2] - radius];
  final cylinderBBMax = [position[0] + radius, position[1] + height, position[2] + radius];

  // compute the grid footprint of the cylinder
  int minx = ((cylinderBBMin[0] - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int miny = ((cylinderBBMin[1] - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int minz = ((cylinderBBMin[2] - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();
  int maxx = ((cylinderBBMax[0] - compactHeightfield.bounds.min.x) / compactHeightfield.cellSize).floor();
  int maxy = ((cylinderBBMax[1] - compactHeightfield.bounds.min.y) / compactHeightfield.cellHeight).floor();
  int maxz = ((cylinderBBMax[2] - compactHeightfield.bounds.min.z) / compactHeightfield.cellSize).floor();

  // early-out if the cylinder is completely outside the grid bounds.
  if (maxx < 0 || minx >= xSize || maxz < 0 || minz >= zSize) {
    return;
  }

  // clamp the cylinder bounds to the grid.
  if (minx < 0) minx = 0;
  if (maxx >= xSize) maxx = xSize - 1;
  if (minz < 0) minz = 0;
  if (maxz >= zSize) maxz = zSize - 1;

  final radiusSq = radius * radius;

  for (int z = minz; z <= maxz; ++z) {
    for (int x = minx; x <= maxx; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      final cellX = compactHeightfield.bounds.min.x + (x + 0.5) * compactHeightfield.cellSize;
      final cellZ = compactHeightfield.bounds.min.z + (z + 0.5) * compactHeightfield.cellSize;
      final deltaX = cellX - position[0];
      final deltaZ = cellZ - position[2];

      // skip this column if it's too far from the center point of the cylinder.
      if (deltaX * deltaX + deltaZ * deltaZ >= radiusSq) {
        continue;
      }

      // mark all overlapping spans
      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        // skip if span is removed.
        if (compactHeightfield.areas[spanIndex] == nullArea) {
          continue;
        }

        // mark if y extents overlap.
        if (span.y >= miny && span.y <= maxy) {
          compactHeightfield.areas[spanIndex] = areaId;
        }
      }
    }
  }
}

/// Helper function to perform insertion sort on a small array
void insertSort(List<num> arr, int length){
  for (int i = 1; i < length; ++i) {
    final key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

final _neighborAreas = List<int>.filled(9, 0);

/// Applies a median filter to walkable area types (based on area id), removing noise.
/// filter is usually applied after applying area id's using functions
/// such as #markBoxArea, #markConvexPolyArea, and #markCylinderArea.
bool medianFilterWalkableArea(CompactHeightfield compactHeightfield){
  final xSize = compactHeightfield.width;
  final zSize = compactHeightfield.height;
  final zStride = xSize; // for readability

  // create a temporary array to store the filtered areas
  final areas = List<int>.filled(compactHeightfield.spanCount, 0xff);

  for (int z = 0; z < zSize; ++z) {
    for (int x = 0; x < xSize; ++x) {
      final cell = compactHeightfield.cells[x + z * zStride];
      final maxSpanIndex = cell.index + cell.count;

      for (int spanIndex = cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
        final span = compactHeightfield.spans[spanIndex];

        if (compactHeightfield.areas[spanIndex] == nullArea) {
          areas[spanIndex] = compactHeightfield.areas[spanIndex];
          continue;
        }

        // collect neighbor areas (including center cell)
        for (int neighborIndex = 0; neighborIndex < 9; ++neighborIndex) {
          _neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
        }

        // check all 4 cardinal directions
        for (int dir = 0; dir < 4; ++dir) {
          if (getCon(span, dir) == notConnected) {
            continue;
          }

          final aX = x + dirOffsets[dir][0];
          final aZ = z + dirOffsets[dir][1];
          final aIndex = compactHeightfield.cells[aX + aZ * zStride].index + getCon(span, dir);

          if (compactHeightfield.areas[aIndex] != nullArea) {
            _neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
          }

          // check diagonal neighbor
          final aSpan = compactHeightfield.spans[aIndex];
          final dir2 = (dir + 1) & 0x3;
          final neighborConnection2 = getCon(aSpan, dir2);

          if (neighborConnection2 != notConnected) {
            final bX = aX + dirOffsets[dir2][0];
            final bZ = aZ + dirOffsets[dir2][1];
            final bIndex = compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;

            if (compactHeightfield.areas[bIndex] != nullArea) {
              _neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
            }
          }
        }

        // sort and take median (middle value)
        insertSort(_neighborAreas, 9);
        areas[spanIndex] = _neighborAreas[4];
      }
    }
  }

  // Copy filtered areas back to the heightfield
  for (int i = 0; i < compactHeightfield.spanCount; ++i) {
    compactHeightfield.areas[i] = areas[i];
  }

  return true;
}
