import 'dart:math' as math;
import 'index.dart';

const int logNbStacks = 3;
const int nbStacks = 1 << logNbStacks;
const int expandIters = 8;

/// Calculate distance field using a two-pass distance transform algorithm
double calculateDistanceField(CompactHeightfield compactHeightfield, List<double> distances) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;

  // initialize distance values to maximum
  distances.fillRange(0, distances.length, 0xffff);

    // mark boundary cells
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; ++i) {
        final span = compactHeightfield.spans[i];
        final area = compactHeightfield.areas[i];

        int neighborCount = 0;
        for (int dir = 0; dir < 4; ++dir) {
          if (getCon(span, dir) != notConnected) {
            final ax = x + dirOffsets[dir][0];
            final ay = y + dirOffsets[dir][1];
            final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
            if (area == compactHeightfield.areas[ai]) {
              neighborCount++;
            }
          }
        }
        if (neighborCount != 4) {
          distances[i] = 0;
        }
      }
    }
  }

  // pass 1: forward pass
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; ++i) {
        final span = compactHeightfield.spans[i];

        if (getCon(span, 0) != notConnected) {
          // (-1,0) - west
          final ax = x + dirOffsets[0][0];
          final ay = y + dirOffsets[0][1];
          final ai = compactHeightfield.cells[ax + ay * w ].index + getCon(span, 0);
          final aSpan = compactHeightfield.spans[ai];
          if (distances[ai] + 2 < distances[i]) {
            distances[i] = distances[ai] + 2;
          }

          // (-1,-1) - northwest
          if (getCon(aSpan, 3) != notConnected) {
            final aax = ax + dirOffsets[3][0];
            final aay = ay + dirOffsets[3][1];
            final aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 3);
            if (distances[aai] + 3 < distances[i]) {
              distances[i] = distances[aai] + 3;
            }
          }
        }

        if (getCon(span, 3) != notConnected) {
          // (0,-1) - north
          final ax = x + dirOffsets[3][0];
          final ay = y + dirOffsets[3][1];
          final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
          final aSpan = compactHeightfield.spans[ai];
          if (distances[ai] + 2 < distances[i]) {
            distances[i] = distances[ai] + 2;
          }

          // (1,-1) - northeast
          if (getCon(aSpan, 2) != notConnected) {
            final aax = ax + dirOffsets[2][0];
            final aay = ay + dirOffsets[2][1];
            final aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 2);
            if (distances[aai] + 3 < distances[i]) {
              distances[i] = distances[aai] + 3;
            }
          }
        }
      }
    }
  }

  // pass 2: backward pass
  for (int y = h - 1; y >= 0; --y) {
    for (int x = w - 1; x >= 0; --x) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; ++i) {
        final span = compactHeightfield.spans[i];

        if (getCon(span, 2) != notConnected) {
          // (1,0) - east
          final ax = x + dirOffsets[2][0];
          final ay = y + dirOffsets[2][1];
          final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 2);
          final aSpan = compactHeightfield.spans[ai];
          if (distances[ai] + 2 < distances[i]) {
            distances[i] = distances[ai] + 2;
          }

          // (1,1) - southeast
          if (getCon(aSpan, 1) != notConnected) {
            final aax = ax + dirOffsets[1][0];
            final aay = ay + dirOffsets[1][1];
            final aai = compactHeightfield.cells[aax + aay * w].index + getCon(aSpan, 1);
            if (distances[aai] + 3 < distances[i]) {
              distances[i] = distances[aai] + 3;
            }
          }
        }

        if (getCon(span, 1) != notConnected) {
          // (0,1) - south
          final ax = x + dirOffsets[1][0];
          final ay = y + dirOffsets[1][1];
          final ai = compactHeightfield.cells[ax + ay * w ].index + getCon(span, 1);
          final aSpan = compactHeightfield.spans[ai];
          if (distances[ai] + 2 < distances[i]) {
            distances[i] = distances[ai] + 2;
          }

          // (-1,1) - southwest
          if (getCon(aSpan, 0) != notConnected) {
            final aax = ax + dirOffsets[0][0];
            final aay = ay + dirOffsets[0][1];
            final aai = compactHeightfield.cells[aax + aay * w ].index + getCon(aSpan, 0);
            if (distances[aai] + 3 < distances[i]) {
              distances[i] = distances[aai] + 3;
            }
          }
        }
      }
    }
  }

  // find maximum distance
  double maxDist = 0;
  for (int i = 0; i < compactHeightfield.spanCount; ++i) {
    maxDist = math.max<double>(distances[i], maxDist);
  }

  return maxDist;
}

/// Apply box blur filter to smooth distance values
void boxBlur(
  CompactHeightfield compactHeightfield,
  int threshold,
  List<double> srcDistances,
  List<double> dstDistances,
) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;

  final scaledThreshold = threshold * 2;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; ++i) {
        final span = compactHeightfield.spans[i];
        final cd = srcDistances[i];

        if (cd <= scaledThreshold) {
          dstDistances[i] = cd;
          continue;
        }

        double d = cd;
        for (int dir = 0; dir < 4; ++dir) {
          if (getCon(span, dir) != notConnected) {
            final ax = x + dirOffsets[dir][0];
            final ay = y + dirOffsets[dir][1];
            final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
            d += srcDistances[ai];

            final aSpan = compactHeightfield.spans[ai];
            final dir2 = (dir + 1) & 0x3;
            if (getCon(aSpan, dir2) != notConnected) {
              final ax2 = ax + dirOffsets[dir2][0];
              final ay2 = ay + dirOffsets[dir2][1];
              final ai2 = compactHeightfield.cells[ax2 + ay2 * w].index + getCon(aSpan, dir2);
              d += srcDistances[ai2];
            } 
            else {
              d += cd;
            }
          } 
          else {
            d += cd * 2;
          }
        }
        dstDistances[i] = (d + 5) / 9;
      }
    }
  }
}

void buildDistanceField(CompactHeightfield compactHeightfield) {
  // create temporary array for blurring
  final tempDistances = List<double>.filled(compactHeightfield.spanCount, 0);

  // calculate distance field directly into the heightfield's distances array
  final maxDist = calculateDistanceField(compactHeightfield, compactHeightfield.distances!);
  compactHeightfield.maxDistance = maxDist;

  // apply box blur
  boxBlur(compactHeightfield, 1, compactHeightfield.distances!, tempDistances);

  // copy the box blur result back to the heightfield
  for (int i = 0; i < compactHeightfield.spanCount; i++) {
    compactHeightfield.distances?[i] = tempDistances[i];
  }
}

class LevelStackEntry {
  int x;
  int y;
  int index;

  LevelStackEntry(this.x, this.y, this.index);
}

bool buildRegions(
  BuildContextState ctx,
  CompactHeightfield compactHeightfield,
  int borderSize,
  double minRegionArea,
  double mergeRegionArea,
) {
  // region building constants
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;

  // initialize region and distance buffers
  final srcReg = List<int>.filled(compactHeightfield.spanCount, 0, growable: true);
  final srcDist = List<int>.filled(compactHeightfield.spanCount, 0, growable: true);

  int regionId = 1;
  int level = (compactHeightfield.maxDistance + 1).toInt() & ~1;

  // initialize level stacks
  final List<List<LevelStackEntry>> lvlStacks = [];//List<List<LevelStackEntry>>.filled(nbStacks, []);
  for (int i = 0; i < nbStacks; i++) {
    lvlStacks.add([]);
  }
  final List<LevelStackEntry> stack = [];//List<LevelStackEntry>.filled(nbStacks, new LevelStackEntry(0,0,0), growable: true);
  for (int i = 0; i < nbStacks; i++) {
    stack.add(LevelStackEntry(0,0,0));
  }

  // paint border regions if border size is specified
  if (borderSize > 0) {
    final bw = math.min(w, borderSize);
    final bh = math.min(h, borderSize);

    // paint border rectangles
    paintRectRegion(0, bw, 0, h, regionId | borderReg, compactHeightfield, srcReg);
    regionId++;
    paintRectRegion(w - bw, w, 0, h, regionId | borderReg, compactHeightfield, srcReg);
    regionId++;
    paintRectRegion(0, w, 0, bh, regionId | borderReg, compactHeightfield, srcReg);
    regionId++;
    paintRectRegion(0, w, h - bh, h, regionId | borderReg, compactHeightfield, srcReg);
    regionId++;
  }

  compactHeightfield.borderSize = borderSize.toDouble();

  int sId = -1;
  while (level > 0) {
    level = level >= 2 ? level - 2 : 0;
    sId = (sId + 1) & (nbStacks - 1);

    if (sId == 0) {
      sortCellsByLevel(level, compactHeightfield, srcReg, nbStacks, lvlStacks, 1);
    } else {
      appendStacks(lvlStacks[sId - 1], lvlStacks[sId], srcReg);
    }

    // expand current regions until no empty connected cells found
    expandRegions(expandIters, level, compactHeightfield, srcReg, srcDist, lvlStacks[sId], false);

    // mark new regions with IDs
    for (int j = 0; j < lvlStacks[sId].length; j++) {
      final current = lvlStacks[sId][j];
      final x = current.x;
      final y = current.y;
      final i = current.index;

      if (i >= 0 && srcReg[i] == 0) {
        if (floodRegion(x, y, i, level, regionId, compactHeightfield, srcReg, srcDist, stack)) {
          if (regionId == 0xffff) {
            print('Region ID overflow');
            return false;
          }
          regionId++;
        }
      }
    }
  }

  // expand current regions until no empty connected cells found
  expandRegions(expandIters * 8, 0, compactHeightfield, srcReg, srcDist, stack, true);

  // merge regions and filter out small regions
  final overlaps = <int>[];
  compactHeightfield.maxRegions = regionId;

  if (!mergeAndFilterRegions(minRegionArea, mergeRegionArea, compactHeightfield, srcReg, overlaps)) {
    print('Failed to merge and filter regions');
    return false;
  }

  // write the result to compact heightfield spans
  for (int i = 0; i < compactHeightfield.spanCount; i++) {
    compactHeightfield.spans[i].region = srcReg[i];
  }

  return true;
}

/// Paint a rectangular region with the given region ID
void paintRectRegion(
  int minx,
  int maxx,
  int miny,
  int maxy,
  int regId,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
) {
  final w = compactHeightfield.width;
  for (int y = miny; y < maxy; y++) {
    for (int x = minx; x < maxx; x++) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; i++) {
        if (compactHeightfield.areas[i] != nullArea) {
          srcReg[i] = regId;
        }
      }
    }
  }
}

/// Sort cells by their distance level into stacks
void sortCellsByLevel(
  int startLevel,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  int nbStacks,
  List<List<LevelStackEntry>> stacks,
  int logLevelsPerStack,
) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;
  final adjustedStartLevel = startLevel >> logLevelsPerStack;

  // clear all stacks
  for (int j = 0; j < nbStacks; j++) {
    stacks[j].clear();
  }

  // put all cells in the level range into appropriate stacks
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      final cell = compactHeightfield.cells[x + y * w];
      for (int i = cell.index; i < cell.index + cell.count; i++) {
        if (compactHeightfield.areas[i] == nullArea || srcReg[i] != 0) {
          continue;
        }

        final level = compactHeightfield.distances![i].toInt() >> logLevelsPerStack;
        int sId = adjustedStartLevel - level;
        if (sId >= nbStacks) {
          continue;
        }
        if (sId < 0) {
          sId = 0;
        }

        stacks[sId].add(LevelStackEntry(x, y, i));
      }
    }
  }
}

/// Append entries from source stack to destination stack
void appendStacks(
  List<LevelStackEntry> srcStack,
  List<LevelStackEntry> dstStack,
  List<int> srcReg,
) {
  for (int j = 0; j < srcStack.length; j++) {
    final entry = srcStack[j];
    if (entry.index < 0 || srcReg[entry.index] != 0) {
      continue;
    }
    dstStack.add(entry);
  }
}

/// Flood fill a region starting from a given point
bool floodRegion(
  int x,
  int y,
  int i,
  int level,
  int r,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  List<int> srcDist,
  List<LevelStackEntry> stack,
){
    final w = compactHeightfield.width;
    final area = compactHeightfield.areas[i];

    // flood fill mark region
    stack.clear();
    stack.add(LevelStackEntry(x, y, i));
    srcReg[i] = r;
    srcDist[i] = 0;

    final lev = level >= 2 ? level - 2 : 0;
    int count = 0;

    while (stack.isNotEmpty) {
        final current = stack.removeLast();
        final cx = current.x;
        final cy = current.y;
        final ci = current.index;

        final span = compactHeightfield.spans[ci];

        // check if any neighbors already have a valid region set
        int ar = 0;
        for (int dir = 0; dir < 4; dir++) {
            if (getCon(span, dir) != notConnected) {
                final ax = cx + dirOffsets[dir][0];
                final ay = cy + dirOffsets[dir][1];
                final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);

                if (compactHeightfield.areas[ai] != area) {
                    continue;
                }

                final nr = srcReg[ai];
                if ((nr & borderReg) != 0) {
                    continue;
                }
                if (nr != 0 && nr != r) {
                    ar = nr;
                    break;
                }

                final aSpan = compactHeightfield.spans[ai];
                final dir2 = (dir + 1) & 0x3;
                if (getCon(aSpan, dir2) != notConnected) {
                    final ax2 = ax + dirOffsets[dir2][0];
                    final ay2 = ay + dirOffsets[dir2][1];
                    final ai2 = compactHeightfield.cells[ax2 + ay2 * w].index + getCon(aSpan, dir2);

                    if (compactHeightfield.areas[ai2] != area) {
                        continue;
                    }

                    final nr2 = srcReg[ai2];
                    if (nr2 != 0 && nr2 != r) {
                        ar = nr2;
                        break;
                    }
                }
            }
        }

        if (ar != 0) {
            srcReg[ci] = 0;
            continue;
        }

        count++;

        // expand neighbors
        for (int dir = 0; dir < 4; dir++) {
            if (getCon(span, dir) != notConnected ) {
                final ax = cx + dirOffsets[dir][0];
                final ay = cy + dirOffsets[dir][1];
                final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);

                if (compactHeightfield.areas[ai] != area) {
                    continue;
                }
                if (compactHeightfield.distances![ai] >= lev && srcReg[ai] == 0) {
                    srcReg[ai] = r;
                    srcDist[ai] = 0;
                    stack.add(LevelStackEntry(ax, ay, ai));
                }
            }
        }
    }

    return count > 0;
}

class DirtyEntry {
    final int index;
    final int region;
    final int distance2;

    DirtyEntry(this.index, this.region, this.distance2);
}

/// Expand regions iteratively
void expandRegions(
  int maxIter,
  int level,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  List<int> srcDist,
  List<LevelStackEntry> stack,
  bool fillStack,
) {
    final w = compactHeightfield.width;
    final h = compactHeightfield.height;

    if (fillStack) {
      // find cells revealed by the raised level
      stack.clear();
      for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
          final cell = compactHeightfield.cells[x + y * w];
          for (int i = cell.index; i < cell.index + cell.count; i++) {
            if (
              compactHeightfield.distances![i] >= level &&
              srcReg[i] == 0 &&
              compactHeightfield.areas[i] != nullArea
            ) {
              stack.add(LevelStackEntry(x, y, i));
            }
          }
        }
      }
    } else {
      // mark cells which already have a region
      for (int j = 0; j < stack.length; j++) {
        final i = stack[j].index;
        if (srcReg[i] != 0) {
          stack[j].index = -1;
        }
      }
    }

    final dirtyEntries = <DirtyEntry>[];
    int iter = 0;

    while (stack.isNotEmpty) {
        int failed = 0;
        dirtyEntries.clear();

        for (int j = 0; j < stack.length; j++) {
            final x = stack[j].x;
            final y = stack[j].y;
            final i = stack[j].index;

            if (i < 0) {
                failed++;
                continue;
            }

            int r = srcReg[i];
            int d2 = 0xffff;
            final area = compactHeightfield.areas[i];
            final span = compactHeightfield.spans[i];

            for (int dir = 0; dir < 4; dir++) {
                if (getCon(span, dir) == notConnected) continue;

                final ax = x + dirOffsets[dir][0];
                final ay = y + dirOffsets[dir][1];
                final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);

                if (compactHeightfield.areas[ai] != area) continue;

                if (srcReg[ai] > 0 && (srcReg[ai] & borderReg) == 0) {
                    if (srcDist[ai] + 2 < d2) {
                        r = srcReg[ai];
                        d2 = srcDist[ai] + 2;
                    }
                }
            }

            if (r != 0) {
                stack[j].index = -1; // mark as used
                dirtyEntries.add(DirtyEntry(i, r, d2));
            } else {
                failed++;
            }
        }

        // copy entries that differ to keep them in sync
        for (int i = 0; i < dirtyEntries.length; i++) {
            final entry = dirtyEntries[i];
            srcReg[entry.index] = entry.region;
            srcDist[entry.index] = entry.distance2;
        }

        if (failed == stack.length) {
            break;
        }

        if (level > 0) {
            iter++;
            if (iter >= maxIter) {
                break;
            }
        }
    }
}

/// Region data structure for merging and filtering
class Region {
  int spanCount;
  int id;
  int areaType;
  bool remap;
  bool visited;
  bool overlap;
  bool connectsToBorder;
  int ymin;
  int ymax;
  late final List<int> connections;
  late final List<int> floors;
  
  Region({
    this.spanCount = 0,
    this.id = 0,
    this.areaType = 0,
    this.remap = false,
    this.visited = false,
    this.overlap = false,
    this.connectsToBorder = false,
    this.ymin = 0xffff,
    this.ymax = 0,
    List<int>? connections,
    List<int>? floors,
  }){
    this.connections = connections ?? [];
    this.floors = floors ?? [];
  }
}

/// Merge and filter regions based on size criteria
bool mergeAndFilterRegions(
  double minRegionArea,
  double mergeRegionSize,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  List<int> overlaps,
) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;
  final nreg = compactHeightfield.maxRegions + 1;

  // construct regions
  final regions = <Region>[];
  for (int i = 0; i < nreg; i++) {
    regions.add(Region(
      spanCount: 0,
      id: i,
      areaType: 0,
      remap: false,
      visited: false,
      overlap: false,
      connectsToBorder: false,
      ymin: 0xffff,
      ymax: 0,
      connections: [],
      floors: [],
    ));
  }

    // find edge of a region and find connections around the contour
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            final cell = compactHeightfield.cells[x + y * w];
            for (int i = cell.index; i < cell.index + cell.count; i++) {
                final r = srcReg[i];
                if (r == 0 || r >= nreg) continue;

                final reg = regions[r];
                reg.spanCount++;

                // update floors
                for (int j = cell.index; j < cell.index + cell.count; j++) {
                    if (i == j) continue;
                    final floorId = srcReg[j];
                    if (floorId == 0 || floorId >= nreg) continue;
                    if (floorId == r) {
                        reg.overlap = true;
                    }
                    addUniqueFloorRegion(reg, floorId);
                }

                // have found contour
                if (reg.connections.isNotEmpty) continue;

                reg.areaType = compactHeightfield.areas[i];

                // check if this cell is next to a border
                int ndir = -1;
                for (int dir = 0; dir < 4; dir++) {
                    if (isSolidEdge(compactHeightfield, srcReg, x, y, i, dir)) {
                        ndir = dir;
                        break;
                    }
                }

                if (ndir != -1) {
                    // the cell is at border - walk around the contour to find all neighbors
                    _walkContour(x, y, i, ndir, compactHeightfield, srcReg, reg.connections);
                }
            }
        }
    }

    // remove too small regions
    final stack = <int>[];
    final trace = <int>[];

    for (int i = 0; i < nreg; i++) {
        final reg = regions[i];
        if (reg.id == 0 || (reg.id & borderReg) != 0) continue;
        if (reg.spanCount == 0) continue;
        if (reg.visited) continue;

        // count the total size of all connected regions
        bool connectsToBorder = false;
        int spanCount = 0;
        stack.clear();
        trace.clear();

        reg.visited = true;
        stack.add(i);

        while (stack.isNotEmpty) {
            final ri = stack.removeLast();
            final creg = regions[ri];

            spanCount += creg.spanCount;
            trace.add(ri);

            for (int j = 0; j < creg.connections.length; j++) {
                if (creg.connections[j] & borderReg != 0) {
                    connectsToBorder = true;
                    continue;
                }
                final neireg = regions[creg.connections[j]];
                if (neireg.visited) continue;
                if (neireg.id == 0 || neireg.id & borderReg != 0) continue;

                stack.add(neireg.id);
                neireg.visited = true;
            }
        }

        // if the accumulated region size is too small, remove it
        if (spanCount < minRegionArea && !connectsToBorder) {
            for (int j = 0; j < trace.length; j++) {
                regions[trace[j]].spanCount = 0;
                regions[trace[j]].id = 0;
            }
        }
    }

    // merge too small regions to neighbor regions
    int mergeCount = 0;
    do {
        mergeCount = 0;
        for (int i = 0; i < nreg; i++) {
            final reg = regions[i];
            if (reg.id == 0 || (reg.id & borderReg) != 0) continue;
            if (reg.overlap) continue;
            if (reg.spanCount == 0) continue;

            // check to see if the region should be merged
            if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg)) {
                continue;
            }

            // find smallest neighbor region that connects to this one
            int smallest = 0xfffffff;
            int mergeId = reg.id;
            for (int j = 0; j < reg.connections.length; j++) {
                if ((reg.connections[j] & borderReg) != 0) continue;
                final mreg = regions[reg.connections[j]];
                if (mreg.id == 0 || (mreg.id & borderReg) != 0 || mreg.overlap) continue;
                if (mreg.spanCount < smallest && canMergeWithRegion(reg, mreg) && canMergeWithRegion(mreg, reg)) {
                    smallest = mreg.spanCount;
                    mergeId = mreg.id;
                }
            }

            // found new id
            if (mergeId != reg.id) {
                final oldId = reg.id;
                final target = regions[mergeId];

                // merge neighbors
                if (mergeRegions(target, reg)) {
                    // fixup regions pointing to current region
                    for (int j = 0; j < nreg; j++) {
                        if (regions[j].id == 0 || (regions[j].id & borderReg) != 0) continue;
                        if (regions[j].id == oldId) {
                            regions[j].id = mergeId;
                        }
                        replaceNeighbor(regions[j], oldId, mergeId);
                    }
                    mergeCount++;
                }
            }
        }
    } while (mergeCount > 0);

    // compress region IDs
    for (int i = 0; i < nreg; i++) {
        regions[i].remap = false;
        if (regions[i].id == 0) continue;
        if ((regions[i].id & borderReg) != 0) continue;
        regions[i].remap = true;
    }

    int regIdGen = 0;
    for (int i = 0; i < nreg; i++) {
        if (!regions[i].remap) continue;
        final oldId = regions[i].id;
        final newId = ++regIdGen;
        for (int j = i; j < nreg; j++) {
            if (regions[j].id == oldId) {
                regions[j].id = newId;
                regions[j].remap = false;
            }
        }
    }
    compactHeightfield.maxRegions = regIdGen;

    // remap regions
    for (int i = 0; i < compactHeightfield.spanCount; i++) {
        if ((srcReg[i] & borderReg) == 0) {
            srcReg[i] = regions[srcReg[i]].id;
        }
    }

    // return regions that we found to be overlapping
    for (int i = 0; i < nreg; i++) {
        if (regions[i].overlap) {
            overlaps.add(regions[i].id);
        }
    }

    return true;
}

/// helper functions for region merging and filtering
void addUniqueFloorRegion(Region reg, int n) {
  for (int i = 0; i < reg.floors.length; i++) {
    if (reg.floors[i] == n) return;
  }
  reg.floors.add(n);
}

bool isRegionConnectedToBorder(Region reg) {
  for (int i = 0; i < reg.connections.length; i++) {
    if (reg.connections[i] == 0) return true;
  }
  return false;
}

bool canMergeWithRegion(Region rega, Region regb) {
    if (rega.areaType != regb.areaType) return false;
    int n = 0;
    for (int i = 0; i < rega.connections.length; i++) {
        if (rega.connections[i] == regb.id) n++;
    }
    if (n > 1) return false;
    for (int i = 0; i < rega.floors.length; i++) {
        if (rega.floors[i] == regb.id) return false;
    }
    return true;
}

bool mergeRegions(Region rega, Region regb) {
  final aid = rega.id;
  final bid = regb.id;

  // duplicate current neighborhood
  final acon = List<int>.from(rega.connections);
  final bcon = regb.connections;

  // find insertion point on A
  int insa = -1;
  for (int i = 0; i < acon.length; i++) {
    if (acon[i] == bid) {
      insa = i;
      break;
    }
  }
  if (insa == -1) return false;

  // find insertion point on B
  int insb = -1;
  for (int i = 0; i < bcon.length; i++) {
    if (bcon[i] == aid) {
      insb = i;
      break;
    }
  }
  if (insb == -1) return false;

  // merge neighbors
  for (int i = 0; i < acon.length - 1; i++) {
    rega.connections.add(acon[(insa + 1 + i) % acon.length]);
  }
  for (int i = 0; i < bcon.length - 1; i++) {
    rega.connections.add(bcon[(insb + 1 + i) % bcon.length]);
  }

  removeAdjacentNeighbors(rega);

  for (int j = 0; j < regb.floors.length; j++) {
    addUniqueFloorRegion(rega, regb.floors[j]);
  }
  rega.spanCount += regb.spanCount;
  regb.spanCount = 0;

  return true;
}

void removeAdjacentNeighbors(Region reg) {
  // remove adjacent duplicates
  for (int i = 0; i < reg.connections.length && reg.connections.length > 1; ) {
    final ni = (i + 1) % reg.connections.length;
    if (reg.connections[i] == reg.connections[ni]) {
      // remove duplicate
      for (int j = i; j < reg.connections.length - 1; j++) {
        reg.connections[j] = reg.connections[j + 1];
      }
      reg.connections.removeLast();
    } 
    else {
      i++;
    }
  }
}

void replaceNeighbor(Region reg, int oldId, int newId) {
  bool neiChanged = false;
  for (int i = 0; i < reg.connections.length; i++) {
    if (reg.connections[i] == oldId) {
      reg.connections[i] = newId;
      neiChanged = true;
    }
  }
  for (int i = 0; i < reg.floors.length; i++) {
    if (reg.floors[i] == oldId) {
      reg.floors[i] = newId;
    }
  }
  if (neiChanged) {
    removeAdjacentNeighbors(reg);
  }
}

bool isSolidEdge(
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  int x,
  int y,
  int i,
  int dir,
) {
  final span = compactHeightfield.spans[i];
  int r = 0;
  if (getCon(span, dir) != notConnected) {
    final ax = x + dirOffsets[dir][0];
    final ay = y + dirOffsets[dir][1];
    final ai = compactHeightfield.cells[ax + ay * compactHeightfield.width.toInt()].index + getCon(span, dir);
    r = srcReg[ai];
  }
  if (r == srcReg[i]) return false;
  return true;
}

void _walkContour(
  int x,
  int y,
  int i,
  int dir,
  CompactHeightfield compactHeightfield,
  List<int> srcReg,
  List<int> cont,
) {
  final startDir = dir;
  final starti = i;

  final ss = compactHeightfield.spans[i];
  int curReg = 0;
  if (getCon(ss, dir) != notConnected) {
    final ax = x + dirOffsets[dir][0];
    final ay = y + dirOffsets[dir][1];
    final ai = compactHeightfield.cells[ax + ay * compactHeightfield.width.toInt()].index + getCon(ss, dir);
    curReg = srcReg[ai];
  }
  cont.add(curReg);

  int iter = 0;
  int currentX = x;
  int currentY = y;
  int currentI = i;
  int currentDir = dir;

  while (++iter < 40000) {
    final s = compactHeightfield.spans[currentI];

    if (isSolidEdge(compactHeightfield, srcReg, currentX, currentY, currentI, currentDir)) {
      // choose the edge corner
      int r = 0;
      if (getCon(s, currentDir) != notConnected) {
        final ax = currentX + dirOffsets[currentDir][0];
        final ay = currentY + dirOffsets[currentDir][1];
        final ai = compactHeightfield.cells[ax + ay * compactHeightfield.width.toInt()].index + getCon(s, currentDir);
        r = srcReg[ai];
      }
      if (r != curReg) {
        curReg = r;
        cont.add(curReg);
      }

      currentDir = (currentDir + 1) & 0x3; // rotate CW
    } 
    else {
      int ni = -1;
      final nx = currentX + dirOffsets[currentDir][0];
      final ny = currentY + dirOffsets[currentDir][1];
      if (getCon(s, currentDir) != notConnected) {
        final nc = compactHeightfield.cells[nx + ny * compactHeightfield.width.toInt()];
        ni = nc.index + getCon(s, currentDir);
      }
      if (ni == -1) {
        //BuildContextState.warn(ctx, 'walkContour: encountered unexpected disconnected neighbour at (${currentX}, ${currentY})');
        // should not happen
        return;
      }
      currentX = nx;
      currentY = ny;
      currentI = ni;
      currentDir = (currentDir + 3) & 0x3; // rotate CCW
    }

    if (starti == currentI && startDir == currentDir) {
      break;
    }
  }

  // remove adjacent duplicates
  if (cont.length > 1) {
    for (int j = 0; j < cont.length; ) {
      final nj = (j + 1) % cont.length;
      if (cont[j] == cont[nj]) {
        for (int k = j; k < cont.length - 1; k++) {
          cont[k] = cont[k + 1];
        }
        cont.removeLast();
      } 
      else {
        j++;
      }
    }
  }
}

const nullNei = 0xffff;

class SweepSpan {
  /// row id 
  int rid;
  /// region id 
  int id;
  /// number of samples 
  int ns;
  /// neighbour id 
  int nei;

  SweepSpan({required this.rid, required this.id, required this.ns, required this.nei});
}

/// Build regions using monotone partitioning algorithm.
/// This is an alternative to the watershed-based buildRegions function.
/// Monotone partitioning creates regions by sweeping the heightfield and
/// does not generate overlapping regions.
bool buildRegionsMonotone(
  CompactHeightfield compactHeightfield,
  int borderSize,
  double minRegionArea,
  double mergeRegionArea,
) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;
  int id = 1;

  final srcReg = List<int>.filled(compactHeightfield.spanCount, 0);
  final nsweeps = math.max(compactHeightfield.width.toInt(), compactHeightfield.height.toInt());
  final  List<SweepSpan> sweeps = [];//List<SweepSpan>.filled(nsweeps, new SweepSpan(rid: 0, id: 0, ns: 0, nei: 0));
  for (int i = 0; i < nsweeps; i++) {
    sweeps.add(SweepSpan(rid: 0, id: 0, ns: 0, nei: 0));
  }
  // mark border regions
  if (borderSize > 0) {
    final bw = math.min(w, borderSize);
    final bh = math.min(h, borderSize);

    paintRectRegion(0, bw, 0, h, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(w - bw, w, 0, h, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(0, w, 0, bh, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(0, w, h - bh, h, id | borderReg, compactHeightfield, srcReg);
    id++;
  }

    compactHeightfield.borderSize = borderSize.toDouble();

    final prev = List<int>.filled(256, 0);

    // sweep one line at a time
    for (int y = borderSize.toInt(); y < h - borderSize; y++) {
        // collect spans from this row
        if (prev.length < id + 1) {
            prev.length = id + 1;
        }
        prev.fillRange(0, id, 0);
        int rid = 1;

        for (int x = borderSize.toInt(); x < w - borderSize; x++) {
            final cell = compactHeightfield.cells[x + y * w];

            for (int i = cell.index; i < cell.index + cell.count; i++) {
                final span = compactHeightfield.spans[i];
                if (compactHeightfield.areas[i] == nullArea) continue;

                // check -x direction
                int previd = 0;
                if (getCon(span, 0) != notConnected) {
                    final ax = x + dirOffsets[0][0];
                    final ay = y + dirOffsets[0][1];
                    final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 0);
                    if ((srcReg[ai] & borderReg) == 0 && compactHeightfield.areas[i] == compactHeightfield.areas[ai]) {
                        previd = srcReg[ai];
                    }
                }

                if (previd == 0) {
                    previd = rid++;
                    sweeps[previd].rid = previd;
                    sweeps[previd].ns = 0;
                    sweeps[previd].nei = 0;
                }

                // check -y direction
                if (getCon(span, 3) != notConnected) {
                    final ax = x + dirOffsets[3][0];
                    final ay = y + dirOffsets[3][1];
                    final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
                    if (
                        srcReg[ai] != 0 &&
                        (srcReg[ai] & borderReg) == 0 &&
                        compactHeightfield.areas[i] == compactHeightfield.areas[ai]
                    ) {
                        final nr = srcReg[ai];
                        if (sweeps[previd].nei == 0 || sweeps[previd].nei == nr) {
                            sweeps[previd].nei = nr;
                            sweeps[previd].ns++;
                            prev[nr]++;
                        } else {
                            sweeps[previd].nei = nullNei;
                        }
                    }
                }

                srcReg[i] = previd;
            }
        }

        // create unique ID
        for (int i = 1; i < rid; i++) {
            if (sweeps[i].nei != nullNei && sweeps[i].nei != 0 && prev[sweeps[i].nei] == sweeps[i].ns) {
                sweeps[i].id = sweeps[i].nei;
            } else {
                sweeps[i].id = id++;
            }
        }

        // remap IDs
        for (int x = borderSize.toInt(); x < w - borderSize; x++) {
            final cell = compactHeightfield.cells[x + y * w];

            for (int i = cell.index; i < cell.index + cell.count; i++) {
                if (srcReg[i] > 0 && srcReg[i] < rid) {
                    srcReg[i] = sweeps[srcReg[i]].id;
                }
            }
        }
    }

    // merge regions and filter out small regions
    final overlaps = <int>[];
    compactHeightfield.maxRegions = id;

    if (!mergeAndFilterRegions(minRegionArea, mergeRegionArea, compactHeightfield, srcReg, overlaps)) {
        return false;
    }

    // store the result
    for (int i = 0; i < compactHeightfield.spanCount; i++) {
        compactHeightfield.spans[i].region = srcReg[i];
    }

    return true;
}

/// Add unique connection to region
void addUniqueConnection(Region reg, int n) {
    for (int i = 0; i < reg.connections.length; i++) {
        if (reg.connections[i] == n) return;
    }
    reg.connections.add(n);
}

/// Merge and filter layer regions
bool mergeAndFilterLayerRegions(
    double minRegionArea,
    CompactHeightfield compactHeightfield,
    List<int> srcReg,
    Nint maxRegionId
) {
    final w = compactHeightfield.width;
    final h = compactHeightfield.height;
    final nreg = maxRegionId.value + 1;

    // construct regions
    final regions = <Region>[];
    for (int i = 0; i < nreg; i++) {
        regions.add(Region(
            spanCount: 0,
            id: i,
            areaType: 0,
            remap: false,
            visited: false,
            overlap: false,
            connectsToBorder: false,
            ymin: 0xffff,
            ymax: 0,
            connections: [],
            floors: [],
        ));
    }

    // find region neighbours and overlapping regions
    final lregs = <int>[];
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            final cell = compactHeightfield.cells[x + y * w];
            lregs.clear();

            for (int i = cell.index; i < cell.index + cell.count; i++) {
                final span = compactHeightfield.spans[i];
                final area = compactHeightfield.areas[i];
                final ri = srcReg[i];
                if (ri == 0 || ri >= nreg) continue;
                final reg = regions[ri];

                reg.spanCount++;
                reg.areaType = area;
                reg.ymin = math.min(reg.ymin, span.y.toInt());
                reg.ymax = math.max(reg.ymax, span.y.toInt());

                // collect all region layers
                lregs.add(ri);

                // update neighbours
                for (int dir = 0; dir < 4; dir++) {
                    if (getCon(span, dir) != notConnected) {
                        final ax = x + dirOffsets[dir][0];
                        final ay = y + dirOffsets[dir][1];
                        final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, dir);
                        final rai = srcReg[ai];
                        if (rai > 0 && rai < nreg && rai != ri) {
                            addUniqueConnection(reg, rai);
                        }
                        if (rai & borderReg != 0) {
                            reg.connectsToBorder = true;
                        }
                    }
                }
            }

            // update overlapping regions
            for (int i = 0; i < lregs.length - 1; i++) {
                for (int j = i + 1; j < lregs.length; j++) {
                    if (lregs[i] != lregs[j]) {
                        final ri = regions[lregs[i]];
                        final rj = regions[lregs[j]];
                        addUniqueFloorRegion(ri, lregs[j]);
                        addUniqueFloorRegion(rj, lregs[i]);
                    }
                }
            }
        }
    }

    // create 2D layers from regions
    int layerId = 1;

    for (int i = 0; i < nreg; i++) {
        regions[i].id = 0;
    }

    // merge monotone regions to create non-overlapping areas
    final stack = <int>[];
    for (int i = 1; i < nreg; i++) {
        final root = regions[i];
        // skip already visited
        if (root.id != 0) continue;

        // start search
        root.id = layerId;
        stack.clear();
        stack.add(i);

        while (stack.isNotEmpty) {
            // pop front
            final regIndex = stack.removeAt(0);
            final reg = regions[regIndex];

            final ncons = reg.connections.length;
            for (int j = 0; j < ncons; j++) {
                final nei = reg.connections[j];
                final regn = regions[nei];
                // skip already visited
                if (regn.id != 0) continue;
                // skip if different area type
                if (reg.areaType != regn.areaType) continue;
                // skip if the neighbour is overlapping root region
                bool overlap = false;
                for (int k = 0; k < root.floors.length; k++) {
                    if (root.floors[k] == nei) {
                        overlap = true;
                        break;
                    }
                }
                if (overlap) continue;

                // deepen
                stack.add(nei);

                // mark layer id
                regn.id = layerId;
                // merge current layers to root
                for (int k = 0; k < regn.floors.length; k++) {
                    addUniqueFloorRegion(root, regn.floors[k]);
                }
                root.ymin = math.min(root.ymin, regn.ymin);
                root.ymax = math.max(root.ymax, regn.ymax);
                root.spanCount += regn.spanCount;
                regn.spanCount = 0;
                root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
            }
        }

        layerId++;
    }

    // remove small regions
    for (int i = 0; i < nreg; i++) {
        if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder) {
            final reg = regions[i].id;
            for (int j = 0; j < nreg; j++) {
                if (regions[j].id == reg) {
                    regions[j].id = 0;
                }
            }
        }
    }

    // compress region IDs
    for (int i = 0; i < nreg; i++) {
        regions[i].remap = false;
        if (regions[i].id == 0) continue;
        if (regions[i].id & borderReg != 0) continue;
        regions[i].remap = true;
    }

    int regIdGen = 0;
    for (int i = 0; i < nreg; i++) {
        if (!regions[i].remap) continue;
        final oldId = regions[i].id;
        final newId = ++regIdGen;
        for (int j = i; j < nreg; j++) {
            if (regions[j].id == oldId) {
                regions[j].id = newId;
                regions[j].remap = false;
            }
        }
    }
    maxRegionId.value = regIdGen;

    // remap regions
    for (int i = 0; i < compactHeightfield.spanCount; i++) {
        if ((srcReg[i] & borderReg) == 0) {
            srcReg[i] = regions[srcReg[i]].id;
        }
    }

    return true;
}

/// Build layer regions using sweep-line algorithm.
/// This creates regions that can be used for building navigation mesh layers.
/// Layer regions handle overlapping walkable areas by creating separate layers.
bool buildLayerRegions(CompactHeightfield compactHeightfield, int borderSize, double minRegionArea) {
  final w = compactHeightfield.width;
  final h = compactHeightfield.height;
  int id = 1;

  final srcReg = List<int>.filled(compactHeightfield.spanCount, 0);
  final nsweeps = math.max(compactHeightfield.width.toInt(), compactHeightfield.height.toInt());
  final List<SweepSpan> sweeps = [];//List<SweepSpan>.filled(nsweeps, SweepSpan(rid: 0, id: 0, ns: 0, nei: 0));

  // initialize sweeps array
  for (int i = 0; i < nsweeps; i++) {
    sweeps[i] = SweepSpan(rid: 0, id: 0, ns: 0, nei: 0);
  }

  // mark border regions
  if (borderSize > 0) {
    final bw = math.min(w, borderSize);
    final bh = math.min(h, borderSize);

    paintRectRegion(0, bw, 0, h, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(w - bw, w, 0, h, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(0, w, 0, bh, id | borderReg, compactHeightfield, srcReg);
    id++;
    paintRectRegion(0, w, h - bh, h, id | borderReg, compactHeightfield, srcReg);
    id++;
  }

  compactHeightfield.borderSize = borderSize.toDouble();

  final prev = List<int>.filled(256, 0);

    // sweep one line at a time
  for (int y = borderSize.toInt(); y < h - borderSize; y++) {
    // collect spans from this row
    if (prev.length < id + 1) {
      prev.length = id + 1;
    }
    prev.fillRange(0, id, 0);
    int rid = 1;

    for (int x = borderSize.toInt(); x < w - borderSize; x++) {
      final cell = compactHeightfield.cells[x + y * w];

      for (int i = cell.index; i < cell.index + cell.count; i++) {
        final span = compactHeightfield.spans[i];
        if (compactHeightfield.areas[i] == nullArea) continue;

        // check -x direction
        int previd = 0;
        if (getCon(span, 0) != notConnected) {
          final ax = x + dirOffsets[0][0];
          final ay = y + dirOffsets[0][1];
          final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 0);
          if ((srcReg[ai] & borderReg) == 0 && compactHeightfield.areas[i] == compactHeightfield.areas[ai]) {
            previd = srcReg[ai];
          }
        }

        if (previd == 0) {
          previd = rid++;
          sweeps[previd].rid = previd;
          sweeps[previd].ns = 0;
          sweeps[previd].nei = 0;
        }

        // check -y direction
        if (getCon(span, 3) != notConnected) {
          final ax = x + dirOffsets[3][0];
          final ay = y + dirOffsets[3][1];
          final ai = compactHeightfield.cells[ax + ay * w].index + getCon(span, 3);
          if (
            srcReg[ai] != 0 &&
            (srcReg[ai] & borderReg) == 0 &&
            compactHeightfield.areas[i] == compactHeightfield.areas[ai]
          ) {
            final nr = srcReg[ai];
          if (sweeps[previd].nei == 0 || sweeps[previd].nei == nr) {
              sweeps[previd].nei = nr;
              sweeps[previd].ns++;
              prev[nr]++;
            } 
            else {
              sweeps[previd].nei = nullNei;
            }
          }
        }

        srcReg[i] = previd;
      }
    }

    // create unique ID
    for (int i = 1; i < rid; i++) {
      if (sweeps[i].nei != nullNei && sweeps[i].nei != 0 && prev[sweeps[i].nei] == sweeps[i].ns) {
        sweeps[i].id = sweeps[i].nei;
      } 
      else {
        sweeps[i].id = id++;
      }
    }

    // remap IDs
    for (int x = borderSize.toInt(); x < w - borderSize; x++) {
      final cell = compactHeightfield.cells[x + y * w];

      for (int i = cell.index; i < cell.index + cell.count; i++) {
        if (srcReg[i] > 0 && srcReg[i] < rid) {
          srcReg[i] = sweeps[srcReg[i]].id;
        }
      }
    }
  }

  // merge monotone regions to layers and remove small regions
  compactHeightfield.maxRegions = id;
  final maxRegionIdRef = Nint(compactHeightfield.maxRegions);

  if (!mergeAndFilterLayerRegions(minRegionArea, compactHeightfield, srcReg, maxRegionIdRef)) {
    return false;
  }

  compactHeightfield.maxRegions = maxRegionIdRef.value;

  // store the result
  for (int i = 0; i < compactHeightfield.spanCount; i++) {
    compactHeightfield.spans[i].region = srcReg[i];
  }

  return true;
}
