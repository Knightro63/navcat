// Direction offsets for 4-directional neighbor access (N, E, S, W)
List<List<int>> dirOffsets = [
  // North (negative Z)
  [-1, 0],
  // East (positive X)
  [0, 1],
  // South (positive Z)
  [1, 0],
  // West (negative X)
  [0, -1],
];

int getDirOffsetX(int dir) {
  return dirOffsets[dir & 0x03][0];
}

int getDirOffsetY(int dir) {
  return dirOffsets[dir & 0x03][1];
}

int getDirForOffset(int x, int y) {
  for (int i = 0; i < dirOffsets.length; i++) {
    if (dirOffsets[i][0] == x && dirOffsets[i][1] == y) {
      return i;
    }
  }
  return 0; // Default to North if no match
}

const int axisX = 0;
const int axisY = 1;
const int axisZ = 2;

const int multipleRegs = 0;
const int meshNullIdx = -1;
const int borderVertex = 0x10000;
const int contourRegMask = 0xffff;
const int areaBorder = 0x20000;

const int nullArea = 0;
const int walkableArea = 1;
const int borderReg = 0x8000;

const int notConnected = 0x3f; // 63
const int maxHeight = 0xffff;
const int maxLayers = notConnected - 1;
const int polyNeisFlagExtLink = 0x8000;
const int polyNeisFlagExtLinkDirMask = 0xff;

class Nint{
  int value;
  Nint([this.value = 0]);
}

class BooleanRef {
  bool value;
  BooleanRef(this.value);
}