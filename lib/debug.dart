import 'package:navcat/generate/index.dart';
import 'package:navcat/query/index.dart';
import 'package:three_js_math/three_js_math.dart';
import 'dart:math' as math;

// debug primitive types
enum DebugPrimitiveType {
  triangles,
  lines,
  points,
  boxes,
}

sealed class DebugPrimitive {
  final DebugPrimitiveType type;
  final List<double> positions;
  final List<double> colors;
  final bool? transparent;
  final double? opacity;

  DebugPrimitive({
    required this.type,
    required this.positions,
    required this.colors,
    this.transparent,
    this.opacity,
  });
}

class DebugTriangles extends DebugPrimitive {
  final List<int>? indices;
  final bool? doubleSided;

  DebugTriangles({
    required super.positions,
    required super.colors,
    required this.indices,
    super.transparent,
    super.opacity,
    this.doubleSided,
  }) : super(type: DebugPrimitiveType.triangles);
}

class DebugLines extends DebugPrimitive {
  final double? lineWidth;

  DebugLines({
    required super.positions,
    required super.colors,
    this.lineWidth,
    super.transparent,
    super.opacity,
  }) : super(type: DebugPrimitiveType.lines);
}

class DebugPoints extends DebugPrimitive {
  final double? size;

  DebugPoints({
    required super.positions,
    required super.colors,
    required this.size,
    super.transparent,
    super.opacity,
  }) : super(type: DebugPrimitiveType.points);
}

class DebugBoxes extends DebugPrimitive {
  final List<double>? scales;
  final List<double>? rotations;

  DebugBoxes({
    required super.positions,
    required super.colors,
    required this.scales,
    this.rotations,
    super.transparent,
    super.opacity,
  }) : super(type: DebugPrimitiveType.boxes);
}

Vector3 hslToRgb(Vector3 out, double h, double s, double l) {
  h /= 360.0;
  final double a = s * math.min(l, 1.0 - l);
  
  double f(double n) {
    final double k = (n + h * 12.0) % 12.0;
    // math.min and math.max require double arguments
    final double val = math.min(k - 3.0, math.min(9.0 - k, 1.0));
    return l - a * math.max(val, -1.0);
  }

  // Vector3 uses x, y, z properties instead of bracket notation [0], [1], [2]
  out.x = f(0.0);
  out.y = f(8.0);
  out.z = f(4.0);
  
  return out;
}

Vector3 regionToColor(Vector3 out, int regionId, [double alpha = 1.0]) {
  if (regionId == 0) {
    out.x = 0.0;
    out.y = 0.0;
    out.z = 0.0;
    return out;
  }
  
  final double hash = regionId * 137.5;
  final double hue = hash % 360.0;
  
  hslToRgb(out, hue, 0.7, 0.6);
  
  out.x *= alpha;
  out.y *= alpha;
  out.z *= alpha;
  
  return out;
}

Vector3 areaToColor(Vector3 out, int area, [double alpha = 1.0]) {
  if (area == walkableArea) {
    out.x = 0.0;
    out.y = 192.0 / 255.0;
    out.z = 1.0;
    return out;
  }
  
  if (area == nullArea) {
    out.x = 0.0;
    out.y = 0.0;
    out.z = 0.0;
    return out;
  }
  
  final double hash = area * 137.5;
  final double hue = hash % 360.0;
  
  hslToRgb(out, hue, 0.7, 0.6);
  
  out.x *= alpha;
  out.y *= alpha;
  out.z *= alpha;
  
  return out;
}

// Helper class to match your JavaScript input object pattern
class MeshInput {
  final List<double> positions;
  final List<int> indices;

  MeshInput({required this.positions, required this.indices});
}

DebugPrimitive? createTriangleAreaIdsHelper(
  MeshInput input,
  List<int> triAreaIds,
) {
  // Maps area IDs to RGB values stored as [R, G, B] lists
  final Map<int, List<double>> areaToColorMap = {};
  
  final List<double> positions = [];
  final List<int> indices = [];
  final List<double> vertexColors = [];
  
  // Reuse a single Vector3 instance for performance inside the loop
  final Vector3 colorBuffer = Vector3();

  // JavaScript's length / 3 translates to integer division (~/) in Dart
  final int triangleCount = input.indices.length ~/ 3;

  for (int triangle = 0; triangle < triangleCount; triangle++) {
    final int areaId = triAreaIds[triangle];
    List<double>? color = areaToColorMap[areaId];

    if (color == null) {
      if (areaId == walkableArea) {
        color = [0.0, 1.0, 0.0];
      } else if (areaId == nullArea) {
        color = [1.0, 0.0, 0.0];
      } else {
        areaToColor(colorBuffer, areaId);
        color = [colorBuffer.x, colorBuffer.y, colorBuffer.z];
      }
      areaToColorMap[areaId] = color;
    }

    // Fetch vertex index mappings
    final int idx0 = input.indices[triangle * 3];
    final int idx1 = input.indices[triangle * 3 + 1];
    final int idx2 = input.indices[triangle * 3 + 2];

    // Push X, Y, Z for Vertex 0
    positions.add(input.positions[idx0 * 3].toDouble());
    positions.add(input.positions[idx0 * 3 + 1].toDouble());
    positions.add(input.positions[idx0 * 3 + 2].toDouble());

    // Push X, Y, Z for Vertex 1
    positions.add(input.positions[idx1 * 3].toDouble());
    positions.add(input.positions[idx1 * 3 + 1].toDouble());
    positions.add(input.positions[idx1 * 3 + 2].toDouble());

    // Push X, Y, Z for Vertex 2
    positions.add(input.positions[idx2 * 3].toDouble());
    positions.add(input.positions[idx2 * 3 + 1].toDouble());
    positions.add(input.positions[idx2 * 3 + 2].toDouble());

    // Push Triangle indices
    indices.add(triangle * 3);
    indices.add(triangle * 3 + 1);
    indices.add(triangle * 3 + 2);

    // Push Color components for all 3 vertices
    for (int i = 0; i < 3; i++) {
      vertexColors.add(color[0]);
      vertexColors.add(color[1]);
      vertexColors.add(color[2]);
    }
  }

  if (positions.isEmpty) {
    return null;
  }

  return DebugTriangles(
      positions: positions,
      colors: vertexColors,
      indices: indices,
      transparent: true,
      opacity: 1.0,
    );
}

DebugPrimitive? createHeightfieldHelper(Heightfield heightfield) {
  // Count total spans
  int totalSpans = 0;
  for (int z = 0; z < heightfield.height; z++) {
    for (int x = 0; x < heightfield.width; x++) {
      final int columnIndex = x + z * heightfield.width;
      HeightfieldSpan? span = heightfield.spans[columnIndex];
      while (span != null) {
        totalSpans++;
        span = span.next;
      }
    }
  }

  if (totalSpans == 0) {
    return null;
  }

  final List<double> positions = [];
  final List<double> colors = [];
  final List<double> scales = [];

  final double boundsMinX = heightfield.bounds.min[0];
  final double boundsMinY = heightfield.bounds.min[1];
  final double boundsMinZ = heightfield.bounds.min[2];

  final double cellSize = heightfield.cellSize;
  final double cellHeight = heightfield.cellHeight;
  
  final Map<int, List<double>> areaToColorMap = {};
  final Vector3 colorBuffer = Vector3();

  for (int z = 0; z < heightfield.height; z++) {
    for (int x = 0; x < heightfield.width; x++) {
      final int columnIndex = x + z * heightfield.width;
      HeightfieldSpan? span = heightfield.spans[columnIndex];
      
      while (span != null) {
        final double worldX = boundsMinX + (x + 0.5) * cellSize;
        final double worldZ = boundsMinZ + (z + 0.5) * cellSize;
        final double spanHeight = (span.max - span.min) * cellHeight;
        final double worldY = boundsMinY + (span.min + (span.max - span.min) * 0.5) * cellHeight;

        positions.addAll([worldX, worldY, worldZ]);
        scales.addAll([cellSize * 0.9, spanHeight, cellSize * 0.9]);

        List<double>? color = areaToColorMap[span.area];
        if (color == null) {
          if (span.area == walkableArea) {
            color = [0.0, 1.0, 0.0];
          } else if (span.area == nullArea) {
            color = [1.0, 0.0, 0.0];
          } else {
            areaToColor(colorBuffer, span.area);
            color = [colorBuffer.x, colorBuffer.y, colorBuffer.z];
          }
          areaToColorMap[span.area] = color;
        }

        colors.addAll(color);
        span = span.next;
      }
    }
  }

  return
    DebugBoxes(
      positions: positions,
      colors: colors,
      scales: scales,
    );
}

// Helper structural classes to match your JS CompactHeightfield pattern
class CompactCell {
  final int index;
  final int count;

  CompactCell({required this.index, required this.count});
}

DebugPrimitive? createCompactHeightfieldSolidHelper(CompactHeightfield compactHeightfield) {
  final chf = compactHeightfield;
  int totalQuads = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final cell = chf.cells[x + y * chf.width];
      totalQuads += cell.count;
    }
  }

  if (totalQuads == 0) {
    return null;
  }

  final List<double> positions = [];
  final List<int> indices = [];
  final List<double> colors = [];
  
  final Vector3 colorBuffer = Vector3();
  int indexOffset = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final fx = (chf.bounds.min[0] + x * chf.cellSize)-0.8;
      final fz = (chf.bounds.min[2] + y * chf.cellSize)-0.8;
      final cell = chf.cells[x + y * chf.width];

      for (int i = cell.index; i < cell.index + cell.count; i++) {
        final span = chf.spans[i];
        final area = chf.areas[i];
        
        areaToColor(colorBuffer, area);
        final double fy = chf.bounds.min[1] + (span.y + 1) * chf.cellHeight;

        // Create quad vertices (4 vertices per span)
        // Vertex 0
        positions.addAll([fx, fy, fz]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 1
        positions.addAll([fx, fy, fz + chf.cellSize]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 2
        positions.addAll([fx + chf.cellSize, fy, fz + chf.cellSize]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 3
        positions.addAll([fx + chf.cellSize, fy, fz]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Create triangles using the quad vertex indices
        indices.addAll([indexOffset, indexOffset + 1, indexOffset + 2]);
        indices.addAll([indexOffset, indexOffset + 2, indexOffset + 3]);
        
        indexOffset += 4;
      }
    }
  }

  return
    DebugTriangles(
      positions: positions,
      colors: colors,
      indices: indices,
      transparent: true,
      opacity: 0.6,
      doubleSided: true,
    );
}

DebugPrimitive? createCompactHeightfieldDistancesHelper(CompactHeightfield compactHeightfield) {
  final chf = compactHeightfield;
  
  // Return early if there is no distance field array present
  if (chf.distances == null) {
    return null;
  }

  int maxd = chf.maxDistance.toInt();
  if (maxd < 1.0) {
    maxd = 1;
  }
  
  final double dscale = 255.0 / maxd;
  int totalQuads = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final cell = chf.cells[x + y * chf.width];
      totalQuads += cell.count;
    }
  }

  if (totalQuads == 0) {
    return null;
  }

  final List<double> positions = [];
  final List<int> indices = [];
  final List<double> colors = [];
  
  final double boundsMinX = chf.bounds.min[0];
  final double boundsMinY = chf.bounds.min[1];
  final double boundsMinZ = chf.bounds.min[2];
  
  int indexOffset = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final double fx = boundsMinX + x * chf.cellSize;
      final double fz = boundsMinZ + y * chf.cellSize;
      final cell = chf.cells[x + y * chf.width];

      for (int i = cell.index; i < cell.index + cell.count; i++) {
        final span = chf.spans[i];
        final double fy = boundsMinY + (span.y + 1) * chf.cellHeight;
        
        // Math.floor(x) translates to (x).floor() in Dart.
        // We use math.min from dart:math (or clamp) to restrict the range.
        final int distanceVal = (chf.distances![i] * dscale).floor();
        final double cd = math.min(255, distanceVal) / 255.0;

        // Create quad vertices
        // Vertex 0
        positions.addAll([fx, fy, fz]);
        colors.addAll([cd, cd, cd]);

        // Vertex 1
        positions.addAll([fx, fy, fz + chf.cellSize]);
        colors.addAll([cd, cd, cd]);

        // Vertex 2
        positions.addAll([fx + chf.cellSize, fy, fz + chf.cellSize]);
        colors.addAll([cd, cd, cd]);

        // Vertex 3
        positions.addAll([fx + chf.cellSize, fy, fz]);
        colors.addAll([cd, cd, cd]);

        // Create triangles
        indices.addAll([indexOffset, indexOffset + 1, indexOffset + 2]);
        indices.addAll([indexOffset, indexOffset + 2, indexOffset + 3]);
        
        indexOffset += 4;
      }
    }
  }

  return
    DebugTriangles(
      positions: positions,
      colors: colors,
      indices: indices,
      transparent: true,
      opacity: 0.8,
      doubleSided: true,
    );
}

// Ensure your CompactSpan class from earlier includes the region property:
// class CompactSpan { final int y; final int region; ... }

DebugPrimitive? createCompactHeightfieldRegionsHelper(CompactHeightfield compactHeightfield) {
  final chf = compactHeightfield;
  int totalQuads = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final cell = chf.cells[x + y * chf.width];
      totalQuads += cell.count;
    }
  }

  if (totalQuads == 0) {
    return null;
  }

  final List<double> positions = [];
  final List<int> indices = [];
  final List<double> colors = [];
  
  final double boundsMinX = chf.bounds.min[0];
  final double boundsMinY = chf.bounds.min[1];
  final double boundsMinZ = chf.bounds.min[2];
  
  final Vector3 colorBuffer = Vector3();
  int indexOffset = 0;

  for (int y = 0; y < chf.height; y++) {
    for (int x = 0; x < chf.width; x++) {
      final double fx = boundsMinX + x * chf.cellSize;
      final double fz = boundsMinZ + y * chf.cellSize;
      final cell = chf.cells[x + y * chf.width];

      for (int i = cell.index; i < cell.index + cell.count; i++) {
        final span = chf.spans[i];
        final double fy = boundsMinY + span.y * chf.cellHeight;
        
        // Converts the unique region integer index into an RGB color vector
        regionToColor(colorBuffer, span.region);

        // Create quad vertices
        // Vertex 0
        positions.addAll([fx, fy, fz]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 1
        positions.addAll([fx, fy, fz + chf.cellSize]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 2
        positions.addAll([fx + chf.cellSize, fy, fz + chf.cellSize]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Vertex 3
        positions.addAll([fx + chf.cellSize, fy, fz]);
        colors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

        // Create triangles
        indices.addAll([indexOffset, indexOffset + 1, indexOffset + 2]);
        indices.addAll([indexOffset, indexOffset + 2, indexOffset + 3]);
        
        indexOffset += 4;
      }
    }
  }

  return
    DebugTriangles(
      positions: positions,
      colors: colors,
      indices: indices,
      transparent: true,
      opacity: 0.9,
      doubleSided: true,
    );
}

List<DebugPrimitive> createRawContoursHelper(ContourSet? contourSet) {
  if (contourSet == null || contourSet.contours.isEmpty) {
    return [];
  }

  final double origX = contourSet.bounds.min[0];
  final double origY = contourSet.bounds.min[1];
  final double origZ = contourSet.bounds.min[2];
  
  final double cs = contourSet.cellSize;
  final double ch = contourSet.cellHeight;

  final List<double> linePositions = [];
  final List<double> lineColors = [];
  final List<double> pointPositions = [];
  final List<double> pointColors = [];
  
  final Vector3 colorBuffer = Vector3();

  // 1. Draw lines for each contour
  for (int i = 0; i < contourSet.contours.length; ++i) {
    final c = contourSet.contours[i];
    regionToColor(colorBuffer, c.reg, 0.8);

    for (int j = 0; j < c.nRawVertices; ++j) {
      // slice() converts to sublist() in Dart
      final List<double> v = c.rawVertices.sublist(j * 4, j * 4 + 4);
      
      final double fx = origX + v[0] * cs;
      final double fy = origY + (v[1] + 1 + (i & 1)) * ch;
      final double fz = origZ + v[2] * cs;

      linePositions.addAll([fx, fy, fz]);
      lineColors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);

      if (j > 0) {
        linePositions.addAll([fx, fy, fz]);
        lineColors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);
      }
    }

    // Loop last segment back to the beginning
    if (c.nRawVertices > 0) {
      final List<double> v = c.rawVertices.sublist(0, 4);
      final double fx = origX + v[0] * cs;
      final double fy = origY + (v[1] + 1 + (i & 1)) * ch;
      final double fz = origZ + v[2] * cs;

      linePositions.addAll([fx, fy, fz]);
      lineColors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);
    }
  }

  // 2. Draw points for each contour
  for (int i = 0; i < contourSet.contours.length; ++i) {
    final c = contourSet.contours[i];
    regionToColor(colorBuffer, c.reg, 0.8);
    
    final List<double> darkenedColor = [
      colorBuffer.x * 0.5, 
      colorBuffer.y * 0.5, 
      colorBuffer.z * 0.5
    ];

    for (int j = 0; j < c.nRawVertices; ++j) {
      final List<double> v = c.rawVertices.sublist(j * 4, j * 4 + 4);
      double off = 0.0;
      List<double> colv = darkenedColor;

      // Checking for the BORDER_VERTEX bitflag mask
      if ((v[3].toInt() & 0x10000) != 0) {
        colv = [1.0, 1.0, 1.0];
        off = ch * 2.0;
      }

      final double fx = origX + v[0] * cs;
      final double fy = origY + (v[1] + 1 + (i & 1)) * ch + off;
      final double fz = origZ + v[2] * cs;

      pointPositions.addAll([fx, fy, fz]);
      pointColors.addAll([colv[0], colv[1], colv[2]]);
    }
  }

  final List<DebugPrimitive> primitives = [];

  if (linePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: linePositions,
        colors: lineColors,
        transparent: true,
        opacity: 0.8,
        lineWidth: 2.0,
      ),
    );
  }

  if (pointPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: pointPositions,
        colors: pointColors,
        size: 0.01,
        transparent: true,
      ),
    );
  }

  return primitives;
}

// Ensure your Contour class from earlier includes simplified vertex fields:
// class Contour { final int reg; final int nVertices; final List<int> vertices; ... }

List<DebugPrimitive> createSimplifiedContoursHelper(ContourSet? contourSet) {
  if (contourSet == null || contourSet.contours.isEmpty) {
    return [];
  }

  final double origX = contourSet.bounds.min[0];
  final double origY = contourSet.bounds.min[1];
  final double origZ = contourSet.bounds.min[2];
  
  final double cs = contourSet.cellSize;
  final double ch = contourSet.cellHeight;

  final List<double> linePositions = [];
  final List<double> lineColors = [];
  final List<double> pointPositions = [];
  final List<double> pointColors = [];
  
  final Vector3 colorBuffer = Vector3();

  // 1. Draw lines for each contour
  for (int i = 0; i < contourSet.contours.length; ++i) {
    final c = contourSet.contours[i];
    if (c.nRawVertices == 0) continue;

    regionToColor(colorBuffer, c.reg, 0.8);
    final List<double> baseColor = [colorBuffer.x, colorBuffer.y, colorBuffer.z];
    const List<double> whiteColor = [1.0, 1.0, 1.0];

    // Compute border color via a linear interpolation (lerp) factor of 128 / 255.0
    const double f = 128.0 / 255.0;
    final List<double> borderColor = [
      baseColor[0] * (1.0 - f) + whiteColor[0] * f,
      baseColor[1] * (1.0 - f) + whiteColor[1] * f,
      baseColor[2] * (1.0 - f) + whiteColor[2] * f,
    ];

    // Dual iteration variable configuration mirroring the JavaScript structure
    int k = c.nRawVertices - 1;
    for (int j = 0; j < c.nRawVertices; k = j++) {
      final List<double> va = c.rawVertices.sublist(k * 4, k * 4 + 4);
      final List<double> vb = c.rawVertices.sublist(j * 4, j * 4 + 4);

      // Checking area border bitflag mask (0x20000)
      final bool isAreaBorder = (va[3].toInt() & 0x20000) != 0;
      final List<double> col = isAreaBorder ? borderColor : baseColor;

      final double fx1 = origX + va[0] * cs;
      final double fy1 = origY + (va[1] + 1 + (i & 1)) * ch;
      final double fz1 = origZ + va[2] * cs;

      final double fx2 = origX + vb[0] * cs;
      final double fy2 = origY + (vb[1] + 1 + (i & 1)) * ch;
      final double fz2 = origZ + vb[2] * cs;

      linePositions.addAll([fx1, fy1, fz1]);
      lineColors.addAll([col[0], col[1], col[2]]);
      
      linePositions.addAll([fx2, fy2, fz2]);
      lineColors.addAll([col[0], col[1], col[2]]);
    }
  }

  // 2. Draw points for each contour
  for (int i = 0; i < contourSet.contours.length; ++i) {
    final c = contourSet.contours[i];
    regionToColor(colorBuffer, c.reg, 0.8);
    
    final List<double> darkenedColor = [
      colorBuffer.x * 0.5, 
      colorBuffer.y * 0.5, 
      colorBuffer.z * 0.5
    ];

    for (int j = 0; j < c.nRawVertices; ++j) {
      final List<double> v = c.rawVertices.sublist(j * 4, j * 4 + 4);
      double off = 0.0;
      List<double> colv = darkenedColor;

      // Checking for the BORDER_VERTEX bitflag mask (0x10000)
      if ((v[3].toInt() & 0x10000) != 0) {
        colv = [1.0, 1.0, 1.0];
        off = ch * 2.0;
      }

      final double fx = origX + v[0] * cs;
      final double fy = origY + (v[1] + 1 + (i & 1)) * ch + off;
      final double fz = origZ + v[2] * cs;

      pointPositions.addAll([fx, fy, fz]);
      pointColors.addAll([colv[0], colv[1], colv[2]]);
    }
  }

  final List<DebugPrimitive> primitives = [];

  if (linePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: linePositions,
        colors: lineColors,
        transparent: true,
        opacity: 0.9,
        lineWidth: 2.5,
      ),
    );
  }

  if (pointPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: pointPositions,
        colors: pointColors,
        size: 0.01,
        transparent: true,
      ),
    );
  }

  return primitives;
}

// Assuming this global constant matching your Recast mesh configuration is defined elsewhere
const int MESH_NULL_IDX = 0xFFFF;

// Helper structural class to match your JS PolyMesh pattern
class PolyMesh {
  int nPolys;
  int nVertices;
  int maxVerticesPerPoly;
  double cellSize;
  double cellHeight;
  final BoundingBox bounds; // [minX, minY, minZ, maxX, maxY, maxZ]
  final List<int> polys;
  final List<double> vertices;
  final List<int> areas;
  late List<int> regions;
  late List<int> flags;
  double maxEdgeError = 0;
  double borderSize = 0;

  double localWidth = 0;
  double localHeight = 0;

  PolyMesh({
    required this.nPolys,
    required this.nVertices,
    required this.maxVerticesPerPoly,
    required this.cellSize,
    required this.cellHeight,
    required this.bounds,
    required this.polys,
    required this.vertices,
    required this.areas,
    List<int>? regions,
    List<int>? flags,
    this.maxEdgeError = 0,
    this.borderSize = 0,
    this.localHeight = 0,
    this.localWidth = 0
  }){
    this.regions = regions ?? [];
    this.flags = flags ?? [];
  }

  factory PolyMesh.fromMap(Map<String,dynamic> map){
    return PolyMesh(
      nPolys: map['nPolys'], 
      nVertices: map['nVertices'], 
      maxVerticesPerPoly: map['maxVerticesPerPoly'], 
      cellSize: map['cellSize'], 
      cellHeight: map['cellHeight'], 
      bounds: map['bounds'], 
      polys: map['polys'], 
      vertices: map['vertices'], 
      areas: map['areas']
    );
  }
}

List<DebugPrimitive> createPolyMeshHelper(PolyMesh? polyMesh) {
  if (polyMesh == null || polyMesh.nPolys == 0) {
    return [];
  }

  final int nvp = polyMesh.maxVerticesPerPoly;
  final double cs = polyMesh.cellSize;
  final double ch = polyMesh.cellHeight;

  final double origX = polyMesh.bounds.min[0];
  final double origY = polyMesh.bounds.min[0];
  final double origZ = polyMesh.bounds.min[0];

  final List<double> triPositions = [];
  final List<double> triColors = [];
  final List<int> triIndices = [];

  final List<double> edgeLinePositions = [];
  final List<double> edgeLineColors = [];

  final List<double> vertexPositions = [];
  final List<double> vertexColors = [];

  final Vector3 colorBuffer = Vector3();
  int triVertexIndex = 0;

  // 1. Draw polygon triangles (Triangle Fan approach)
  for (int i = 0; i < polyMesh.nPolys; i++) {
    final int polyBase = i * nvp;
    final int area = polyMesh.areas[i];
    areaToColor(colorBuffer, area);

    for (int j = 2; j < nvp; j++) {
      final int v0 = polyMesh.polys[polyBase + 0];
      final int v1 = polyMesh.polys[polyBase + j - 1];
      final int v2 = polyMesh.polys[polyBase + j];

      if (v2 == MESH_NULL_IDX) break;

      final List<int> verticesList = [v0, v1, v2];
      for (int k = 0; k < 3; k++) {
        final int vertIndex = verticesList[k] * 3;
        final double x = origX + polyMesh.vertices[vertIndex] * cs;
        final double y = origY + (polyMesh.vertices[vertIndex + 1] + 1.0) * ch;
        final double z = origZ + polyMesh.vertices[vertIndex + 2] * cs;

        triPositions.addAll([x, y, z]);
        triColors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);
      }

      triIndices.addAll([triVertexIndex, triVertexIndex + 1, triVertexIndex + 2]);
      triVertexIndex += 3;
    }
  }

  // 2. Draw edges
  final List<double> edgeColor = [0.0, 48.0 / 255.0, 64.0 / 255.0];
  for (int i = 0; i < polyMesh.nPolys; i++) {
    final int polyBase = i * nvp;
    for (int j = 0; j < nvp; j++) {
      final int v0 = polyMesh.polys[polyBase + j];
      if (v0 == MESH_NULL_IDX) break;

      final int nj = (j + 1 >= nvp || polyMesh.polys[polyBase + j + 1] == MESH_NULL_IDX) ? 0 : j + 1;
      final int v1 = polyMesh.polys[polyBase + nj];

      final List<int> verticesList = [v0, v1];
      for (int k = 0; k < 2; k++) {
        final int vertIndex = verticesList[k] * 3;
        final double x = origX + polyMesh.vertices[vertIndex] * cs;
        final double y = origY + (polyMesh.vertices[vertIndex + 1] + 1.0) * ch + 0.01;
        final double z = origZ + polyMesh.vertices[vertIndex + 2] * cs;

        edgeLinePositions.addAll([x, y, z]);
        edgeLineColors.addAll(edgeColor);
      }
    }
  }

  // 3. Draw vertices (Points)
  const List<double> vertexColor = [1.0, 1.0, 1.0];
  for (int i = 0; i < polyMesh.nVertices; i++) {
    final int vertIndex = i * 3;
    final double x = origX + polyMesh.vertices[vertIndex] * cs;
    final double y = origY + (polyMesh.vertices[vertIndex + 1] + 1.0) * ch + 0.01;
    final double z = origZ + polyMesh.vertices[vertIndex + 2] * cs;

    vertexPositions.addAll([x, y, z]);
    vertexColors.addAll(vertexColor);
  }

  final List<DebugPrimitive> primitives = [];

  if (triPositions.isNotEmpty) {
    primitives.add(
      DebugTriangles(
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: false,
        opacity: 1.0,
        doubleSided: true,
      ),
    );
  }

  if (edgeLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: edgeLinePositions,
        colors: edgeLineColors,
        transparent: true,
        opacity: 0.5,
        lineWidth: 1.5,
      ),
    );
  }

  if (vertexPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: vertexPositions,
        colors: vertexColors,
        size: 0.025,
        transparent: true,
      ),
    );
  }

  return primitives;
}

// Helper structural class to match your JS PolyMeshDetail pattern
class PolyMeshDetail {
  final int nMeshes;
  final List<int> meshes; // 4 entries per mesh: [vertBase, vertCount, triBase, triCount]
  final List<double> vertices; // Flattened [x, y, z] layout
  final List<int> triangles; // Flattened [t0, t1, t2, flags] layout
  int nTriangles = 0;
  int nVertices = 0;

  PolyMeshDetail({
    required this.nMeshes,
    required this.meshes,
    required this.vertices,
    required this.triangles,
    this.nTriangles = 0,
    this.nVertices = 0
  });
}

List<DebugPrimitive> createPolyMeshDetailHelper(PolyMeshDetail? polyMeshDetail) {
  if (polyMeshDetail == null || polyMeshDetail.nMeshes == 0) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];
  const List<double> edgeColor = [0.0, 0.0, 0.0];
  const List<double> vertexColor = [1.0, 1.0, 1.0];
  final Vector3 colorBuffer = Vector3();

  // Local utility function to calculate submesh color variants
  Vector3 submeshToColor(Vector3 out, int submeshIndex) {
    final double hash = submeshIndex * 137.5;
    final double hue = hash % 360.0;
    hslToRgb(out, hue, 0.7, 0.6);
    out.x *= 0.3;
    out.y *= 0.3;
    out.z *= 0.3;
    return out;
  }

  // 1. Draw triangles
  final List<double> triPositions = [];
  final List<double> triColors = [];
  final List<int> triIndices = [];
  int triVertexIndex = 0;

  for (int i = 0; i < polyMeshDetail.nMeshes; ++i) {
    final int m = i * 4;
    final int bverts = polyMeshDetail.meshes[m + 0];
    final int btris = polyMeshDetail.meshes[m + 2];
    final int ntris = polyMeshDetail.meshes[m + 3];

    final int verts = bverts * 3;
    final int tris = btris * 4;
    submeshToColor(colorBuffer, i);

    for (int j = 0; j < ntris; ++j) {
      final int triBase = tris + j * 4;
      final int t0 = polyMeshDetail.triangles[triBase + 0];
      final int t1 = polyMeshDetail.triangles[triBase + 1];
      final int t2 = polyMeshDetail.triangles[triBase + 2];

      final int v0Base = verts + t0 * 3;
      final int v1Base = verts + t1 * 3;
      final int v2Base = verts + t2 * 3;

      // Add triangle vertices efficiently using addAll
      triPositions.addAll([
        polyMeshDetail.vertices[v0Base],
        polyMeshDetail.vertices[v0Base + 1],
        polyMeshDetail.vertices[v0Base + 2],
        polyMeshDetail.vertices[v1Base],
        polyMeshDetail.vertices[v1Base + 1],
        polyMeshDetail.vertices[v1Base + 2],
        polyMeshDetail.vertices[v2Base],
        polyMeshDetail.vertices[v2Base + 1],
        polyMeshDetail.vertices[v2Base + 2],
      ]);

      // Add colors for all three vertices
      for (int k = 0; k < 3; k++) {
        triColors.addAll([colorBuffer.x, colorBuffer.y, colorBuffer.z]);
      }

      triIndices.addAll([triVertexIndex, triVertexIndex + 1, triVertexIndex + 2]);
      triVertexIndex += 3;
    }
  }

  if (triPositions.isNotEmpty) {
    primitives.add(
      DebugTriangles(
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: false,
        opacity: 1.0,
      ),
    );
  }

  // 2. Draw internal edges
  final List<double> internalLinePositions = [];
  final List<double> internalLineColors = [];

  for (int i = 0; i < polyMeshDetail.nMeshes; ++i) {
    final int m = i * 4;
    final int bverts = polyMeshDetail.meshes[m + 0];
    final int btris = polyMeshDetail.meshes[m + 2];
    final int ntris = polyMeshDetail.meshes[m + 3];

    final int verts = bverts * 3;
    final int tris = btris * 4;

    for (int j = 0; j < ntris; ++j) {
      final int t = tris + j * 4;
      final List<int> triVertices = [
        polyMeshDetail.triangles[t + 0],
        polyMeshDetail.triangles[t + 1],
        polyMeshDetail.triangles[t + 2],
      ];

      int kp = 2;
      for (int k = 0; k < 3; kp = k++) {
        // Evaluate internal edge flag configurations
        final int ef = (polyMeshDetail.triangles[t + 3] >> (kp * 2)) & 0x3;
        if (ef == 0) {
          final int tkp = triVertices[kp];
          final int tk = triVertices[k];
          
          if (tkp < tk) {
            final int vkpBase = verts + tkp * 3;
            final int vkBase = verts + tk * 3;

            internalLinePositions.addAll([
              polyMeshDetail.vertices[vkpBase],
              polyMeshDetail.vertices[vkpBase + 1],
              polyMeshDetail.vertices[vkpBase + 2],
              polyMeshDetail.vertices[vkBase],
              polyMeshDetail.vertices[vkBase + 1],
              polyMeshDetail.vertices[vkBase + 2],
            ]);

            for (int l = 0; l < 2; l++) {
              internalLineColors.addAll(edgeColor);
            }
          }
        }
      }
    }
  }

  if (internalLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: internalLinePositions,
        colors: internalLineColors,
        transparent: true,
        opacity: 0.4,
        lineWidth: 1.0,
      ),
    );
  }

  // 3. Draw external edges
  final List<double> externalLinePositions = [];
  final List<double> externalLineColors = [];

  for (int i = 0; i < polyMeshDetail.nMeshes; ++i) {
    final int m = i * 4;
    final int bverts = polyMeshDetail.meshes[m + 0];
    final int btris = polyMeshDetail.meshes[m + 2];
    final int ntris = polyMeshDetail.meshes[m + 3];

    final int verts = bverts * 3;
    final int tris = btris * 4;

    for (int j = 0; j < ntris; ++j) {
      final int t = tris + j * 4;
      final List<int> triVertices = [
        polyMeshDetail.triangles[t + 0],
        polyMeshDetail.triangles[t + 1],
        polyMeshDetail.triangles[t + 2],
      ];

      int kp = 2;
      for (int k = 0; k < 3; kp = k++) {
        // Evaluate external edge flag configurations
        final int ef = (polyMeshDetail.triangles[t + 3] >> (kp * 2)) & 0x3;
        if (ef != 0) {
          final int tkp = triVertices[kp];
          final int tk = triVertices[k];
          
          final int vkpBase = verts + tkp * 3;
          final int vkBase = verts + tk * 3;

          externalLinePositions.addAll([
            polyMeshDetail.vertices[vkpBase],
            polyMeshDetail.vertices[vkpBase + 1],
            polyMeshDetail.vertices[vkpBase + 2],
            polyMeshDetail.vertices[vkBase],
            polyMeshDetail.vertices[vkBase + 1],
            polyMeshDetail.vertices[vkBase + 2],
          ]);

          for (int l = 0; l < 2; l++) {
            externalLineColors.addAll(edgeColor);
          }
        }
      }
    }
  }

  if (externalLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: externalLinePositions,
        colors: externalLineColors,
        transparent: true,
        opacity: 0.8,
        lineWidth: 10.0,
      ),
    );
  }

  // 4. Draw vertices as points
  final List<double> vertexPositions = [];
  final List<double> vertexColors = [];

  for (int i = 0; i < polyMeshDetail.nMeshes; ++i) {
    final int m = i * 4;
    final int bverts = polyMeshDetail.meshes[m];
    final int nverts = polyMeshDetail.meshes[m + 1];
    final int verts = bverts * 3;

    for (int j = 0; j < nverts; ++j) {
      final int vBase = verts + j * 3;
      vertexPositions.addAll([
        polyMeshDetail.vertices[vBase],
        polyMeshDetail.vertices[vBase + 1],
        polyMeshDetail.vertices[vBase + 2],
      ]);
      vertexColors.addAll(vertexColor);
    }
  }

  if (vertexPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: vertexPositions,
        colors: vertexColors,
        size: 0.025,
        transparent: true,
      ),
    );
  }

  return primitives;
}

// Assuming this global constant matching your Recast mesh configuration is defined elsewhere
const int POLY_NEIS_FLAG_EXT_LINK = 0x8000;

class PolyDetailMesh {
  final int trianglesBase;
  final int trianglesCount;
  final int verticesBase;

  PolyDetailMesh({
    required this.trianglesBase,
    required this.trianglesCount,
    required this.verticesBase,
  });
}

List<DebugPrimitive>createNavMeshHelper(NavMesh? navMesh) {
  if (navMesh == null || navMesh.tiles.isEmpty) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];
  
  final List<double> triPositions = [];
  final List<double> triColors = [];
  final List<int> triIndices = [];
  int triVertexIndex = 0;

  final List<double> interPolyLinePositions = [];
  final List<double> interPolyLineColors = [];
  final List<double> outerPolyLinePositions = [];
  final List<double> outerPolyLineColors = [];
  final List<double> vertexPositions = [];
  final List<double> vertexColors = [];

  final Vector3 colorBuffer = Vector3();

  // Iterate over all active navmesh map tiles
  for (final tile in navMesh.tiles.values) {
    if (tile == null) continue;

    // 1. Draw detail triangles for each polygon
    for (int polyId = 0; polyId < (tile.polys?.length ?? 0); polyId++) {
      final poly = tile.polys![polyId];
      
      // Protect against indices overflow safely using optional layout boundaries
      if (polyId >= tile.detailMeshes.length) continue;
      final polyDetail = tile.detailMeshes[polyId];
      if (polyDetail == null) continue;

      // Get polygon color based on area code matching alpha metrics
      areaToColor(colorBuffer, poly.area, 0.4);
      final List<double> col = [colorBuffer.x, colorBuffer.y, colorBuffer.z];

      // Draw detail triangles for this specific polygon
      for (int j = 0; j < polyDetail.trianglesCount; j++) {
        final int triBase = (polyDetail.trianglesBase + j) * 4;
        final int t0 = tile.detailTriangles[triBase];
        final int t1 = tile.detailTriangles[triBase + 1];
        final int t2 = tile.detailTriangles[triBase + 2];

        // Process endpoints tracking positions data models
        for (int k = 0; k < 3; k++) {
          final int vertIndex = (k == 0) ? t0 : (k == 1) ? t1 : t2;
          double vx;
          double vy;
          double vz;

          if (vertIndex < poly.vertices.length) {
            // Vertex derived from the master boundary polygon
            final int polyVertIndex = poly.vertices[vertIndex];
            final int vBase = polyVertIndex * 3;
            vx = tile.vertices[vBase];
            vy = tile.vertices[vBase + 1];
            vz = tile.vertices[vBase + 2];
          } else {
            // Vertex derived from internal resolution detail mesh definitions
            final int detailVertIndex = (polyDetail.verticesBase + vertIndex - poly.vertices.length) * 3;
            vx = tile.detailVertices[detailVertIndex];
            vy = tile.detailVertices[detailVertIndex + 1];
            vz = tile.detailVertices[detailVertIndex + 2];
          }

          triPositions.addAll([vx, vy, vz]);
          triColors.addAll(col);
        }

        triIndices.addAll([triVertexIndex, triVertexIndex + 1, triVertexIndex + 2]);
        triVertexIndex += 3;
      }
    }

    // 2. Draw polygon boundary layout segments
    const List<double> innerColor = [0.2, 0.2, 0.2];
    const List<double> outerColor = [0.6, 0.6, 1.0];

    for (int polyId = 0; polyId < (tile.polys?.length ?? 0); polyId++) {
      final poly = tile.polys![polyId];
      
      for (int j = 0; j < poly.vertices.length; j++) {
        final int nj = (j + 1) % poly.vertices.length;
        final int nei = poly.neis[j];

        // Strict bitwise constraint checks matching external boundary masks
        final bool isBoundary = (nei & POLY_NEIS_FLAG_EXT_LINK) != 0;

        final int polyVertIndex1 = poly.vertices[j];
        final int polyVertIndex2 = poly.vertices[nj];
        final int v1Base = polyVertIndex1 * 3;
        final int v2Base = polyVertIndex2 * 3;

        final double v1x = tile.vertices[v1Base];
        final double v1y = tile.vertices[v1Base + 1] + 0.01; // Elevation buffer offset
        final double v1z = tile.vertices[v1Base + 2];

        final double v2x = tile.vertices[v2Base];
        final double v2y = tile.vertices[v2Base + 1] + 0.01;
        final double v2z = tile.vertices[v2Base + 2];

        if (isBoundary) {
          // External link boundary edge layout structures
          outerPolyLinePositions.addAll([v1x, v1y, v1z, v2x, v2y, v2z]);
          outerPolyLineColors.addAll([...outerColor, ...outerColor]);
        } else {
          // Internal structural edge segmentation lines
          interPolyLinePositions.addAll([v1x, v1y, v1z, v2x, v2y, v2z]);
          interPolyLineColors.addAll([...innerColor, ...innerColor]);
        }
      }
    }

    // 3. Draw master reference vertices
    const List<double> vertexColor = [1.0, 1.0, 1.0];
    for (int i = 0; i < tile.vertices.length; i += 3) {
      vertexPositions.addAll([tile.vertices[i], tile.vertices[i + 1], tile.vertices[i + 2]]);
      vertexColors.addAll(vertexColor);
    }
  }

  // 4. Assemble pipeline render primitives
  if (triPositions.isNotEmpty) {
    primitives.add(
      DebugTriangles(
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: true,
        opacity: 0.8,
        doubleSided: true,
      ),
    );
  }

  if (interPolyLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: interPolyLinePositions,
        colors: interPolyLineColors,
        transparent: true,
        opacity: 0.3,
        lineWidth: 1.5,
      ),
    );
  }

  if (outerPolyLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: outerPolyLinePositions,
        colors: outerPolyLineColors,
        transparent: true,
        opacity: 0.9,
        lineWidth: 2.5,
      ),
    );
  }

  if (vertexPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: vertexPositions,
        colors: vertexColors,
        size: 0.025,
        transparent: true,
      ),
    );
  }

  return primitives;
}

List<DebugPrimitive> createNavMeshTileHelper(NavMeshTile? tile) {
  if (tile == null) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];
  
  final List<double> triPositions = [];
  final List<double> triColors = [];
  final List<int> triIndices = [];
  int triVertexIndex = 0;

  final List<double> interPolyLinePositions = [];
  final List<double> interPolyLineColors = [];
  final List<double> outerPolyLinePositions = [];
  final List<double> outerPolyLineColors = [];
  final List<double> vertexPositions = [];
  final List<double> vertexColors = [];

  final Vector3 colorBuffer = Vector3();

  // 1. Draw detail triangles for each polygon
  for (int polyId = 0; polyId < (tile.polys?.length ?? 0); polyId++) {
    final poly = tile.polys![polyId];
    
    if (polyId >= tile.detailMeshes.length) continue;
    final polyDetail = tile.detailMeshes[polyId];
    if (polyDetail == null) continue;

    // Get polygon color based on area code matching alpha metrics
    areaToColor(colorBuffer, poly.area, 0.4);
    final List<double> col = [colorBuffer.x, colorBuffer.y, colorBuffer.z];

    // Draw detail triangles for this polygon
    for (int j = 0; j < polyDetail.trianglesCount; j++) {
      final int triBase = (polyDetail.trianglesBase + j) * 4;
      final int t0 = tile.detailTriangles[triBase];
      final int t1 = tile.detailTriangles[triBase + 1];
      final int t2 = tile.detailTriangles[triBase + 2];

      // Get triangle vertices
      for (int k = 0; k < 3; k++) {
        final int vertIndex = (k == 0) ? t0 : (k == 1) ? t1 : t2;
        double vx;
        double vy;
        double vz;

        if (vertIndex < poly.vertices.length) {
          // Vertex from main polygon boundary
          final int polyVertIndex = poly.vertices[vertIndex];
          final int vBase = polyVertIndex * 3;
          vx = tile.vertices[vBase];
          vy = tile.vertices[vBase + 1];
          vz = tile.vertices[vBase + 2];
        } else {
          // Vertex from high-resolution detail mesh definitions
          final int detailVertIndex = (polyDetail.verticesBase + vertIndex - poly.vertices.length) * 3;
          vx = tile.detailVertices[detailVertIndex];
          vy = tile.detailVertices[detailVertIndex + 1];
          vz = tile.detailVertices[detailVertIndex + 2];
        }

        triPositions.addAll([vx, vy, vz]);
        triColors.addAll(col);
      }

      // Add triangle indices
      triIndices.addAll([triVertexIndex, triVertexIndex + 1, triVertexIndex + 2]);
      triVertexIndex += 3;
    }
  }

  // 2. Draw polygon boundary layout segments
  const List<double> innerColor = [0.2, 0.2, 0.2];
  const List<double> outerColor = [0.6, 0.6, 1.0];

  for (int polyId = 0; polyId < (tile.polys?.length ?? 0); polyId++) {
    final poly = tile.polys![polyId];
    
    for (int j = 0; j < poly.vertices.length; j++) {
      final int nj = (j + 1) % poly.vertices.length;
      final int nei = poly.neis[j];

      // Explicit bitwise check against external boundary masks
      final bool isBoundary = (nei & POLY_NEIS_FLAG_EXT_LINK) != 0;

      final int polyVertIndex1 = poly.vertices[j];
      final int polyVertIndex2 = poly.vertices[nj];
      final int v1Base = polyVertIndex1 * 3;
      final int v2Base = polyVertIndex2 * 3;

      final double v1x = tile.vertices[v1Base];
      final double v1y = tile.vertices[v1Base + 1] + 0.01; // Tiny height offset to prevent z-fighting
      final double v1z = tile.vertices[v1Base + 2];

      final double v2x = tile.vertices[v2Base];
      final double v2y = tile.vertices[v2Base + 1] + 0.01;
      final double v2z = tile.vertices[v2Base + 2];

      if (isBoundary) {
        // Outer boundary link edges
        outerPolyLinePositions.addAll([v1x, v1y, v1z, v2x, v2y, v2z]);
        outerPolyLineColors.addAll([...outerColor, ...outerColor]);
      } else {
        // Inner polygon structural sharing lines
        interPolyLinePositions.addAll([v1x, v1y, v1z, v2x, v2y, v2z]);
        interPolyLineColors.addAll([...innerColor, ...innerColor]);
      }
    }
  }

  // 3. Draw individual reference vertices
  const List<double> vertexColor = [1.0, 1.0, 1.0];
  for (int i = 0; i < tile.vertices.length; i += 3) {
    vertexPositions.addAll([tile.vertices[i], tile.vertices[i + 1], tile.vertices[i + 2]]);
    vertexColors.addAll(vertexColor);
  }

  // 4. Assemble final debug rendering primitives lists
  if (triPositions.isNotEmpty) {
    primitives.add(
      DebugTriangles(
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: true,
        opacity: 0.8,
        doubleSided: true,
      ),
    );
  }

  if (interPolyLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: interPolyLinePositions,
        colors: interPolyLineColors,
        transparent: true,
        opacity: 0.3,
        lineWidth: 1.5,
      ),
    );
  }

  if (outerPolyLinePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: outerPolyLinePositions,
        colors: outerPolyLineColors,
        transparent: true,
        opacity: 0.9,
        lineWidth: 2.5,
      ),
    );
  }

  if (vertexPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: vertexPositions,
        colors: vertexColors,
        size: 0.025,
        transparent: true,
      ),
    );
  }

  return primitives;
}

List<DebugPrimitive> createNavMeshPolyHelper(
  NavMesh navMesh,
  int nodeRef, [
  List<double> color = const [0.0, 0.75, 1.0],
]) {
  final List<DebugPrimitive> primitives = [];

  // Safely look up references matching the input path tracking token
  final lookup = getNodeByRef(navMesh, nodeRef)!;
  final tile = navMesh.tiles[lookup.tileId];
  
  if (tile == null || lookup.polyIndex >= (tile.polys?.length ?? 0)) {
    return primitives;
  }

  final poly = tile.polys![lookup.polyIndex];
  
  // Guard access boundaries if detailMeshes indices are sparse
  final NavMeshPolyDetail? detailMesh = lookup.polyIndex < tile.detailMeshes.length 
      ? tile.detailMeshes[lookup.polyIndex] 
      : null;

  final List<double> baseColor = [color[0] * 0.25, color[1] * 0.25, color[2] * 0.25];

  if (detailMesh == null) {
    // Fallback: draw basic polygon without detailed mesh data using a Triangle Fan approach
    final List<double> triPositions = [];
    final List<double> triColors = [];
    final List<int> triIndices = [];

    if (poly.vertices.length >= 3) {
      for (int i = 2; i < poly.vertices.length; i++) {
        final int v0Index = poly.vertices[0] * 3;
        final int v1Index = poly.vertices[i - 1] * 3;
        final int v2Index = poly.vertices[i] * 3;

        // Append vertex components using addAll
        triPositions.addAll([
          tile.vertices[v0Index], tile.vertices[v0Index + 1], tile.vertices[v0Index + 2],
          tile.vertices[v1Index], tile.vertices[v1Index + 1], tile.vertices[v1Index + 2],
          tile.vertices[v2Index], tile.vertices[v2Index + 1], tile.vertices[v2Index + 2],
        ]);

        // Push layout color settings across 3 endpoints
        for (int j = 0; j < 3; j++) {
          triColors.addAll(baseColor);
        }

        final int baseIndex = (i - 2) * 3;
        triIndices.addAll([baseIndex, baseIndex + 1, baseIndex + 2]);
      }
    }

    if (triPositions.isNotEmpty) {
      primitives.add(
        DebugTriangles(
          positions: triPositions,
          colors: triColors,
          indices: triIndices,
          transparent: true,
          opacity: 0.6,
          doubleSided: true,
        ),
      );
    }
    return primitives;
  }

  // Draw detail triangles for this polygon
  final List<double> triPositions = [];
  final List<double> triColors = [];
  final List<int> triIndices = [];

  for (int i = 0; i < detailMesh.trianglesCount; ++i) {
    final int t = (detailMesh.trianglesBase + i) * 4;
    final List<int> detailTriangles = tile.detailTriangles;

    for (int j = 0; j < 3; ++j) {
      final int vertIndex = detailTriangles[t + j];
      
      if (vertIndex < poly.vertices.length) {
        final int polyVertIndex = poly.vertices[vertIndex] * 3;
        triPositions.addAll([
          tile.vertices[polyVertIndex],
          tile.vertices[polyVertIndex + 1],
          tile.vertices[polyVertIndex + 2],
        ]);
      } else {
        final int detailVertIndex = (detailMesh.verticesBase + vertIndex - poly.vertices.length) * 3;
        triPositions.addAll([
          tile.detailVertices[detailVertIndex],
          tile.detailVertices[detailVertIndex + 1],
          tile.detailVertices[detailVertIndex + 2],
        ]);
      }

      triColors.addAll(baseColor);
    }

    final int baseIndex = i * 3;
    triIndices.addAll([baseIndex, baseIndex + 1, baseIndex + 2]);
  }

  if (triPositions.isNotEmpty) {
    primitives.add(
      DebugTriangles(
        positions: triPositions,
        colors: triColors,
        indices: triIndices,
        transparent: true,
        opacity: 0.6,
        doubleSided: true,
      ),
    );
  }

  return primitives;
}

// Helper structural classes to expand your JS NavMesh data model
class BVNode {
  final int i; // Leaf index marker
  final List<int> bounds; // [minX, minY, minZ, maxX, maxY, maxZ] quantized values

  BVNode({required this.i, required this.bounds});
}

class BVTree {
  final double quantFactor;
  final List<BVNode> nodes;

  BVTree({required this.quantFactor, required this.nodes});
}

// Add final BVTree bvTree; to your MeshTile class definition from earlier:
// class MeshTile { ... final BVTree bvTree; ... }

List<DebugPrimitive> createNavMeshTileBvTreeHelper(NavMeshTile? navMeshTile) {
  final List<DebugPrimitive> primitives = [];
  if (navMeshTile == null || navMeshTile.bvTree.nodes.isEmpty) {
    return primitives;
  }

  // Arrays for wireframe box edges
  final List<double> linePositions = [];
  final List<double> lineColors = [];

  // Color for BV tree nodes (white with transparency)
  const List<double> nodeColor = [1.0, 1.0, 1.0];

  // Calculate inverse quantization factor
  final double cs = 1.0 / navMeshTile.bvTree.quantFactor;

  final double boundsMinX = navMeshTile.bounds.min[0];
  final double boundsMinY = navMeshTile.bounds.min[1];
  final double boundsMinZ = navMeshTile.bounds.min[2];

  for (int i = 0; i < navMeshTile.bvTree.nodes.length; i++) {
    final node = navMeshTile.bvTree.nodes[i];

    // Follows your original skipping rule exactly
    if (node.i < 0) continue;

    // Calculate world coordinates from quantized bounds boundaries
    final double minX = boundsMinX + node.bounds.min[0] * cs;
    final double minY = boundsMinY + node.bounds.min[1] * cs;
    final double minZ = boundsMinZ + node.bounds.min[2] * cs;
    final double maxX = boundsMinX + node.bounds.max[0] * cs;
    final double maxY = boundsMinY + node.bounds.max[1] * cs;
    final double maxZ = boundsMinZ + node.bounds.max[2] * cs;

    // Create wireframe box edges (12 segments = 24 sequential vertices)
    linePositions.addAll([
      // Bottom face segment loops
      minX, minY, minZ, maxX, minY, minZ,
      maxX, minY, minZ, maxX, minY, maxZ,
      maxX, minY, maxZ, minX, minY, maxZ,
      minX, minY, maxZ, minX, minY, minZ,
      
      // Top face segment loops
      minX, maxY, minZ, maxX, maxY, minZ,
      maxX, maxY, minZ, maxX, maxY, maxZ,
      maxX, maxY, maxZ, minX, maxY, maxZ,
      minX, maxY, maxZ, minX, maxY, minZ,
      
      // Vertical connecting structural edges
      minX, minY, minZ, minX, maxY, minZ,
      maxX, minY, minZ, maxX, maxY, minZ,
      maxX, minY, maxZ, maxX, maxY, maxZ,
      minX, minY, maxZ, minX, maxY, maxZ,
    ]);

    // Add colors for all line segment endpoints (24 vertices)
    for (int j = 0; j < 24; j++) {
      lineColors.addAll(nodeColor);
    }
  }

  // Create line segments primitive
  if (linePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: linePositions,
        colors: lineColors,
        transparent: true,
        opacity: 0.5,
        lineWidth: 1.0,
      ),
    );
  }

  return primitives;
}

List<DebugPrimitive> createNavMeshBvTreeHelper(NavMesh? navMesh) {
  if (navMesh == null || navMesh.tiles.isEmpty) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];

  // Draw BV tree layout configurations for all active tiles in the nav mesh map
  for (final tile in navMesh.tiles.values) {
    if (tile == null) continue;
    
    final List<DebugPrimitive> tilePrimitives = createNavMeshTileBvTreeHelper(tile);
    primitives.addAll(tilePrimitives);
  }

  return primitives;
}

final Vector3 _createNavMeshLinksHelperSourceCenter = Vector3();
final Vector3 _createNavMeshLinksHelperTargetCenter = Vector3();
final Vector3 _createNavMeshLinksHelperEdgeStart = Vector3();
final Vector3 _createNavMeshLinksHelperEdgeEnd = Vector3();
final Vector3 _createNavMeshLinksHelperEdgeMidpoint = Vector3();
final Vector3 _createNavMeshLinksHelperSourcePoint = Vector3();
final Vector3 _createNavMeshLinksHelperTargetPoint = Vector3();

List<DebugPrimitive> createNavMeshLinksHelper(NavMesh? navMesh) {
  if (navMesh == null || navMesh.links.isEmpty) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];
  final List<double> linePositions = [];
  final List<double> lineColors = [];
  const List<double> linkColor = [1.0, 1.0, 0.0]; // Bright yellow

  // Local helper function to lineally interpolate between two scalar values
  double lerp(double start, double end, double t) => start + (end - start) * t;

  // Local helper function to calculate the mathematical center of a polygon
  Vector3 getPolyCenter(Vector3 out, NavMeshTile tile, NavMeshPoly poly) {
    double centerX = 0.0;
    double centerY = 0.0;
    double centerZ = 0.0;
    final int nv = poly.vertices.length;

    for (int i = 0; i < nv; i++) {
      final int vertIndex = poly.vertices[i] * 3;
      centerX += tile.vertices[vertIndex];
      centerY += tile.vertices[vertIndex + 1];
      centerZ += tile.vertices[vertIndex + 2];
    }
    
    out.setValues(centerX / nv, centerY / nv, centerZ / nv);
    return out;
  }

  // Process each connection link
  for (final link in navMesh.links.values) {
    if (link?.allocated == null) continue;

    // Safety checks against node boundaries lookup
    if ((link?.fromNodeIndex ?? 0) >= navMesh.nodes.length || (link?.toNodeIndex ?? 0) >= navMesh.nodes.length) continue;

    // Get source polygon data layers
    final sourceLookup = navMesh.nodes[link!.fromNodeIndex];
    final sourceTile = navMesh.tiles[sourceLookup!.tileId];
    final sourcePoly = (sourceTile != null && sourceLookup.polyIndex < (sourceTile.polys?.length ?? 0))
        ? sourceTile.polys![sourceLookup.polyIndex]
        : null;

    // Get target polygon data layers
    final targetLookup = navMesh.nodes[link.toNodeIndex];
    final targetTile = navMesh.tiles[targetLookup!.tileId];
    final targetPoly = (targetTile != null && targetLookup.polyIndex < (targetTile.polys?.length ?? 0))
        ? targetTile.polys![targetLookup.polyIndex]
        : null;

    if (sourceTile == null || sourcePoly == null || targetTile == null || targetPoly == null) {
      continue;
    }

    // Calculate source and target center positions utilizing our library scratch buffers
    final Vector3 sourceCenter = getPolyCenter(_createNavMeshLinksHelperSourceCenter, sourceTile, sourcePoly);
    final Vector3 targetCenter = getPolyCenter(_createNavMeshLinksHelperTargetCenter, targetTile, targetPoly);

    // Fetch bounding edge vectors mapping coordinates out of the raw float lists
    final int edgeIndex = link.edge;
    final int nextEdgeIndex = (edgeIndex + 1) % sourcePoly.vertices.length;
    
    final int v0Index = sourcePoly.vertices[edgeIndex] * 3;
    final int v1Index = sourcePoly.vertices[nextEdgeIndex] * 3;

    // Equivalent to glMatrix vec3.fromBuffer inside three_js_math
    final Vector3 edgeStart = _createNavMeshLinksHelperEdgeStart.setValues(
      sourceTile.vertices[v0Index],
      sourceTile.vertices[v0Index + 1],
      sourceTile.vertices[v0Index + 2],
    );
    final Vector3 edgeEnd = _createNavMeshLinksHelperEdgeEnd.setValues(
      sourceTile.vertices[v1Index],
      sourceTile.vertices[v1Index + 1],
      sourceTile.vertices[v1Index + 2],
    );

    // Calculate edge midpoint via instance addition logic
    final Vector3 edgeMidpoint = _createNavMeshLinksHelperEdgeMidpoint
        .setFrom(edgeStart)
        .add(edgeEnd)
        .scale(0.5);

    // Displace midpoint 10% inward towards polygon center to smooth arcing paths
    const double inwardFactor = 0.1;
    final Vector3 sourcePoint = _createNavMeshLinksHelperSourcePoint
        .setFrom(edgeMidpoint)
        .lerp(sourceCenter, inwardFactor);
    sourcePoint.y += 0.05; // Elevation buffer offset

    final Vector3 targetPoint = _createNavMeshLinksHelperTargetPoint.setFrom(targetCenter);
    targetPoint.y += 0.05;

    // Create arced line paths utilizing multiple subdivisions
    const int numSegments = 12;
    const double arcHeight = 0.3;

    for (int i = 0; i < numSegments; i++) {
      final double t0 = i / numSegments;
      final double t1 = (i + 1) / numSegments;

      // Calculate path markers with sinusoidal vertical peak heights
      final double x0 = lerp(sourcePoint.x, targetPoint.x, t0);
      final double y0 = lerp(sourcePoint.y, targetPoint.y, t0) + math.sin(t0 * math.pi) * arcHeight;
      final double z0 = lerp(sourcePoint.z, targetPoint.z, t0);

      final double x1 = lerp(sourcePoint.x, targetPoint.x, t1);
      final double y1 = lerp(sourcePoint.y, targetPoint.y, t1) + math.sin(t1 * math.pi) * arcHeight;
      final double z1 = lerp(sourcePoint.z, targetPoint.z, t1);

      // Append segment layout metrics to buffers
      linePositions.addAll([x0, y0, z0, x1, y1, z1]);
      lineColors.addAll([...linkColor, ...linkColor]);
    }

    // Add a directional arrow head indicator towards the destination terminal
    const double arrowT = 0.85;
    final double arrowX = lerp(sourcePoint.x, targetPoint.x, arrowT);
    final double arrowY = lerp(sourcePoint.y, targetPoint.y, arrowT) + math.sin(arrowT * math.pi) * arcHeight;
    final double arrowZ = lerp(sourcePoint.z, targetPoint.z, arrowT);

    const double nextT = 0.95;
    final double nextX = lerp(sourcePoint.x, targetPoint.x, nextT);
    final double nextY = lerp(sourcePoint.y, targetPoint.y, nextT) + math.sin(nextT * math.pi) * arcHeight;
    final double nextZ = lerp(sourcePoint.z, targetPoint.z, nextT);

    final double dirX = nextX - arrowX;
    final double dirY = nextY - arrowY;
    final double dirZ = nextZ - arrowZ;
    
    final double len = math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    final double nx = (len == 0) ? 0.0 : dirX / len;
    final double ny = (len == 0) ? 0.0 : dirY / len;
    final double nz = (len == 0) ? 0.0 : dirZ / len;

    // Form geometric wings via cross perpendicular projections
    const double arrowLength = 0.15;
    const double arrowWidth = 0.08;
    final double perpX = -nz;
    final double perpZ = nx;

    // Left wing segment
    linePositions.addAll([
      arrowX, arrowY, arrowZ,
      arrowX - nx * arrowLength + perpX * arrowWidth,
      arrowY - ny * arrowLength,
      arrowZ - nz * arrowLength + perpZ * arrowWidth,
    ]);

    // Right wing segment
    linePositions.addAll([
      arrowX, arrowY, arrowZ,
      arrowX - nx * arrowLength - perpX * arrowWidth,
      arrowY - ny * arrowLength,
      arrowZ - nz * arrowLength - perpZ * arrowWidth,
    ]);

    // Fill structural color mappings across the arrow segment updates (4 endpoints)
    for (int k = 0; k < 4; k++) {
      lineColors.addAll(linkColor);
    }
  }

  if (linePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: linePositions,
        colors: lineColors,
        transparent: true,
        opacity: 0.9,
        lineWidth: 3.0,
      ),
    );
  }

  return primitives;
}

// Ensure your MeshTile class includes the walkableClimb property:
// class MeshTile { ... final double walkableClimb; ... }
List<DebugPrimitive> createNavMeshTilePortalsHelper(NavMeshTile? navMeshTile) {
  final List<DebugPrimitive> primitives = [];
  if (navMeshTile == null || navMeshTile.polys?.isNotEmpty == false) {
    return primitives;
  }

  const double padx = 0.04; // Visual padding multiplier
  final double pady = navMeshTile.walkableClimb; // Vertical clearance scale

  final Map<int, List<double>> sideColors = {
    0: [128.0 / 255.0, 0.0, 0.0],          // Red boundary
    2: [0.0, 128.0 / 255.0, 0.0],          // Green boundary
    4: [128.0 / 255.0, 0.0, 128.0 / 255.0], // Magenta boundary
    6: [0.0, 128.0 / 255.0, 128.0 / 255.0], // Cyan boundary
  };

  final List<double> positions = [];
  final List<double> colors = [];
  const List<int> drawSides = [0, 2, 4, 6];

  for (final side in drawSides) {
    final int matchMask = POLY_NEIS_FLAG_EXT_LINK | side;
    final List<double>? color = sideColors[side];
    if (color == null) continue;

    for (int polyId = 0; polyId < (navMeshTile.polys?.length ?? 0); polyId++) {
      final poly = navMeshTile.polys![polyId];
      final int nv = poly.vertices.length;

      for (int j = 0; j < nv; j++) {
        // Enforces exact bitwise connection mask verification match
        if (poly.neis[j] != matchMask) continue;

        final int v0Index = poly.vertices[j];
        final int v1Index = poly.vertices[(j + 1) % nv];
        
        final int aBase = v0Index * 3;
        final int bBase = v1Index * 3;

        final double ax = navMeshTile.vertices[aBase];
        final double ay = navMeshTile.vertices[aBase + 1];
        final double az = navMeshTile.vertices[aBase + 2];
        
        final double bx = navMeshTile.vertices[bBase];
        final double by = navMeshTile.vertices[bBase + 1];
        final double bz = navMeshTile.vertices[bBase + 2];

        if (side == 0 || side == 4) {
          final double x = ax + (side == 0 ? -padx : padx);
          
          // Generate four layout edges of rectangle (8 vertices for 4 line segments)
          positions.addAll([
            x, ay - pady, az, x, ay + pady, az,
            x, ay + pady, az, x, by + pady, bz,
            x, by + pady, bz, x, by - pady, bz,
            x, by - pady, bz, x, ay - pady, az,
          ]);
        } else if (side == 2 || side == 6) {
          final double z = az + (side == 2 ? -padx : padx);
          
          positions.addAll([
            ax, ay - pady, z, ax, ay + pady, z,
            ax, ay + pady, z, bx, by + pady, z,
            bx, by + pady, z, bx, by - pady, z,
            bx, by - pady, z, ax, ay - pady, z,
          ]);
        }

        // Add matching color components across all 8 rectangle segment endpoints
        for (int k = 0; k < 8; k++) {
          colors.addAll(color);
        }
      }
    }
  }

  if (positions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: positions,
        colors: colors,
        transparent: true,
        opacity: 0.5,
        lineWidth: 2.0,
      ),
    );
  }

  return primitives;
}

List<DebugPrimitive> createNavMeshPortalsHelper(NavMesh? navMesh) {
  if (navMesh == null || navMesh.tiles.isEmpty) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];

  // Draw tile portal geometry structures across all mapped navmesh chunks
  for (final tile in navMesh.tiles.values) {
    if (tile == null) continue;

    final List<DebugPrimitive> tilePrimitives = createNavMeshTilePortalsHelper(tile);
    primitives.addAll(tilePrimitives);
  }

  return primitives;
}

// Add final Map<String, OffMeshConnection?> offMeshConnections; to your NavMesh layout:
// class NavMesh { ... final Map<String, OffMeshConnection?> offMeshConnections; }

List<DebugPrimitive> createSearchNodesHelper(SearchNodePool? nodePool) {
  final List<DebugPrimitive> primitives = [];
  if (nodePool == null || nodePool.isEmpty) {
    return primitives;
  }

  const double yOffset = 0.5;
  final List<double> pointPositions = [];
  final List<double> pointColors = [];
  final List<double> linePositions = [];
  final List<double> lineColors = [];

  // Color mappings
  const List<double> pointColor = [1.0, 192.0 / 255.0, 0.0];
  const List<double> lineColor = [1.0, 192.0 / 255.0, 0.0];

  // 1. Collect point positions from the pool
  for (final nodes in nodePool.values) {
    for (int i = 0; i < nodes.length; i++) {
      final node = nodes[i];
      pointPositions.addAll([
        node.position[0],
        node.position[1] + yOffset,
        node.position[2],
      ]);
      pointColors.addAll(pointColor);
    }
  }

  // 2. Form connection traces back to parent nodes
  for (final nodes in nodePool.values) {
    for (int i = 0; i < nodes.length; i++) {
      final node = nodes[i];
      if (node.parentNodeRef == null || node.parentState == null) continue;

      final parentNodes = nodePool[node.parentNodeRef];
      if (parentNodes == null) continue;

      SearchNode? parent;
      for (int j = 0; j < parentNodes.length; j++) {
        if (parentNodes[j].state == node.parentState) {
          parent = parentNodes[j];
          break;
        }
      }

      if (parent == null) continue;

      linePositions.addAll([
        node.position[0], node.position[1] + yOffset, node.position[2],
        parent.position[0], parent.position[1] + yOffset, parent.position[2],
      ]);
      lineColors.addAll([...lineColor, ...lineColor]);
    }
  }

  if (pointPositions.isNotEmpty) {
    primitives.add(
      DebugPoints(
        positions: pointPositions,
        colors: pointColors,
        size: 0.01,
        transparent: true,
        opacity: 1.0,
      ),
    );
  }

  if (linePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: linePositions,
        colors: lineColors,
        transparent: true,
        opacity: 0.5,
        lineWidth: 2.0,
      ),
    );
  }

  return primitives;
}

List<DebugPrimitive> createNavMeshOffMeshConnectionsHelper(NavMesh? navMesh) {
  if (navMesh == null || navMesh.offMeshConnections.isEmpty) {
    return [];
  }

  final List<DebugPrimitive> primitives = [];
  const int arcSegments = 16;
  const int circleSegments = 20;

  final List<double> arcPositions = [];
  final List<double> arcColors = [];
  final List<double> circlePositions = [];
  final List<double> circleColors = [];

  const List<double> arcColor = [255.0 / 255.0, 196.0 / 255.0, 0.0 / 255.0];
  const List<double> pillarColor = [0.0 / 255.0, 48.0 / 255.0, 64.0 / 255.0];
  const List<double> oneWayEndColor = [220.0 / 255.0, 32.0 / 255.0, 16.0 / 255.0];

  double lerp(double start, double end, double t) => start + (end - start) * t;

  // Local helper closure function to build endpoints anchors
  void addCircle(Vector3 center, List<double> color, double radius) {
    for (int i = 0; i < circleSegments; i++) {
      final double a0 = (i / circleSegments) * math.pi * 2.0;
      final double a1 = ((i + 1) / circleSegments) * math.pi * 2.0;

      final double x0 = center[0] + math.cos(a0) * radius;
      final double z0 = center[2] + math.sin(a0) * radius;
      final double x1 = center[0] + math.cos(a1) * radius;
      final double z1 = center[2] + math.sin(a1) * radius;

      circlePositions.addAll([
        x0, center[1] + 0.1, z0,
        x1, center[1] + 0.1, z1,
      ]);
      circleColors.addAll([...color, ...color]);
    }

    // Connect standard vertical base structural columns
    circlePositions.addAll([
      center[0], center[1], center[2],
      center[0], center[1] + 0.2, center[2],
    ]);
    circleColors.addAll([...pillarColor, ...pillarColor]);
  }

  for (final con in navMesh.offMeshConnections.values) {
    if (con == null) continue;

    final Vector3 start = con.start;
    final Vector3 end = con.end;
    final double radius = con.radius;

    // 1. Generate curved connection traces
    for (int i = 0; i < arcSegments; i++) {
      final double t0 = i / arcSegments;
      final double t1 = (i + 1) / arcSegments;

      final double x0 = lerp(start[0], end[0], t0);
      final double y0 = lerp(start[1], end[1], t0) + math.sin(t0 * math.pi) * 0.25;
      final double z0 = lerp(start[2], end[2], t0);

      final double x1 = lerp(start[0], end[0], t1);
      final double y1 = lerp(start[1], end[1], t1) + math.sin(t1 * math.pi) * 0.25;
      final double z1 = lerp(start[2], end[2], t1);

      arcPositions.addAll([x0, y0, z0, x1, y1, z1]);
      arcColors.addAll([...arcColor, ...arcColor]);
    }

    // 2. Append path indicator pointers if direction matches start-to-end setups
    if (con.direction == OffMeshConnectionDirection.startToEnd) {
      const double tMid = 0.5;
      final double xMid = lerp(start[0], end[0], tMid);
      final double yMid = lerp(start[1], end[1], tMid) + 0.25;
      final double zMid = lerp(start[2], end[2], tMid);

      final double dirX = end[0] - start[0];
      final double dirZ = end[2] - start[2];
      
      // Math.hypot(x, y) becomes math.sqrt(x*x + y*y) in Dart
      final double hypotLen = math.sqrt(dirX * dirX + dirZ * dirZ);
      final double len = (hypotLen == 0.0) ? 1.0 : hypotLen;
      
      final double nx = dirX / len;
      final double nz = dirZ / len;
      const double back = 0.3;

      arcPositions.addAll([
        xMid, yMid, zMid,
        xMid - nx * back + nz * back * 0.5, yMid - 0.05, zMid - nz * back - nx * back * 0.5,
        xMid, yMid, zMid,
        xMid - nx * back - nz * back * 0.5, yMid - 0.05, zMid - nz * back + nx * back * 0.5,
      ]);
      
      for (int k = 0; k < 4; k++) {
        arcColors.addAll(arcColor);
      }
    }

    // 3. Render endpoints boundary rings
    addCircle(start, arcColor, radius);
    addCircle(
      end, 
      con.direction == OffMeshConnectionDirection.bidirectional ? arcColor : oneWayEndColor, 
      radius,
    );
  }

  if (arcPositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: arcPositions,
        colors: arcColors,
        transparent: true,
        opacity: 0.9,
        lineWidth: 2.0,
      ),
    );
  }

  if (circlePositions.isNotEmpty) {
    primitives.add(
      DebugLines(
        positions: circlePositions,
        colors: circleColors,
        transparent: true,
        opacity: 0.8,
        lineWidth: 1.5,
      ),
    );
  }

  return primitives;
}
