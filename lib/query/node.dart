import 'dart:math';

enum NodeType {
  poly(0),
  offMesh(1);

  final int value;
  const NodeType(this.value);
}

/// Invalid node reference constant
const invalidNodeRef = -1;

const typeBits = 1;
const nodeIndexBits = 31;
const sequenceBits = 20;

// masks for 32-bit operations (bits 1-32)
const typeMask = 0x1; // bit 1
const nodeIndexMask = 0x7FFFFFFF; // bits 2-32 (31 bits)
const nodeIndexShift = typeBits; // 1

// sequence number uses bits beyond 32-bit boundary (bits 33-52)
const sequenceShift = typeBits + nodeIndexBits; // 32
const sequenceMask = (1 << sequenceBits) - 1; // 0xFFFFF (20 bits)

// maximum values for each field based on bit allocation
const maxNodeIndex = nodeIndexMask; // 2147483647 (31 bits: 2^31 - 1)
const maxSequence = sequenceMask; // 1048575 (20 bits: 2^20 - 1)

/// Serializes a node reference from its components
int serNodeRef(NodeType type, int nodeIndex, int sequence) {
  // NOTE: mask inputs to avoid accidental overflow
  final t = type.value & typeMask;
  final n = nodeIndex & nodeIndexMask;
  final s = sequence & sequenceMask;
  
  // Pack as: [type: 1 bit][nodeIndex: 31 bits][sequence: 20 bits]
  // Use multiplication instead of bitwise shift to avoid 32-bit truncation, we encode sequence in higher bits
  return t + (n * 2) + (s * pow(2, sequenceShift)).toInt();
}

/// Gets the node type from a node reference
int getNodeRefType(int ref) {
  return (ref & typeMask);
}

/// Gets the node index from a node reference
int getNodeRefIndex(int ref) {
  return (ref >>> nodeIndexShift) & nodeIndexMask;
}

/// Gets the sequence number from a node reference
int getNodeRefSequence(int ref) {
  return (ref / pow(2, sequenceShift)).floor() & sequenceMask;
}
