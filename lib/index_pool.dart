IndexPool createIndexPool(){
  return IndexPool();
}

class IndexPool{
  List<int> free = [];
  int counter = 0;

  static int requestIndex(IndexPool indexPool) {
    if (indexPool.free.isNotEmpty) {
      return indexPool.free.removeLast();
    }

    return indexPool.counter++;
  }

  static void releaseIndex(IndexPool indexPool, int index) {
    indexPool.free.add(index);
  }
}


