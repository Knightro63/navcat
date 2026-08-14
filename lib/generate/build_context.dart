enum BuildContextLogType {
  info(0),
  warning(1),
  error(2);

  final int value;
  const BuildContextLogType(this.value);
}

class BuildContextLog {
  BuildContextLogType? type;
  String? message;

  BuildContextLog({
    this.type,
    this.message,
  });
}

class BuildContextTime {
  String? name;
  int? duration;

  BuildContextTime({
    this.name,
    this.duration,
  });
}

class BuildContextState {
  List<BuildContextLog>? logs;
  List<BuildContextTime>? times;
  Map<String, int>? startTimes;

  BuildContextState({
    this.logs,
    this.times,
    this.startTimes,
  });

  static BuildContextState create() {
    return BuildContextState(
      logs: [],
      times: [],
      startTimes: {},
    );
  }

  static void start(BuildContextState context, String name) {
    context.startTimes![name] = DateTime.now().millisecondsSinceEpoch;
  }

  static void end(BuildContextState context, String name) {
    final now = DateTime.now().millisecondsSinceEpoch;
    final start = context.startTimes![name];
    final duration = now - start!;
    context.startTimes!.remove(name);
    context.times!.add(BuildContextTime(name: name, duration: duration));
  }

  static void info(BuildContextState context, String message) {
    context.logs!.add(BuildContextLog(
      type: BuildContextLogType.info,
      message: message,
    ));
  }

  static void warn(BuildContextState context, String message) {
    context.logs!.add(BuildContextLog(
      type: BuildContextLogType.warning,
      message: message,
    ));
  }

  static void error(BuildContextState context, String message) {
    context.logs!.add(BuildContextLog(
      type: BuildContextLogType.error,
      message: message,
    ));
  }
}
