class SensorData {
  String state;
  String strategy;
  double quality;
  double successProb;
  bool slip;
  bool hdReady;
  List<double> belief;
  List<double> plausibility;
  double conflict;
  double ignorance;
  List<double> forces;
  List<int> depth;
  double? impLogMag;
  double? impPhase;
  PlanData? plan;
  List<String> log;

  SensorData({
    this.state = 'IDLE',
    this.strategy = '--',
    this.quality = 0,
    this.successProb = 0,
    this.slip = false,
    this.hdReady = false,
    List<double>? belief,
    List<double>? plausibility,
    this.conflict = 0,
    this.ignorance = 1,
    List<double>? forces,
    List<int>? depth,
    this.impLogMag,
    this.impPhase,
    this.plan,
    List<String>? log,
  })  : belief = belief ?? List.filled(6, 0),
        plausibility = plausibility ?? List.filled(6, 0),
        forces = forces ?? [0, 0, 0],
        depth = depth ?? [],
        log = log ?? [];

  factory SensorData.fromJson(Map<String, dynamic> j) {
    return SensorData(
      state: j['state'] ?? 'IDLE',
      strategy: j['strategy'] ?? '--',
      quality: (j['quality'] ?? 0).toDouble(),
      successProb: (j['success_prob'] ?? 0).toDouble(),
      slip: j['slip'] ?? false,
      hdReady: j['hd_ready'] ?? false,
      belief: (j['belief'] as List?)?.map((e) => (e as num).toDouble()).toList(),
      plausibility: (j['plaus'] as List?)?.map((e) => (e as num).toDouble()).toList(),
      conflict: (j['conflict'] ?? 0).toDouble(),
      ignorance: (j['ignorance'] ?? 1).toDouble(),
      forces: (j['forces'] as List?)?.map((e) => (e as num).toDouble()).toList(),
      depth: (j['depth'] as List?)?.map((e) => (e as num).toInt()).toList(),
      impLogMag: j['imp'] != null ? (j['imp']['log_mag'] as num?)?.toDouble() : null,
      impPhase: j['imp'] != null ? (j['imp']['phase'] as num?)?.toDouble() : null,
      plan: j['plan'] != null ? PlanData.fromJson(j['plan']) : null,
      log: (j['log'] as List?)?.map((e) => e.toString()).toList(),
    );
  }
}

class PlanData {
  String strategy;
  double force;
  double speed;
  double aperture;
  double ramp;

  PlanData({this.strategy = '--', this.force = 0, this.speed = 0, this.aperture = 0, this.ramp = 0});

  factory PlanData.fromJson(Map<String, dynamic> j) => PlanData(
    strategy: j['strat'] ?? '--',
    force: (j['force'] ?? 0).toDouble(),
    speed: (j['speed'] ?? 0).toDouble(),
    aperture: (j['aper'] ?? 0).toDouble(),
    ramp: (j['ramp'] ?? 0).toDouble(),
  );
}
