import 'package:flutter/material.dart';

class ForceBars extends StatelessWidget {
  final List<double> forces;
  final bool slip;
  final bool hdReady;

  const ForceBars({super.key, required this.forces, this.slip = false, this.hdReady = false});

  static const _colors = [Color(0xFFE74C3C), Color(0xFF27AE60), Color(0xFF3498DB)];
  static const _labels = ['F1', 'F2', 'F3'];

  @override
  Widget build(BuildContext context) {
    double maxF = forces.fold(0.0, (a, b) => a > b ? a : b);
    if (maxF < 0.5) maxF = 2;

    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF141B2D), borderRadius: BorderRadius.circular(6), border: Border.all(color: const Color(0xFF1E2A42))),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Row(children: [
          const Text('FORCE', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), letterSpacing: 1, fontWeight: FontWeight.w600)),
          const Spacer(),
          if (hdReady) const Text('HD', style: TextStyle(fontSize: 9, color: Color(0xFF9B59B6), fontWeight: FontWeight.bold)),
          if (slip) const Padding(padding: EdgeInsets.only(left: 6), child: Text('SLIP!', style: TextStyle(fontSize: 10, color: Color(0xFFE74C3C), fontWeight: FontWeight.bold))),
        ]),
        const SizedBox(height: 8),
        SizedBox(
          height: 100,
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            crossAxisAlignment: CrossAxisAlignment.end,
            children: List.generate(3, (i) {
              final f = i < forces.length ? forces[i] : 0.0;
              final h = (f / maxF * 80).clamp(0, 80).toDouble();
              return Column(mainAxisAlignment: MainAxisAlignment.end, children: [
                Text('${f.toStringAsFixed(1)}N', style: const TextStyle(fontSize: 9)),
                const SizedBox(height: 2),
                Container(width: 30, height: h, decoration: BoxDecoration(color: _colors[i], borderRadius: BorderRadius.circular(3))),
                const SizedBox(height: 2),
                Text(_labels[i], style: const TextStyle(fontSize: 9, color: Color(0xFF5A6F8F))),
              ]);
            }),
          ),
        ),
      ]),
    );
  }
}
