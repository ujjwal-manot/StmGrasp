import 'package:flutter/material.dart';

class PosteriorChart extends StatelessWidget {
  final List<double> belief;
  final List<double> plausibility;

  const PosteriorChart({super.key, required this.belief, required this.plausibility});

  static const _names = ['Metal', 'Skin', 'Plastic', 'Wood', 'Glass', 'Cardboard'];
  static const _colors = [Color(0xFF3498DB), Color(0xFFF5B041), Color(0xFF9B59B6), Color(0xFF28B463), Color(0xFFEC7063), Color(0xFFD4AC0D)];

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF141B2D), borderRadius: BorderRadius.circular(6), border: Border.all(color: const Color(0xFF1E2A42))),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        const Text('MATERIAL POSTERIOR (D-S)', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), letterSpacing: 1, fontWeight: FontWeight.w600)),
        const SizedBox(height: 8),
        ...List.generate(6, (i) => _row(i)),
      ]),
    );
  }

  Widget _row(int i) {
    final bel = i < belief.length ? belief[i] : 0.0;
    final pl = i < plausibility.length ? plausibility[i] : 0.0;
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(children: [
        SizedBox(width: 65, child: Text(_names[i], textAlign: TextAlign.right, style: const TextStyle(fontSize: 11))),
        const SizedBox(width: 6),
        Expanded(
          child: Stack(children: [
            Container(height: 16, decoration: BoxDecoration(color: const Color(0xFF1E2A42), borderRadius: BorderRadius.circular(3))),
            FractionallySizedBox(
              widthFactor: pl.clamp(0, 1),
              child: Container(height: 16, decoration: BoxDecoration(color: _colors[i].withOpacity(0.2), borderRadius: BorderRadius.circular(3))),
            ),
            FractionallySizedBox(
              widthFactor: bel.clamp(0, 1),
              child: Container(height: 16, decoration: BoxDecoration(color: _colors[i], borderRadius: BorderRadius.circular(3))),
            ),
            Positioned(right: 4, top: 0, bottom: 0, child: Center(child: Text('${(bel * 100).round()}%', style: const TextStyle(fontSize: 9, color: Colors.white)))),
          ]),
        ),
      ]),
    );
  }
}
