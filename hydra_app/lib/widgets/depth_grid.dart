import 'package:flutter/material.dart';

class DepthGridWidget extends StatelessWidget {
  final List<int> depth;

  const DepthGridWidget({super.key, required this.depth});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF141B2D), borderRadius: BorderRadius.circular(6), border: Border.all(color: const Color(0xFF1E2A42))),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        const Text('DEPTH (8x8)', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), letterSpacing: 1, fontWeight: FontWeight.w600)),
        const SizedBox(height: 6),
        AspectRatio(
          aspectRatio: 1,
          child: GridView.builder(
            physics: const NeverScrollableScrollPhysics(),
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 8, mainAxisSpacing: 1, crossAxisSpacing: 1),
            itemCount: 64,
            itemBuilder: (_, i) {
              if (depth.length != 64) return Container(color: const Color(0xFF1E2A42));
              final mn = depth.where((v) => v > 0).fold(9999, (a, b) => a < b ? a : b);
              final mx = depth.fold(0, (a, b) => a > b ? a : b);
              final range = (mx - mn).clamp(1, 9999);
              final t = ((depth[i] - mn) / range).clamp(0.0, 1.0);
              return Container(
                decoration: BoxDecoration(
                  color: Color.lerp(const Color(0xFFFF4444), const Color(0xFF4444FF), t),
                  borderRadius: BorderRadius.circular(1),
                ),
              );
            },
          ),
        ),
      ]),
    );
  }
}
