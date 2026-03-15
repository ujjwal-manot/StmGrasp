import 'package:flutter/material.dart';

class DecisionLogWidget extends StatelessWidget {
  final List<String> log;

  const DecisionLogWidget({super.key, required this.log});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF141B2D), borderRadius: BorderRadius.circular(6), border: Border.all(color: const Color(0xFF1E2A42))),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        const Text('DECISION LOG', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), letterSpacing: 1, fontWeight: FontWeight.w600)),
        const SizedBox(height: 6),
        Container(
          constraints: const BoxConstraints(maxHeight: 120),
          child: SingleChildScrollView(
            reverse: true,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: log.isEmpty
                  ? [const Text('Waiting...', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), fontFamily: 'monospace'))]
                  : log.map((l) => Padding(
                      padding: const EdgeInsets.symmetric(vertical: 1),
                      child: Text(l, style: const TextStyle(fontSize: 10, color: Color(0xFF88FFAA), fontFamily: 'monospace')),
                    )).toList(),
            ),
          ),
        ),
      ]),
    );
  }
}
