import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../services/websocket_service.dart';
import '../models/sensor_data.dart';
import 'posterior_chart.dart';
import 'force_bars.dart';
import 'depth_grid.dart';
import 'decision_log.dart';

class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final ws = context.watch<WebSocketService>();
    final d = ws.data;

    return Scaffold(
      body: SafeArea(
        child: Column(children: [
          _header(ws, d),
          Expanded(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(8),
              child: Column(children: [
                Row(children: [
                  Expanded(child: _qualityCard(d)),
                  const SizedBox(width: 8),
                  Expanded(child: _planCard(d)),
                ]),
                const SizedBox(height: 8),
                PosteriorChart(belief: d.belief, plausibility: d.plausibility),
                const SizedBox(height: 8),
                Row(children: [
                  Expanded(child: ForceBars(forces: d.forces, slip: d.slip, hdReady: d.hdReady)),
                  const SizedBox(width: 8),
                  Expanded(child: DepthGridWidget(depth: d.depth)),
                ]),
                const SizedBox(height: 8),
                DecisionLogWidget(log: d.log),
              ]),
            ),
          ),
          _controls(ws),
        ]),
      ),
    );
  }

  Widget _header(WebSocketService ws, SensorData d) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
      color: const Color(0xFF141B2D),
      child: Row(children: [
        const Text('HYDRA Grasp', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold, color: Color(0xFF4FC3F7))),
        const Spacer(),
        Text(d.strategy, style: const TextStyle(color: Color(0xFFF39C12), fontWeight: FontWeight.bold)),
        const SizedBox(width: 12),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 4),
          decoration: BoxDecoration(color: _stateColor(d.state), borderRadius: BorderRadius.circular(10)),
          child: Text(d.state, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600)),
        ),
        const SizedBox(width: 8),
        Icon(Icons.circle, size: 10, color: ws.connected ? Colors.green : Colors.red),
      ]),
    );
  }

  Widget _qualityCard(SensorData d) {
    return _card('Quality & Prediction', Column(children: [
      Text('${(d.quality * 100).round()}%', style: const TextStyle(fontSize: 28, fontWeight: FontWeight.bold)),
      _bar(d.quality, d.quality > 0.6 ? Colors.green : d.quality > 0.3 ? Colors.orange : Colors.red),
      const SizedBox(height: 8),
      const Text('P(success)', style: TextStyle(fontSize: 10, color: Color(0xFF5A6F8F))),
      Text('${(d.successProb * 100).round()}%', style: const TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: Color(0xFF9B59B6))),
      _bar(d.successProb, const Color(0xFF9B59B6)),
    ]));
  }

  Widget _planCard(SensorData d) {
    final p = d.plan;
    return _card('Plan', Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
      _planRow('Strategy', p?.strategy ?? '--'),
      _planRow('Force', '${p?.force.toStringAsFixed(1) ?? '--'} N'),
      _planRow('Speed', p?.speed.toStringAsFixed(2) ?? '--'),
      _planRow('Aperture', '${p?.aperture.toStringAsFixed(0) ?? '--'} mm'),
      _planRow('Ramp', '${p?.ramp.toStringAsFixed(1) ?? '--'} N/s'),
    ]));
  }

  Widget _planRow(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(children: [
        Text('$label: ', style: const TextStyle(fontSize: 11, color: Color(0xFF5A6F8F))),
        Text(value, style: const TextStyle(fontSize: 11, fontWeight: FontWeight.bold)),
      ]),
    );
  }

  Widget _controls(WebSocketService ws) {
    return Container(
      padding: const EdgeInsets.all(8),
      child: Row(mainAxisAlignment: MainAxisAlignment.center, children: [
        _btn('START', const Color(0xFF27AE60), () => ws.sendCommand('start')),
        const SizedBox(width: 8),
        _btn('RELEASE', const Color(0xFFE67E22), () => ws.sendCommand('release')),
        const SizedBox(width: 8),
        _btn('E-STOP', const Color(0xFFE74C3C), () => ws.sendCommand('estop')),
        const SizedBox(width: 8),
        _btn('CAL', const Color(0xFF2980B9), () => ws.sendCommand('calibrate')),
      ]),
    );
  }

  Widget _btn(String label, Color color, VoidCallback onTap) {
    return ElevatedButton(
      onPressed: onTap,
      style: ElevatedButton.styleFrom(backgroundColor: color, padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10)),
      child: Text(label, style: const TextStyle(fontWeight: FontWeight.w600)),
    );
  }

  Widget _card(String title, Widget child) {
    return Container(
      padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF141B2D), borderRadius: BorderRadius.circular(6), border: Border.all(color: const Color(0xFF1E2A42))),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text(title, style: const TextStyle(fontSize: 10, color: Color(0xFF5A6F8F), letterSpacing: 1, fontWeight: FontWeight.w600)),
        const SizedBox(height: 6),
        child,
      ]),
    );
  }

  Widget _bar(double value, Color color) {
    return Container(
      height: 6, margin: const EdgeInsets.only(top: 4),
      decoration: BoxDecoration(color: const Color(0xFF1E2A42), borderRadius: BorderRadius.circular(3)),
      child: FractionallySizedBox(
        alignment: Alignment.centerLeft,
        widthFactor: value.clamp(0, 1),
        child: Container(decoration: BoxDecoration(color: color, borderRadius: BorderRadius.circular(3))),
      ),
    );
  }

  Color _stateColor(String state) {
    switch (state) {
      case 'HOLDING': return Colors.green;
      case 'ERROR': return Colors.red;
      case 'GRIPPING': case 'APPROACHING': return Colors.orange;
      default: return const Color(0xFF2980B9);
    }
  }
}
