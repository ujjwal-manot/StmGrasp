import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'services/websocket_service.dart';
import 'widgets/dashboard.dart';

void main() {
  runApp(
    ChangeNotifierProvider(
      create: (_) => WebSocketService()..connect(),
      child: const HydraApp(),
    ),
  );
}

class HydraApp extends StatelessWidget {
  const HydraApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'HYDRA Grasp',
      debugShowCheckedModeBanner: false,
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF0A0E17),
        cardColor: const Color(0xFF141B2D),
        colorScheme: const ColorScheme.dark(
          primary: Color(0xFF4FC3F7),
          secondary: Color(0xFFF39C12),
        ),
      ),
      home: const DashboardScreen(),
    );
  }
}
