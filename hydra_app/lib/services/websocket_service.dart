import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import '../models/sensor_data.dart';

class WebSocketService extends ChangeNotifier {
  WebSocketChannel? _channel;
  SensorData _data = SensorData();
  bool _connected = false;
  Timer? _reconnectTimer;
  String _host = '192.168.4.1';

  SensorData get data => _data;
  bool get connected => _connected;
  String get host => _host;

  void connect({String? host}) {
    if (host != null) _host = host;
    _disconnect();
    try {
      _channel = WebSocketChannel.connect(Uri.parse('ws://$_host/ws'));
      _connected = true;
      notifyListeners();
      _channel!.stream.listen(
        (msg) {
          try {
            _data = SensorData.fromJson(jsonDecode(msg));
            notifyListeners();
          } catch (_) {}
        },
        onDone: _onDisconnect,
        onError: (_) => _onDisconnect(),
      );
    } catch (_) {
      _onDisconnect();
    }
  }

  void _onDisconnect() {
    _connected = false;
    notifyListeners();
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 2), connect);
  }

  void _disconnect() {
    _reconnectTimer?.cancel();
    _channel?.sink.close();
    _channel = null;
    _connected = false;
  }

  void sendCommand(String cmd) {
    if (_connected && _channel != null) {
      _channel!.sink.add(jsonEncode({'cmd': cmd}));
    }
  }

  @override
  void dispose() {
    _disconnect();
    super.dispose();
  }
}
