import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:shared_preferences/shared_preferences.dart';

class NotificationIconButton extends StatefulWidget {
  const NotificationIconButton({super.key});

  @override
  State<NotificationIconButton> createState() => _NotificationIconButtonState();
}

class _NotificationIconButtonState extends State<NotificationIconButton> {
  List<Map<String, dynamic>> _alertHistory = [];

  bool _isSheetOpen = false;

  @override
  void initState() {
    super.initState();
    _loadAlertsFromLocal();

    // Đăng ký topic chung để nhận thông báo từ FCM
    FirebaseMessaging.instance.subscribeToTopic('fire_guard');

    // Nhận thông báo khi app đang mở
    FirebaseMessaging.onMessage.listen((RemoteMessage message) {
      if (message.notification != null) {
        final title = message.notification!.title ?? 'Thông báo';
        final body = message.notification!.body ?? '';
        _handlePushNotification(title, body);
      }
    });
  }

  Future<void> _loadAlertsFromLocal() async 
  {
    final prefs = await SharedPreferences.getInstance();
    final List<String> stored = prefs.getStringList('alert_history') ?? [];

    final List<Map<String, dynamic>> parsed =
        stored.map((e) => jsonDecode(e) as Map<String, dynamic>).toList();

    setState(() {
      _alertHistory = parsed;
    });
  }

  Future<void> _saveAlertsToLocal() async
  {
    final prefs = await SharedPreferences.getInstance();
    final List<String> data = _alertHistory.map((e) => jsonEncode(e)).toList();
    await prefs.setStringList('alert_history', data);
  }

  // void _handlePushNotification(String title, String body) {
  //   setState(() {
  //     _alertHistory.insert(0, {
  //       'type': title,
  //       'location': body,
  //       'timestamp': DateTime.now().toString(),
  //     });
  //   });

  //   _showBottomSheet(context);
  // }

  void _handlePushNotification(String title, String body) {
    final newAlert = {
      'type': title,
      'location': body,
      'timestamp': DateTime.now().toIso8601String(),
    };

    setState(() {
      _alertHistory.insert(0, newAlert);
      _alertHistory = _alertHistory.take(10).toList(); // Giới hạn 10 thông báo
    });

    _saveAlertsToLocal(); // Ghi vào local
    _showBottomSheet(context);
  }

  void _showBottomSheet(BuildContext context) {
    if(_isSheetOpen) return;

    _isSheetOpen = true;

    showModalBottomSheet(
      context: context,
      isScrollControlled: false,
      backgroundColor: Colors.white,
      shape: const RoundedRectangleBorder(
        borderRadius: BorderRadius.vertical(top: Radius.circular(16)),
      ),
      builder: (_) => SizedBox(
        height: 400,
        child: Column(
          children: [
            const SizedBox(height: 12),
            const Text(
              'Thông báo',
              style: TextStyle(fontWeight: FontWeight.bold, fontSize: 18),
            ),
            const Divider(),
            Expanded(
              child: _alertHistory.isEmpty
                  ? const Center(child: Text("Không có cảnh báo nào."))
                  : ListView.builder(
                      itemCount: _alertHistory.length,
                      itemBuilder: (ctx, index) {
                        final alert = _alertHistory[index];
                        final timestamp = DateTime.tryParse(alert['timestamp'] ?? '');
                        final formattedTime = timestamp != null
                            ? '${timestamp.hour.toString().padLeft(2, '0')}:${timestamp.minute.toString().padLeft(2, '0')} '
                              '${timestamp.day.toString().padLeft(2, '0')}/${timestamp.month.toString().padLeft(2, '0')}'
                            : 'Không rõ thời gian';

                        return ListTile(
                          leading: const Icon(Icons.warning_amber_rounded, color: Colors.red),
                          title: Text("${alert['type']}"),
                          subtitle: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text("${alert['location']}"),
                              Text(formattedTime),
                              // Text("${alert['timestamp']}"),
                            ],
                          ),
                        );
                      },
                    ),
            ),
          ],
        ),
      ),
    ).whenComplete((){
      _isSheetOpen = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    return IconButton(
      icon: const Icon(Icons.notifications),
      color: Theme.of(context).colorScheme.primary,
      onPressed: () => _showBottomSheet(context),  // chỉ mở sheet, không gọi API nữa
    );
  }
}