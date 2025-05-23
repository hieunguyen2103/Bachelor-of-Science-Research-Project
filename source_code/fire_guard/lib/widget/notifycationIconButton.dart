import 'package:flutter/material.dart';
import 'package:firebase_messaging/firebase_messaging.dart';

class NotificationIconButton extends StatefulWidget {
  const NotificationIconButton({super.key});

  @override
  State<NotificationIconButton> createState() => _NotificationIconButtonState();
}

class _NotificationIconButtonState extends State<NotificationIconButton> {
  final List<Map<String, dynamic>> _alertHistory = [];

  @override
  void initState() {
    super.initState();

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

  void _handlePushNotification(String title, String body) {
    setState(() {
      _alertHistory.insert(0, {
        'type': title,
        'location': body,
        'timestamp': DateTime.now().toString(),
      });
    });

    _showBottomSheet(context);
  }

  void _showBottomSheet(BuildContext context) {
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
                        return ListTile(
                          leading: const Icon(Icons.warning_amber_rounded, color: Colors.red),
                          title: Text("${alert['type']}"),
                          subtitle: Column(
                            crossAxisAlignment: CrossAxisAlignment.start,
                            children: [
                              Text("${alert['location']}"),
                              Text("${alert['timestamp']}"),
                            ],
                          ),
                        );
                      },
                    ),
            ),
          ],
        ),
      ),
    );
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