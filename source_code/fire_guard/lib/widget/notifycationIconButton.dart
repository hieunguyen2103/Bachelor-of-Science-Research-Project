import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

class NotificationIconButton extends StatefulWidget {
  const NotificationIconButton({super.key});

  @override
  State<NotificationIconButton> createState() => _NotificationIconButtonState();
}

class _NotificationIconButtonState extends State<NotificationIconButton> {
  final List<Map<String, dynamic>> _alertHistory = [];

  Future<void> _loadAlerts(BuildContext context) async {
    const apiUrl = 'http://yourserver.com/api/latest-alert'; // sửa URL

    try {
      final response = await http.get(Uri.parse(apiUrl));
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        if (_isValidAlert(data)) {
          setState(() {
            // Nếu chưa có hoặc khác thời gian thì thêm
            if (_alertHistory.isEmpty ||
                data['timestamp'] != _alertHistory.first['timestamp']) {
              _alertHistory.insert(0, data);
            }
          });

          _showBottomSheet(context);
        } else {
          _showBottomSheet(context); // vẫn mở sheet nhưng không có dữ liệu
        }
      } else {
        _showBottomSheet(context);
      }
    } catch (e) {
      _showBottomSheet(context);
    }
  }

  bool _isValidAlert(dynamic data) {
    return data != null &&
        data is Map &&
        data['type'] != null &&
        data['location'] != null &&
        data['timestamp'] != null;
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
      onPressed: () => _loadAlerts(context),
    );
  }
}
