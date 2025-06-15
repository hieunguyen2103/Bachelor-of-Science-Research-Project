import 'package:fire_guard/controll/authGateControll.dart';
import 'package:fire_guard/screens/authScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:fire_guard/screens/splashScreen.dart';
import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'firebase_options.dart';
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'dart:convert';

final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();
bool isDialogShowing = false;

void main() async{
  WidgetsFlutterBinding.ensureInitialized();  // đảm bảo binding Flutter được khởi tạo trước khi chạy các hàm async.

  FirebaseMessaging.onBackgroundMessage(_firebaseMessagingBackgroundHandler);

  await Firebase.initializeApp(  // Hàm này để khởi tạo Firebase trong ứng dụng trước khi sử dụng dịch vụ của Firebase
    options: DefaultFirebaseOptions.currentPlatform,  // lấy cấu hình Firebase tương ứng với từng nền tảng, đã tạo sẵn bằng tool flutterfire configure.
  );

  // Lắng nghe thông báo khi app tắt
  // FirebaseMessaging.onBackgroundMessage(_firebaseMessagingBackgroundHandler);

  // Lắng nghe thông báo khi app đang chạy (foreground)
  FirebaseMessaging.onMessage.listen(_handleForegroundMessage);
  
  runApp(const App());
}

Future<void> _handleForegroundMessage(RemoteMessage message) async {
  print("🔔 [FG] Nhận thông báo: ${message.notification?.title}");

  final prefs = await SharedPreferences.getInstance();
  final title = message.notification?.title ?? 'Thông báo';
  final body = message.notification?.body ?? '';
  final timestamp = DateTime.now().toIso8601String();

  // 📝 Lưu thông báo vào local
  final alert = {
    'type': title,
    'location': body,
    'timestamp': timestamp,
  };

  final oldList = prefs.getStringList('alert_history') ?? [];
  final newList = [jsonEncode(alert), ...oldList].take(10).toList();
  await prefs.setStringList('alert_history', newList);

  // ❗Đóng popup cũ nếu có
  final navigator = navigatorKey.currentState;
  if (navigator == null) return;

  if (isDialogShowing && navigator.canPop()) {
    navigator.pop();
    isDialogShowing = false;
    await Future.delayed(const Duration(milliseconds: 100));
  }

  isDialogShowing = true;

  showDialog(
    context: navigator.context,
    builder: (_) => AlertDialog(
      title: Text(title),
      content: Text(body),
      actions: [
        TextButton(
          onPressed: () {
            isDialogShowing = false;
            navigator.pop();
          },
          child: const Text("Đóng"),
        )
      ],
    ),
  );
}

Future<void> _firebaseMessagingBackgroundHandler(RemoteMessage message) async {
  await Firebase.initializeApp();
  print('[BG]Thông báo khi app tắt: ${message.messageId}');

  final prefs = await SharedPreferences.getInstance();
  final title = message.notification?.title ?? 'Thông báo';
  final body = message.notification?.body ?? '';
  final timestamp = DateTime.now().toIso8601String();

  final alert = {
    'type': title,
    'location': body,
    'timestamp': timestamp,
  };

  final oldList = prefs.getStringList('alert_history') ?? [];
  final newList = [jsonEncode(alert), ...oldList].take(10).toList();
  await prefs.setStringList('alert_history', newList);
}



class App extends StatelessWidget {
  const App({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorKey: navigatorKey,
      title: 'FireGuard',
      theme: ThemeData().copyWith(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
            seedColor: const Color.fromARGB(255, 63, 17, 177)),
      ),
      home: const AuthGateControll(),
    );
  }
}