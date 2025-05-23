import 'package:fire_guard/controll/authGateControll.dart';
import 'package:fire_guard/screens/authScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:fire_guard/screens/splashScreen.dart';
import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'firebase_options.dart';
import 'package:firebase_messaging/firebase_messaging.dart';

void main() async{
  WidgetsFlutterBinding.ensureInitialized();  // đảm bảo binding Flutter được khởi tạo trước khi chạy các hàm async.
  await Firebase.initializeApp(  // Hàm này để khởi tạo Firebase trong ứng dụng trước khi sử dụng dịch vụ của Firebase
    options: DefaultFirebaseOptions.currentPlatform,  // lấy cấu hình Firebase tương ứng với từng nền tảng, đã tạo sẵn bằng tool flutterfire configure.
  );
  // Lắng nghe thông báo khi app tắt
  FirebaseMessaging.onBackgroundMessage(_firebaseMessagingBackgroundHandler);
  runApp(const App());
}

Future<void> _firebaseMessagingBackgroundHandler(RemoteMessage message) async {
  await Firebase.initializeApp();
  print('[BG]Thông báo khi app tắt: ${message.messageId}');
}


class App extends StatelessWidget {
  const App({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'FireGuard',
      theme: ThemeData().copyWith(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
            seedColor: const Color.fromARGB(255, 63, 17, 177)),
      ),
      home: const AuthGateControll(),
      // home: StreamBuilder(  // Cái này giúp lắng nghe luồng dữ liệu (stream) và tự động cập nhật giao diện UI mỗi khi có dữ liệu mới phát sinh từ stream đó.
      //   stream: FirebaseAuth.instance.authStateChanges(), // Đây là nguồn dữ liệu mà StreamBuilder sẽ theo dõi
      //   builder: (ctx, snapshot) {
      //     if(snapshot.connectionState == ConnectionState.waiting)
      //     {
      //       return const SplashScreen();  // cái màn hình chờ thôi
      //     }
      //     if(snapshot.hasData)  // có dự liệu thì vào MainScreen
      //     {
      //       print("Log in success!");
      //       return const MainScreen();
      //     }
      //     return const AuthScreen();  // Không có dữ liệu gì tức là đăng xuất rồi
      //   },
      // ),
    );
  }
}