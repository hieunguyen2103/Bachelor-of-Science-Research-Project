import 'package:flutter/material.dart';

// Màn hình hiển thị đơn giản khi khởi động app, trong lúc chờ load dữ liệu
class SplashScreen extends StatelessWidget
{
  const SplashScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Home page'),
      ),
      body: const Center(
        child: Text('Loading...'),
      ),
    );
  }
}