import 'package:flutter/material.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:cloud_firestore/cloud_firestore.dart';

// Màn hình hiển thị video stream trực tiếp qua WebView
class StreamVideoScreen extends StatefulWidget {
  @override
  State<StreamVideoScreen> createState() => _StreamVideoScreenState();
}

class _StreamVideoScreenState extends State<StreamVideoScreen> {
  late final WebViewController _webViewController;   // Controll cho WebView
  bool _isError = false;  // Cờ kiểm tra lỗi khi load trang
  bool _isLoading = true; // Cờ kiểm tra loading


   @override
  void initState() {
    super.initState();
    _loadUserAndStream(); // Gọi hàm loadStream ngay khi khởi tạo. Người dùng vào giao diện này cái là stream sẽ được load ngay
  }

  // Người dùng ở tầng nào sẽ xem được camera ở tầng ấy, vậy nên trước khi phát stream cần lấy thông tin người dùng xem họ đang ở tầng mấy
  Future<void> _loadUserAndStream() async {
    final user = FirebaseAuth.instance.currentUser;

    if (user != null) {
      try {
        // Lấy dữ liệu người dùng từ Firebase
        final userDoc = await FirebaseFirestore.instance
            .collection('users') 
            .doc(user.uid)
            .get();

        final data = userDoc.data();
        if (data != null && data.containsKey('level_id')) {
          // Lấy level_id của người dùng, chính là số tầng
          final levelId = int.parse(data['level_id'].toString()).toString();

          // Sau khi có số tầng thì tạo url stream tương ứng
          final streamUrl = 'http://103.69.97.153:8888/pi_tang_$levelId.html';

          // Cấu hình WebView
          _webViewController = WebViewController()
            ..setJavaScriptMode(JavaScriptMode.unrestricted)
            ..setNavigationDelegate(
              NavigationDelegate(
                onPageStarted: (_) {
                  setState(() => _isError = false);   // Đặt cờ lỗi bằng false khi load thành công
                },
                onWebResourceError: (error) {
                  setState(() => _isError = true);  // Thất bị thì đặt thành true
                },
              ),
            )
            ..loadRequest(Uri.parse(streamUrl));  // Load URL vào WebView
        } else {
          _isError = true;  // Nếu đến đây thì là do không có level_id trong CSDL trên Firebase
        }
      } catch (e) {   // Bắt lỗi nếu lấy dữ liệu từ Firebase thất bại
        print('Lỗi khi lấy level_id: $e');  
        _isError = true;
      }
    } else {
      _isError = true;  // Không có user thỏa mãn, người dùng chưa đăng nhập
    }

    setState(() {
      _isLoading = false;
    });
  }

  // Hàm tải lại WebView
  void _reloadPage() {
    _webViewController.reload();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Camera trực tiếp'),
        actions: [
          IconButton(onPressed: _reloadPage, icon: Icon(Icons.refresh)),
        ],
      ),
      body: _isLoading
          ? Center(child: CircularProgressIndicator())  // Hiển thị khi đang loading
          : _isError
              ? Center(
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(Icons.wifi_off, size: 60, color: Colors.grey),
                      SizedBox(height: 10),
                      Text('Không thể tải stream'), // Thông báo lỗi
                      SizedBox(height: 10),
                      ElevatedButton(
                        onPressed: _reloadPage, // Tải lại
                        child: Text('Thử lại'),
                      ),
                    ],
                  ),
                )
              : WebViewWidget(controller: _webViewController),
    );
  }
}
