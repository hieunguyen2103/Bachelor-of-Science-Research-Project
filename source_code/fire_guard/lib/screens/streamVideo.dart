// import 'package:cloud_firestore/cloud_firestore.dart';
// import 'package:flutter/material.dart';
// import 'package:video_player/video_player.dart';
// import 'package:webview_flutter/webview_flutter.dart';

// class StreamVideoScreen extends StatefulWidget
// {
//   @override
//   State<StatefulWidget> createState() {
//     return _StreamVideoScreenState();
//   }
// }

// class _StreamVideoScreenState extends State<StreamVideoScreen>
// {
//   late VideoPlayerController _controller;   // biến _controller là đối tượng của VideoPlayerController dùng để điều khiển việc phát video.
//   late Future<void> _initializeVideoPlayerFuture;   // Đây là 1 biến future giúp theo dõi quá trình khởi tạo video player, khi video sẵn sàng thì biến này sẽ ok
//   late final WebViewController _webViewController;
//   bool _isError = false;

//   void initState()    // Phương thức khởi tạo lớp State, được gọi khi widget tạo lần đầu tiên
//   {
//     super.initState();

//     // _controller = VideoPlayerController.networkUrl(Uri.parse('http://192.168.1.103:8081/'));   // Khởi tạo 1 video từ URL
//     // _initializeVideoPlayerFuture = _controller.initialize();    // Khởi tạo video player đồng thời trả về 1 future mà chúng ta sẽ theo dõi để biết khi nào video đã sẵn sàng phát
//     // _controller.setLooping(true);   // Đặt video tự động lặp lại sau khi kết thúc để video stream liên tục
//     // _controller.play();   // Bắt đầu phát video ngay khi sẵn sàng

//     _webViewController = WebViewController()
//       ..setJavaScriptMode(JavaScriptMode.unrestricted)
//       ..setNavigationDelegate(
//         NavigationDelegate(
//           onPageStarted: (_){
//             setState(() => _isError = false);
//           },
//           onWebResourceError: (error) {
//             setState(() => _isError = true);
//           },
//         ),
//       )
//       //..loadRequest(Uri.parse('http://192.168.1.103:8081/'));
//       ..loadRequest(Uri.parse('http://103.69.97.153:8888/hls/cam.m3u8'));
//   }

//   void _reloadPage()
//   {
//     _webViewController.reload();
//   }

//   // Dọn dẹp tài nguyên khi widget này bị hủy
//   @override
//   void dispose() {
//     // TODO: implement dispose
//     super.dispose();
//     _controller.dispose();
//   }

//   @override
//   Widget build(BuildContext context) {
//     // TODO: implement build
//     return Scaffold(
//       appBar: AppBar(
//         title: Text('Video Stream'),
//         actions: [
//           IconButton(
//             onPressed: _reloadPage, 
//             icon: Icon(Icons.refresh),
//           )
//         ],
//       ),
//       body: _isError
//         ? Center(
//           child: Column(
//             mainAxisSize: MainAxisSize.min,
//             children: [
//               Icon(Icons.wifi_off, size: 60, color: Colors.grey),
//               SizedBox(height: 10,),
//               Text(
//                 'Can not connect to stream',
//                 style: TextStyle(fontSize: 16),
//               ),
//               SizedBox(height: 10,),
//               ElevatedButton(
//                 onPressed: _reloadPage, 
//                 child: Text('Try again'),
//               ),
//             ],
//           ),
//         )
//         : WebViewWidget(controller: _webViewController),
//     );
//   }

//   // @override
//   // Widget build(BuildContext context) {
//   //   // TODO: implement build
//   //   return Scaffold(
//   //     appBar: AppBar(
//   //       title: Text('Video Stream'),
//   //     ),
//   //     body: WebViewWidget(controller: _webViewController),

//   //     // body: Center(
//   //     //   child: FutureBuilder<void>(
//   //     //     future: _initializeVideoPlayerFuture,   // Truyền vào future muốn theo dõi 
//   //     //     builder: (context, snapshot)  {
//   //     //       if(snapshot.connectionState == ConnectionState.done)   // Kiểm tra nếu video đã được tải xong và sẵn sàng để phát, nếu có thì tạo 1 AspectRatio để hiển thị video
//   //     //       {
//   //     //         return AspectRatio(
//   //     //           aspectRatio: _controller.value.aspectRatio,   // Giữ tỷ lệ khung hình của video phù hợp với tỷ lệ gốc của nó
//   //     //           child: VideoPlayer(_controller),    // Dùng để phát video, sử dụng _controller để điều khiển việc phát
//   //     //         );
//   //     //       }
//   //     //       else
//   //     //       {
//   //     //         return Center(child: CircularProgressIndicator(),);   // Nếu video chưa tải xong thì quay vòng vòng
//   //     //       }
//   //     //     }
//   //     //   ),
//   //     // ),
//   //   );
//   // }

// }


import 'package:flutter/material.dart';
import 'package:video_player/video_player.dart';

class StreamVideoScreen extends StatefulWidget {
  @override
  State<StreamVideoScreen> createState() => _StreamVideoScreenState();
}

class _StreamVideoScreenState extends State<StreamVideoScreen> {
  late VideoPlayerController _controller;
  late Future<void> _initializeVideoPlayerFuture;
  bool _isError = false;

  @override
  void initState() {
    super.initState();

    // Khởi tạo video HLS từ đường dẫn m3u8
    _controller = VideoPlayerController.network(
      'http://103.69.97.153:8888/hls/cam.m3u8',
    );

    _initializeVideoPlayerFuture = _controller.initialize().then((_) {
      setState(() {});
      _controller.setLooping(true);
      _controller.play();
    }).catchError((error) {
      setState(() => _isError = true);
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  void _reloadVideo() {
    setState(() {
      _isError = false;
      _controller.pause();
      _controller.dispose();

      _controller = VideoPlayerController.network(
        'http://103.69.97.153:8888/hls/cam.m3u8',
      );

      _initializeVideoPlayerFuture = _controller.initialize().then((_) {
        setState(() {});
        _controller.setLooping(true);
        _controller.play();
      }).catchError((error) {
        setState(() => _isError = true);
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text('Video Stream'),
        actions: [
          IconButton(
            onPressed: _reloadVideo,
            icon: Icon(Icons.refresh),
          ),
        ],
      ),
      body: _isError
          ? Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(Icons.wifi_off, size: 60, color: Colors.grey),
                  SizedBox(height: 10),
                  Text(
                    'Can not connect to stream',
                    style: TextStyle(fontSize: 16),
                  ),
                  SizedBox(height: 10),
                  ElevatedButton(
                    onPressed: _reloadVideo,
                    child: Text('Try again'),
                  ),
                ],
              ),
            )
          : FutureBuilder<void>(
              future: _initializeVideoPlayerFuture,
              builder: (context, snapshot) {
                if (snapshot.connectionState == ConnectionState.done) {
                  return Center(
                    child: AspectRatio(
                      aspectRatio: _controller.value.aspectRatio,
                      child: VideoPlayer(_controller),
                    ),
                  );
                } else if (snapshot.hasError) {
                  return Center(child: Text('Error loading video'));
                } else {
                  return Center(child: CircularProgressIndicator());
                }
              },
            ),
    );
  }
}



// import 'package:flutter/material.dart';
// import 'package:better_player/better_player.dart';

// class StreamVideoScreen extends StatefulWidget {
//   @override
//   State<StreamVideoScreen> createState() => _StreamVideoScreenState();
// }

// class _StreamVideoScreenState extends State<StreamVideoScreen> {
//   late BetterPlayerController _betterPlayerController;
//   bool _isError = false;

//   final String videoUrl = 'http://103.69.97.153:8888/hls/cam.m3u8';

//   @override
//   void initState() {
//     super.initState();
//     _setupBetterPlayer();
//   }

//   void _setupBetterPlayer() {
//     try {
//       BetterPlayerDataSource dataSource = BetterPlayerDataSource(
//         BetterPlayerDataSourceType.network,
//         videoUrl,
//         liveStream: true,
//       );

//       _betterPlayerController = BetterPlayerController(
//         BetterPlayerConfiguration(
//           autoPlay: true,
//           looping: true,
//           aspectRatio: 16 / 9,
//           fit: BoxFit.contain,
//           controlsConfiguration: BetterPlayerControlsConfiguration(
//             enablePlayPause: true,
//             enableFullscreen: true,
//             enablePlaybackSpeed: false,
//           ),
//           // bufferingConfiguration: BetterPlayerBufferingConfiguration(
//           //   minBufferMs: 1000, // 1 giây
//           //   maxBufferMs: 2000, // 2 giây
//           //   bufferForPlaybackMs: 500,
//           //   bufferForPlaybackAfterRebufferMs: 1000,
//           // ),
//         ),
//         betterPlayerDataSource: dataSource,
//       );
//     } catch (e) {
//       setState(() {
//         _isError = true;
//       });
//     }
//   }

//   void _reloadVideo() {
//     setState(() {
//       _isError = false;
//       _betterPlayerController.dispose();
//       _setupBetterPlayer();
//     });
//   }

//   @override
//   void dispose() {
//     _betterPlayerController.dispose();
//     super.dispose();
//   }

//   @override
//   Widget build(BuildContext context) {
//     return Scaffold(
//       appBar: AppBar(
//         title: Text('Video Stream'),
//         actions: [
//           IconButton(
//             onPressed: _reloadVideo,
//             icon: Icon(Icons.refresh),
//           ),
//         ],
//       ),
//       body: _isError
//           ? Center(
//               child: Column(
//                 mainAxisSize: MainAxisSize.min,
//                 children: [
//                   Icon(Icons.wifi_off, size: 60, color: Colors.grey),
//                   SizedBox(height: 10),
//                   Text(
//                     'Can not connect to stream',
//                     style: TextStyle(fontSize: 16),
//                   ),
//                   SizedBox(height: 10),
//                   ElevatedButton(
//                     onPressed: _reloadVideo,
//                     child: Text('Try again'),
//                   ),
//                 ],
//               ),
//             )
//           : Center(
//               child: AspectRatio(
//                 aspectRatio: 16 / 9,
//                 child: BetterPlayer(
//                   controller: _betterPlayerController,
//                 ),
//               ),
//             ),
//     );
//   }
// }









