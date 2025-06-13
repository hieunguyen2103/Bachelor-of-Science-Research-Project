import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/screens/DeviceSettingScreen.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/widget/SemiCircle.dart';
import 'package:fire_guard/widget/notifycationIconButton.dart';
import 'package:fire_guard/screens/streamVideo.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:syncfusion_flutter_gauges/gauges.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:intl/intl.dart';
import 'dart:async';
import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:firebase_messaging/firebase_messaging.dart';
import 'package:dropdown_button2/dropdown_button2.dart';
import 'package:shared_preferences/shared_preferences.dart';


class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() {
    return _MainSCreenState();
  }
}

class _MainSCreenState extends State<MainScreen> {
  bool _isAccountActivated = false;
  bool _isLoading = true;

  double? coLevel;
  double? smokeLevel;
  double? temperature;
  DateTime _lastUpdated = DateTime.now();
  DateTime _currentTime = DateTime.now();
  Timer? _clockTimer;

  // String? _selectedSensor;
  final List<String> _defaultSensorList  = ['Sensor 1', 'Sensor 2', 'Sensor 3'];
  List<String> _customSensorNames = [];
  List<String> _deviceNames = [];

  int _currentSensorIndex = 0;
  late PageController _sensorPageController;

  // Future<void> _loadSensorNames() async {
  //   try {
  //     final prefs = await SharedPreferences.getInstance();
  //     _customSensorNames = List.generate(_defaultSensorList.length, (i) {
  //       return prefs.getString('sensor_name_$i') ?? _defaultSensorList[i];
  //     });
  //   } catch (e) {
  //     debugPrint("❌ Lỗi khi load SharedPreferences: $e");
  //     // fallback nếu lỗi
  //     _customSensorNames = List.from(_defaultSensorList);
  //   }
  // }

  /**************************************** Hàm lấy tên thiết bị từ trên Server ************************************/
  /*****************************************************************************************************************/
  // Future<void> _loadSensorNames() async {
  //   final user = FirebaseAuth.instance.currentUser;
  //   if (user == null) return;

  //   try {
  //     final url = Uri.parse('http://103.69.97.153:5000/get-devices?user_id=${user.uid}');
  //     final response = await http.get(url);
  //     final prefs = await SharedPreferences.getInstance();

  //     if (response.statusCode == 200) {
  //       final data = json.decode(response.body);
  //       final List<dynamic> names = data['device_names'];
  //       final prefs = await SharedPreferences.getInstance();
  //       setState(() {
  //         _deviceNames = List<String>.from(names); // Lưu danh sách gốc
  //         // _customSensorNames = List<String>.from(names);
  //         _customSensorNames = List<String>.from(names.map((deviceName) {
  //           final customName = prefs.getString('sensor_name_$deviceName');
  //           return customName ?? deviceName;
  //         }));
  //       });
  //     } else {
  //       print('❌ Lỗi gọi get-devices: ${response.statusCode}');
  //       // _customSensorNames = List.from(_defaultSensorList); // fallback
  //       final prefs = await SharedPreferences.getInstance();
  //       setState(() {
  //         _customSensorNames = List.generate(_defaultSensorList.length, (i) {
  //           return prefs.getString('sensor_name_${_defaultSensorList[i]}') ?? _defaultSensorList[i];
  //         });
  //       });
  //       // _customSensorNames = List.generate(_defaultSensorList.length, (i) {
  //       //   return prefs.getString('sensor_name_${_defaultSensorList[i]}') ?? _defaultSensorList[i];
  //       // });
  //     }
  //   } catch (e) {
  //     print('❌ Lỗi lấy tên thiết bị: $e');
  //     setState(() {
  //       _customSensorNames = List.from(_defaultSensorList);
  //     });
  //     // _customSensorNames = List.from(_defaultSensorList); // fallback
  //   }
  // }

  Future<void> _loadSensorNames() async {
    final user = FirebaseAuth.instance.currentUser;
    if (user == null) return;

    try {
      final url = Uri.parse('http://103.69.97.153:5000/get-devices?user_id=${user.uid}');
      final response = await http.get(url);
      final prefs = await SharedPreferences.getInstance();

      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        final List<dynamic> names = data['device_names'];

        if (names != null && names.isNotEmpty) {
          setState(() {
            _deviceNames = List<String>.from(names);
            _customSensorNames = _deviceNames.map((deviceName) {
              final custom = prefs.getString('sensor_name_$deviceName');
              return custom ?? deviceName;
            }).toList();
          });
          return;
        }
      }

      // ❌ Nếu không có data hợp lệ → fallback
      debugPrint('❌ Lỗi gọi API hoặc không có dữ liệu');
      setState(() {
        _deviceNames = List.from(_defaultSensorList); // ❗ phải có dòng này
        _customSensorNames = _defaultSensorList.map((deviceName) {
          return prefs.getString('sensor_name_$deviceName') ?? deviceName;
        }).toList();
      });
    } catch (e) {
      print('❌ Lỗi lấy tên thiết bị: $e');
      final prefs = await SharedPreferences.getInstance();
      setState(() {
        _deviceNames = List.from(_defaultSensorList); // ❗ phải có dòng này
        _customSensorNames = _defaultSensorList.map((deviceName) {
          return prefs.getString('sensor_name_$deviceName') ?? deviceName;
        }).toList();
      });
    }
  }

  // Future<void> _editSensorName(int index) async {
  //   // final originalDeviceName = _customSensorNames[index];
  //   final controller = TextEditingController(text: _customSensorNames[index]);
  //   const maxLength = 18;
  //   int currentLength = controller.text.length;

  //   await showDialog(
  //     context: context,
  //     builder: (BuildContext dialogContext) {
  //       return StatefulBuilder(
  //         builder: (context, setState) {
  //           return AlertDialog(
  //             title: const Text('Đặt tên cảm biến'),
  //             content: Column(
  //               mainAxisSize: MainAxisSize.min,
  //               crossAxisAlignment: CrossAxisAlignment.start,
  //               children: [
  //                 TextField(
  //                   controller: controller,
  //                   maxLength: maxLength,
  //                   onChanged: (text) {
  //                     setState(() {
  //                       currentLength = text.length;
  //                     });
  //                   },
  //                   decoration: const InputDecoration(
  //                     hintText: 'Nhập tên mới',
  //                     counterText: '', // Ẩn đếm ký tự mặc định
  //                   ),
  //                 ),
  //                 Align(
  //                   alignment: Alignment.centerRight,
  //                   child: Text(
  //                     '$currentLength/$maxLength',
  //                     style: TextStyle(fontSize: 12, color: Colors.grey[600]),
  //                   ),
  //                 ),
  //               ],
  //             ),
  //             actions: [
  //               TextButton(
  //                 onPressed: () => Navigator.pop(dialogContext),
  //                 child: const Text('Hủy'),
  //               ),
  //               TextButton(
  //                 onPressed: () async {
  //                   final newName = controller.text.trim();
  //                   if (newName.isNotEmpty) {
  //                     try {
  //                       final prefs = await SharedPreferences.getInstance();
  //                       final deviceName  = _deviceNames[index];
  //                       await prefs.setString('sensor_name_$deviceName', newName);
  //                       // await prefs.setString('sensor_name_$index', newName);
  //                       setState(() {
  //                         _customSensorNames[index] = newName;
  //                       });
  //                     } catch (e) {
  //                       debugPrint('Lỗi khi lưu SharedPreferences: $e');
  //                     }
  //                     FocusScope.of(dialogContext).unfocus();
  //                     Navigator.pop(dialogContext);
  //                   }
  //                 },
  //                 child: const Text('Lưu'),
  //               ),
  //             ],
  //           );
  //         },
  //       );
  //     },
  //   );
  // }

  Future<void> _editSensorName(int index) async {
    final controller = TextEditingController(text: _customSensorNames[index]);
    const maxLength = 18;
    int currentLength = controller.text.length;

    await showDialog(
      context: context,
      builder: (BuildContext dialogContext) {
        return StatefulBuilder(
          builder: (context, setState) {
            return AlertDialog(
              title: const Text('Đặt tên cảm biến'),
              content: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  TextField(
                    controller: controller,
                    maxLength: maxLength,
                    onChanged: (text) {
                      setState(() {
                        currentLength = text.length;
                      });
                    },
                    decoration: const InputDecoration(
                      hintText: 'Nhập tên mới',
                      counterText: '', // Ẩn đếm ký tự mặc định
                    ),
                  ),
                  Align(
                    alignment: Alignment.centerRight,
                    child: Text(
                      '$currentLength/$maxLength',
                      style: TextStyle(fontSize: 12, color: Colors.grey[600]),
                    ),
                  ),
                ],
              ),
              actions: [
                TextButton(
                  onPressed: () => Navigator.pop(dialogContext),
                  child: const Text('Hủy'),
                ),
                TextButton(
                  onPressed: () async {
                    final newName = controller.text.trim();
                    if (newName.isNotEmpty) {
                      try {
                        final prefs = await SharedPreferences.getInstance();

                        // Nếu deviceNames chưa có, fallback sang defaultSensorList
                        final deviceName = (index < _deviceNames.length)
                            ? _deviceNames[index]
                            : (index < _defaultSensorList.length)
                                ? _defaultSensorList[index]
                                : null;

                        if (deviceName != null) {
                          await prefs.setString('sensor_name_$deviceName', newName);
                          setState(() {
                            _customSensorNames[index] = newName;
                          });
                        } else {
                          debugPrint('❌ Không tìm thấy tên thiết bị tại index $index');
                        }
                      } catch (e) {
                        debugPrint('Lỗi khi lưu SharedPreferences: $e');
                      }

                      FocusScope.of(dialogContext).unfocus();
                      Navigator.pop(dialogContext);
                    }
                  },
                  child: const Text('Lưu'),
                ),
              ],
            );
          },
        );
      },
    );
  }

  void _initFCM() async {
    FirebaseMessaging messaging = FirebaseMessaging.instance;

    // Yêu cầu quyền nhận thông báo (Android 13+)
    await messaging.requestPermission();

    // Lấy token thiết bịư
    final fcmToken = await messaging.getToken();
    print('🟢 FCM Token: $fcmToken');

    // Gửi token về server
    final user = FirebaseAuth.instance.currentUser;

    if (user != null && fcmToken != null) {
      try {
        // Lấy level_id từ Firestore
        final userDoc = await FirebaseFirestore.instance
            .collection('users')  // thay bằng tên collection của bạn nếu khác
            .doc(user.uid)
            .get();

        final levelId = userDoc.data()?['level_id'] ?? 'unknown';

        final bodyData = {
          'user_id': user.uid,
          'token': fcmToken,
          'level_id': levelId,
        };

        print('📤 GỬI LÊN SERVER: ${jsonEncode(bodyData)}');

        // await http.post(
        //   Uri.parse('http://103.69.97.153:5000/register-token'),
        //   headers: {'Content-Type': 'application/json'},
        //   body: jsonEncode(bodyData),
        // );

        final response = await http.post(
          Uri.parse('http://103.69.97.153:5000/register-token'),
          headers: {'Content-Type': 'application/json'},
          body: jsonEncode(bodyData),
        );

        if (response.statusCode == 200) {
          print('✅ Gửi token thành công!');
        } else {
          print('❌ Gửi token thất bại: ${response.statusCode} - ${response.body}');
        }

      } catch (e) {
          print('❌ Lỗi khi gửi token (Connection r): $e');
      }
    }
    else {
      print('⚠️ user hoặc fcmToken bị null. Không gửi được.');
    }

    // Lắng nghe khi app đang mở
    FirebaseMessaging.onMessage.listen((RemoteMessage message) async {
      print("🔔 Nhận thông báo: ${message.notification?.title}");

      if (message.notification != null) {
        // Đóng dialog đang mở (nếu có)
        if (Navigator.canPop(context)) {
          Navigator.of(context, rootNavigator: true).pop();
        }

        // Chờ 100ms để đảm bảo dialog trước đã đóng
        await Future.delayed(Duration(milliseconds: 100));
        
        showDialog(
          context: context,
          builder: (_) => AlertDialog(
            title: Text(message.notification!.title ?? "Thông báo"),
            content: Text(message.notification!.body ?? "Không có nội dung"),
            actions: [
              TextButton(
                onPressed: () => Navigator.pop(context),
                child: const Text("Đóng"),
              )
            ],
          ),
        );
      }
    });
  }

  @override
  void initState() {
    super.initState();
    _customSensorNames = [];
    _sensorPageController = PageController(initialPage: 50);
    _currentSensorIndex = 0;
    _loadSensorNames().then((_) {
      // Chỉ cập nhật khi load xong
      if (_customSensorNames.isNotEmpty) {
        _sensorPageController.jumpToPage(50);
        setState(() {
          _currentSensorIndex = 50 % _customSensorNames.length;
        });
      }
    });
    // _customSensorNames = List.from(_defaultSensorList); // fallback

    //_loadSensorNames();

    // WidgetsBinding.instance.addPostFrameCallback((_) {
    //   _loadSensorNames();
    // });
    // _sensorPageController = PageController(initialPage: _currentSensorIndex);
    _loadUserAndData();
    _initFCM();
    _clockTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      setState(() {
        _currentTime = DateTime.now();
      });
    });
  }

  @override
  void dispose() {
    _clockTimer?.cancel();
    _sensorPageController.dispose();
    super.dispose();
  }

  Future<void> _loadUserAndData() async {
    final user = FirebaseAuth.instance.currentUser;
    if (user == null) return;

    try {
      final userDoc = await FirebaseFirestore.instance
          .collection('users')
          .doc(user.uid)
          .get();

      final data = userDoc.data();
      if (data != null && data['activated'] == true) {
        setState(() {
          _isAccountActivated = true;
        });

        final currentDeviceName = _deviceNames[_currentSensorIndex];
        // final url = Uri.parse('http://103.69.97.153:5000/get-sensor-data?user_id=${user.uid}');
        final url = Uri.parse('http://103.69.97.153:5000/get-sensor-data?device_name=$currentDeviceName');
        final response = await http.get(url);
        if(response.statusCode == 200)
        {
          final jsonData = json.decode(response.body);
          setState(() {
            coLevel = (jsonData['co'] as num?)?.toDouble() ?? 0;
            smokeLevel = (jsonData['smokes'] as num?)?.toDouble() ?? 0;
            temperature = (jsonData['temp'] as num?)?.toDouble() ?? 0;
            _lastUpdated = DateTime.now();
          });
        }
        else
        {
          print('Lỗi khi gọi API: ${response.statusCode}');
        }

        // final sensorDoc = await FirebaseFirestore.instance
        //     .collection('users')
        //     .doc(user.uid)
        //     .collection('data_sensor')
        //     .doc('00000000')
        //     .get();

        // final sensorData = sensorDoc.data();
        // if (sensorData != null) {
        //   setState(() {
        //     coLevel = (sensorData['Co_level'] as num).toDouble();
        //     smokeLevel = (sensorData['Smoke_level'] as num).toDouble();
        //     _lastUpdated = DateTime.now();
        //   });
        // }
      }
    } catch (e) {
      print('Error loading user or sensor data: $e');
    }

    setState(() {
      _isLoading = false;
    });
  }

  void _setScreen(BuildContext context, String identifier) {
    Navigator.of(context).pop();
    if (identifier == 'Account') {
      Navigator.of(context).push(
          MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
    } else if (identifier == 'Device') {
      Navigator.of(context).push(
          MaterialPageRoute(builder: (ctx) => const DeviceSettingScreen()));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Home page'),
        actions: [
          IconButton(
            onPressed: () {
              Navigator.of(context).push(
                MaterialPageRoute(builder: (ctx) => StreamVideoScreen()),
              );
            },
            icon: const Icon(Icons.videocam),
            tooltip: 'Camera',
          ),
          const NotificationIconButton(),
        ],
      ),
      drawer: MainDrawerScreen(
        onSelectScreen: _setScreen,
      ),

      body: _isLoading
        ? const Center(child: CircularProgressIndicator())
        : _isAccountActivated
            ? Column(
                children: [
                  const SizedBox(height: 12),
                  Text(
                    'Fire Guard',
                    style: GoogleFonts.robotoSlab(
                      fontSize: 28,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),

                  // ✅ PageView cho toàn bộ block sensor
                  Expanded(
                    child: PageView.builder(
                      controller: _sensorPageController,
                      // itemCount: _defaultSensorList .length,
                      itemCount: _customSensorNames.length > 0 ? 100 : 1,
                      onPageChanged: (index) {
                        setState(() {
                          _currentSensorIndex = index % _customSensorNames .length;
                          // TODO: Load lại data theo sensor nếu cần
                        });
                        _loadUserAndData(); // Load lại data theo thiết bị mới
                      },
                      itemBuilder: (context, index) {
                        final sensorIndex = index % _customSensorNames .length;
                        return SingleChildScrollView(
                          padding: const EdgeInsets.symmetric(horizontal: 16),
                          child: Column(
                            children: [
                              // Hiển thị tên cảm biến
                              Container(
                                padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
                                decoration: BoxDecoration(
                                  color: Colors.grey[850],
                                  borderRadius: BorderRadius.circular(12),
                                  border: Border.all(color: Colors.white),
                                ),

                                child: Row(
                                  mainAxisSize: MainAxisSize.min,
                                  children: [
                                    Text(
                                      _customSensorNames[sensorIndex],
                                      style: const TextStyle(
                                        color: Colors.white,
                                        fontSize: 18,
                                        fontWeight: FontWeight.bold,
                                      ),
                                    ),
                                    const SizedBox(width: 8),
                                    GestureDetector(
                                      onTap: () => _editSensorName(sensorIndex),
                                      child: const Icon(Icons.edit, size: 18, color: Colors.white),
                                    ),
                                  ],
                                ),

                                // child: Text(
                                //   _defaultSensorList [sensorIndex],
                                //   style: const TextStyle(
                                //     color: Colors.white,
                                //     fontSize: 18,
                                //     fontWeight: FontWeight.bold,
                                //   ),
                                // ),

                              ),
                              const SizedBox(height: 16),

                              // Block Smoke + CO
                              SizedBox(
                                height: 240,
                                child: Row(
                                  children: [
                                    Expanded(
                                      child: Padding(
                                        padding: const EdgeInsets.all(8.0),
                                        child: buildGauge(
                                          'Smoke',
                                          smokeLevel ?? 0,
                                          0,
                                          100,
                                          [
                                            GaugeRange(startValue: 0, endValue: 20, color: Colors.green),
                                            GaugeRange(startValue: 21, endValue: 60, color: Colors.orange),
                                            GaugeRange(startValue: 61, endValue: 100, color: Colors.red),
                                          ],
                                          '',
                                        ),
                                      ),
                                    ),
                                    Expanded(
                                      child: Padding(
                                        padding: const EdgeInsets.all(8.0),
                                        child: buildGauge(
                                          'CO',
                                          coLevel ?? 0,
                                          0,
                                          100,
                                          [
                                            GaugeRange(startValue: 0, endValue: 10, color: Colors.green),
                                            GaugeRange(startValue: 11, endValue: 35, color: Colors.orange),
                                            GaugeRange(startValue: 36, endValue: 100, color: Colors.red),
                                          ],
                                          '',
                                        ),
                                      ),
                                    ),
                                  ],
                                ),
                              ),

                              const SizedBox(height: 8),

                              // Block Temperature
                              Center(
                                child: SizedBox(
                                  height: 200,
                                  width: 200,
                                  child: buildGauge(
                                    'Temperature',
                                    temperature ?? 0,
                                    0,
                                    100,
                                    [
                                      GaugeRange(startValue: 0, endValue: 30, color: Colors.green),
                                      GaugeRange(startValue: 31, endValue: 50, color: Colors.orange),
                                      GaugeRange(startValue: 51, endValue: 100, color: Colors.red),
                                    ],
                                    '',
                                  ),
                                ),
                              ),
                            ],
                          ),
                        );
                      },
                    ),
                  ),
                ],
              )
            : const Center(
                child: Text(
                  'Please activate your account to use features',
                  style: TextStyle(color: Colors.red),
                ),
              ),

      // body: _isLoading
      //     ? const Center(child: CircularProgressIndicator())
      //     : _isAccountActivated
      //         ? Padding(
      //             padding: const EdgeInsets.all(16.0),
      //             child: SingleChildScrollView(
      //               child: Column(
      //                 crossAxisAlignment: CrossAxisAlignment.center,
      //                 children: [
      //                   Text(
      //                     'Fire Guard',
      //                     style: GoogleFonts.robotoSlab(
      //                       fontSize: 28,
      //                       fontWeight: FontWeight.bold,
      //                     ),
      //                   ),
      //                   const SizedBox(height: 12,),

      //                   //*************** Phần hiển thị nút chọn cảm biến ***********************/
      //                   SizedBox(
      //                     height: 60,
      //                     child: PageView.builder(
      //                       controller: _sensorPageController,
      //                       itemCount: _defaultSensorList .length,
      //                       onPageChanged: (index) {
      //                         setState(() {
      //                           _currentSensorIndex = index;
      //                           // TODO: Load lại data theo sensor nếu cần
      //                         });
      //                       },
      //                       itemBuilder: (context, index) {
      //                         return Center(
      //                           child: Container(
      //                             padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 10),
      //                             decoration: BoxDecoration(
      //                               color: Colors.grey[850],
      //                               borderRadius: BorderRadius.circular(12),
      //                               border: Border.all(color: Colors.white),
      //                             ),
      //                             child: Text(
      //                               _defaultSensorList [index],
      //                               style: const TextStyle(
      //                                 color: Colors.white,
      //                                 fontSize: 18,
      //                                 fontWeight: FontWeight.bold,
      //                               ),
      //                             ),
      //                           ),
      //                         );
      //                       },
      //                     ),
      //                   ),
      //                   /**************************************************************************/

      //                   const SizedBox(height: 16),
      //                   SizedBox(
      //                     height: 240,
      //                     child: Row(
      //                       children: [
      //                         Expanded(
      //                           child: Padding(
      //                             padding: const EdgeInsets.all(8.0),
      //                             child: buildGauge(
      //                               'Smoke',
      //                               smokeLevel ?? 0,
      //                               0,
      //                               100,
      //                               [
      //                                 GaugeRange(startValue: 0, endValue: 20, color: Colors.green),
      //                                 GaugeRange(startValue: 21, endValue: 60, color: Colors.orange),
      //                                 GaugeRange(startValue: 61, endValue: 100, color: Colors.red),
      //                               ],
      //                               '',
      //                             ),
      //                           ),
      //                         ),
      //                         Expanded(
      //                           child: Padding(
      //                             padding: const EdgeInsets.all(8.0),
      //                             child: buildGauge(
      //                               'CO',
      //                               coLevel ?? 0,
      //                               0,
      //                               100,
      //                               [
      //                                 GaugeRange(startValue: 0, endValue: 10, color: Colors.green),
      //                                 GaugeRange(startValue: 11, endValue: 35, color: Colors.orange),
      //                                 GaugeRange(startValue: 36, endValue: 100, color: Colors.red),
      //                               ],
      //                               '',
      //                             ),
      //                           ),
      //                         ),
      //                       ],
      //                     ),
      //                   ),
                    
      //                   const SizedBox(height: 8),
      //                   Center(
      //                   child: SizedBox(
      //                     height: 200,
      //                     width: 200,
      //                     child: buildGauge(
      //                       'Temperature',
      //                       temperature ?? 0,
      //                       0,
      //                       100,
      //                       [
      //                         GaugeRange(startValue: 0, endValue: 30, color: Colors.green),
      //                         GaugeRange(startValue: 31, endValue: 50, color: Colors.orange),
      //                         GaugeRange(startValue: 51, endValue: 100, color: Colors.red),
      //                       ],
      //                       '',
      //                     ),
      //                   ),
      //                 ),
                    
      //                   // const SizedBox(height: 20),
      //                   // _buildStatusWarning(temperature, coLevel, smokeLevel),
      //                   // const SizedBox(height: 16),
      //                   // Text(
      //                   //   '⏱ Bây giờ là: ${DateFormat('HH:mm:ss dd/MM/yyyy').format(_currentTime)}',
      //                   //   style: const TextStyle(fontSize: 14, color: Colors.grey),
      //                   // )
      //                 ],
      //               ),
      //             ),
      //           )
      //         : const Center(
      //             child: Text(
      //               'Please activate your account to use features',
      //               style: TextStyle(color: Colors.red),
      //             ),
      //           ),

    );
  }

  Widget buildGauge(
    String title,
    double value,
    double min,
    double max,
    List<GaugeRange> ranges,
    String unit,
  ) {
    return SfRadialGauge(
      title: GaugeTitle(
        text: title,
        textStyle: GoogleFonts.roboto(
          fontSize: 16,
          fontWeight: FontWeight.bold,
        ),
        alignment: GaugeAlignment.center,
      ),
      axes: <RadialAxis>[
        RadialAxis(
          showTicks: false,
          showLabels: false,
          minimum: min,
          maximum: max,
          radiusFactor: 0.9,
          axisLineStyle: AxisLineStyle(
            thickness: 0.18,
            thicknessUnit: GaugeSizeUnit.factor,
            cornerStyle: CornerStyle.bothCurve,
            color: Colors.grey.shade300,
          ),
          ranges: ranges,
          pointers: <GaugePointer>[
            NeedlePointer(
              value: value,
              needleColor: Colors.deepPurple,
              knobStyle: KnobStyle(color: Colors.deepPurple),
            ),
          ],
          annotations: <GaugeAnnotation>[
            GaugeAnnotation(
              angle: 90,
              positionFactor: 0.6,
              widget: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Text(
                    value.toStringAsFixed(1),
                    style: GoogleFonts.robotoMono(
                      fontSize: 20,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  if (unit.isNotEmpty)
                    Text(
                      unit,
                      style: GoogleFonts.roboto(fontSize: 14),
                    ),
                ],
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildStatusWarning(double? temperature, double? co, double? smoke) {
    String coStatus = 'Chưa có dữ liệu';
    String smokeStatus = 'Chưa có dữ liệu';
    String tempStatus = 'Chưa có dữ liệu';

    if (co != null) {
      if (co > 35) {
        coStatus = '⚠️ Mức CO cao! Nguy hiểm!';
      } else if (co > 10) {
        coStatus = 'CO ở mức trung bình';
      } else {
        coStatus = '✅ Mức CO an toàn';
      }
    }

    if (smoke != null) {
      if (smoke > 60) {
        smokeStatus = '🚨 Phát hiện khói dày!';
      } else if (smoke > 20) {
        smokeStatus = '⚠️ Mức khói tăng cao';
      } else {
        smokeStatus = '✅ Mức khói ổn định';
      }
    }

    if (temperature != null) {
      if (temperature > 50) {
        tempStatus = '🔥 Nhiệt độ rất cao!';
      } else if (temperature > 30) {
        tempStatus = '⚠️ Nhiệt độ khá nóng';
      } else {
        tempStatus = '✅ Nhiệt độ ổn định';
      }
    }

    return Column(
      children: [
        Text(coStatus, style: const TextStyle(fontSize: 16)),
        const SizedBox(height: 8),
        Text(smokeStatus, style: const TextStyle(fontSize: 16)),
        const SizedBox(height: 8),
        Text(tempStatus, style: const TextStyle(fontSize: 16)),
      ],
    );
  }
}