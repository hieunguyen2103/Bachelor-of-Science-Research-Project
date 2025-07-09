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
  bool _sensorApiSuccess = false;

  double? coLevel;
  double? smokeLevel;
  double? temperature;
  String _currentStatus = 'offline';
  DateTime _lastUpdated = DateTime.now();
  DateTime _currentTime = DateTime.now();
  Timer? _clockTimer;
  Timer? _dataUpdateTimer;

  final List<String> _defaultSensorList  = ['Sensor 1', 'Sensor 2', 'Sensor 3'];
  List<String> _customSensorNames = [];
  List<String> _deviceNames = [];
  List<String> _statusList = [];

  int _currentSensorIndex = 0;
  late PageController _sensorPageController;

  void _initFCM() async {
    FirebaseMessaging messaging = FirebaseMessaging.instance;

    // Yêu cầu quyền nhận thông báo (Android 13+)
    await messaging.requestPermission();

    // Lấy token thiết bịư
    final fcmToken = await messaging.getToken();
    print('FCM Token: $fcmToken');

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

        print('GỬI LÊN SERVER: ${jsonEncode(bodyData)}');
        final response = await http.post(
          Uri.parse('http://103.69.97.153:5000/register-token'),  // Gửi token tài khoản lên cho server
          headers: {'Content-Type': 'application/json'},
          body: jsonEncode(bodyData),
        );

        if (response.statusCode == 200) {
          print('Gửi token thành công!');
        } else {
          print('Gửi token thất bại: ${response.statusCode} - ${response.body}');
        }

      } catch (e) {
          print('Lỗi khi gửi token (Connection r): $e');
      }
    }
    else {
      print('user hoặc fcmToken bị null. Không gửi được.');
    }
  }

  @override
  void initState() {
    super.initState();
    _checkAccountActivated();
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

    _loadUserAndData();
    _dataUpdateTimer = Timer.periodic(const Duration(seconds: 2), (_) { // Cập nhật data 2 giây 1 lần
      _loadUserAndData();
    });
    _initFCM();
    _clockTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      setState(() {
        _currentTime = DateTime.now();
      });
    });
  }

  /**************************************** Hàm lấy tên thiết bị từ trên Server ************************************/
  /*****************************************************************************************************************/
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

        setState(() {
          _sensorApiSuccess = true;  // API gọi thành công
          if (names != null && names.isNotEmpty) {
            _deviceNames = List<String>.from(names);
            _customSensorNames = _deviceNames.map((deviceName) {
              final custom = prefs.getString('sensor_name_$deviceName');
              return custom ?? deviceName;
            }).toList();
             // Tạo list status mặc định ban đầu
            _statusList = List<String>.filled(_deviceNames.length, 'offline');
          } else {
            _deviceNames = [];
            _customSensorNames = [];
            _statusList = [];
          }
        });
        return;
      }
    } catch (e) {
      print('Lỗi lấy tên thiết bị: $e');
      setState(() {
        _sensorApiSuccess = false;
        _deviceNames = [];
        _customSensorNames = [];
      });
    }
  }

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
                          debugPrint('Không tìm thấy tên thiết bị tại index $index');
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

  @override
  void dispose() {
    _clockTimer?.cancel();
    _dataUpdateTimer?.cancel();
    _sensorPageController.dispose();
    super.dispose();
  }

  Future<void> _checkAccountActivated() async {
    final user = FirebaseAuth.instance.currentUser;
    if (user == null)
    {
      setState(() {
        _isLoading = false;
      });
      return;
    }

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
      }
      else
      {
        setState(() {
          _isAccountActivated = false;
        });
      }
    } catch (e) {
      print('Lỗi khi kiểm tra activated: $e');
    }

    // Kế thúc kiểm tra activate account
    setState(() {
      _isLoading = false;
    });
  }

  Future<void> _loadUserAndData() async {
    final user = FirebaseAuth.instance.currentUser;
    if (user == null || !_isAccountActivated) return;

    if (_deviceNames.isEmpty || _currentSensorIndex >= _deviceNames.length) {
      print('⚠️ Không có thiết bị hoặc index quá giới hạn');
      return;
    }

    try {
      final currentDeviceName = _deviceNames[_currentSensorIndex];
      final url = Uri.parse('http://103.69.97.153:5000/get-sensor-data?device_name=$currentDeviceName');
      final response = await http.get(url);
      if(response.statusCode == 200)
      {
        final jsonData = json.decode(response.body);
        print("Dữ liệu từ server: $jsonData (${jsonData.runtimeType})");

        Map<String, dynamic> data;
        if (jsonData is List && jsonData.isNotEmpty) {
          data = jsonData[0]; // Lấy bản ghi đầu tiên nếu có nhiều dòng
        } else if (jsonData is Map<String, dynamic>) {
          data = jsonData;
        } else {
          print("Dữ liệu trả về không đúng định dạng: $jsonData");
          return;
        }
        setState(() {
          coLevel = (data['co'] as num?)?.toDouble() ?? 0;
          smokeLevel = (data['smokes'] as num?)?.toDouble() ?? 0;
          temperature = (data['temp'] as num?)?.toDouble() ?? 0;
          _statusList[_currentSensorIndex] = data['status'] ?? 'offline';
          print(">>> Status cho sensor ${_deviceNames[_currentSensorIndex]}: ${_statusList[_currentSensorIndex]}");
          _lastUpdated = DateTime.now();
        });
      }
      else
      {
        print('Lỗi khi gọi API: ${response.statusCode}');
      }
    } catch (e) {
      print('Lỗi khi lấy dữ liệu: $e');
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

                  // PageView cho toàn bộ block sensor
                  Expanded(
                    child: _customSensorNames.isEmpty
                    ? Center(
                        child: Text(
                          _sensorApiSuccess
                          ? 'Chưa có thiết bị nào được đăng ký'
                          : 'Lỗi kết nối, hiện không thể đọc dữ liệu từ cảm biến!',
                          style: TextStyle(fontSize: 16, color: Colors.grey),
                          textAlign: TextAlign.center,
                        ),
                      )
                    : PageView.builder(
                      controller: _sensorPageController,
                      itemCount: 100,
                      onPageChanged: (index) {
                        setState(() {
                          _currentSensorIndex = index % _customSensorNames .length;
                        });
                        _loadUserAndData(); // Load lại data theo thiết bị mới
                      },
                      itemBuilder: (context, index) {
                        final sensorIndex = index % _customSensorNames .length;
                        final status = (_statusList.length > sensorIndex) ? _statusList[sensorIndex] : 'offline';
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
                              ),
                              const SizedBox(height: 16),
                              
                              // Nếu offline, hiện cảnh báo
                              if (status.trim().toLowerCase() == 'offline')
                                Padding(
                                  padding: const EdgeInsets.all(20),
                                  child: Text(
                                    '⚠️ Không thể kết nối đến cảm biến!',
                                    style: TextStyle(color: Colors.red, fontSize: 16, fontWeight: FontWeight.bold),
                                    textAlign: TextAlign.center,
                                  ),
                                )
                              else ...[
                                // Khối Smoke + CO
                                SizedBox(
                                  height: 200,
                                  child: Row(
                                    children: [
                                      Expanded(
                                        child: Padding(
                                          padding: const EdgeInsets.all(8.0),
                                          child: buildGauge(
                                            'Smoke',
                                            smokeLevel ?? 0,
                                            0,
                                            1000,
                                            [
                                              GaugeRange(startValue: 0, endValue: 199, color: Colors.green),
                                              GaugeRange(startValue: 200, endValue: 399, color: Colors.orange),
                                              GaugeRange(startValue: 400, endValue: 1000, color: Colors.red),
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
                                            300,
                                            [
                                              GaugeRange(startValue: 0, endValue: 74, color: Colors.green),
                                              GaugeRange(startValue: 74, endValue: 149, color: Colors.orange),
                                              GaugeRange(startValue: 150, endValue: 300, color: Colors.red),
                                            ],
                                            '',
                                          ),
                                        ),
                                      ),
                                    ],
                                  ),
                                ),

                                const SizedBox(height: 8),

                                // Khối Nhiệt độ
                                Center(
                                  child: SizedBox(
                                    height: 200,
                                    width: 200,
                                    child: buildGauge(
                                      'Temperature',
                                      temperature ?? 0,
                                      0,
                                      50,
                                      [
                                        GaugeRange(startValue: 0, endValue: 34, color: Colors.green),
                                        GaugeRange(startValue: 35, endValue: 44, color: Colors.orange),
                                        GaugeRange(startValue: 45, endValue: 50, color: Colors.red),
                                      ],
                                      '',
                                    ),
                                  ),
                                ),

                                const SizedBox(height: 16),

                                _buildStatusWarning(temperature, coLevel, smokeLevel),
                              ]
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
                  style: TextStyle(fontSize: 16),
                  textAlign: TextAlign.center,
                ),
              ),
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
      if (co >= 150) {
        coStatus = 'Mức CO cao! Nguy hiểm!';
      } else if (co >= 75) {
        coStatus = 'CO ở mức trung bình';
      } else {
        coStatus = 'Mức CO an toàn';
      }
    }

    if (smoke != null) {
      if (smoke >= 400) {
        smokeStatus = 'Phát hiện khói dày!';
      } else if (smoke >= 200) {
        smokeStatus = 'Mức khói tăng cao';
      } else {
        smokeStatus = 'Mức khói ổn định';
      }
    }

    if (temperature != null) {
      if (temperature >= 45) {
        tempStatus = 'Nhiệt độ rất cao!';
      } else if (temperature >= 35) {
        tempStatus = 'Nhiệt độ khá cao';
      } else {
        tempStatus = 'Nhiệt độ ổn định';
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