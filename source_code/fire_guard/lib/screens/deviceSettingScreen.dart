import 'dart:async';
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:permission_handler/permission_handler.dart';

class DeviceSettingScreen extends StatefulWidget // Có nhiều trạng thái thay đổi nên dùng StatefulWidget
{
  const DeviceSettingScreen({super.key});

  @override
  State<StatefulWidget> createState() {
    // TODO: implement createState
    return _DeviceSettingScreenState();
  }
}

class _DeviceSettingScreenState extends State<DeviceSettingScreen> 
{
  bool _isAccountActivated = false;  // Ban đầu để trạng thái tài khoản là chưa được kích hoạt, nhưng nên đặt là true
  bool _isLoading = true;  // Trạng thái load dữ liệu, đặt là true hay false đều được
  List<ScanResult> _devices = [];  // Danh sách thiết bị quét được
  Set<String> _connectedDeviceIds = {};  // Danh sách thiết bị đã kết nối
  bool _isScanning = false;  // Trạng thái có đang quét hay không, nên đặt là false, true cũng chả vấn đề gì
  StreamSubscription<List<ScanResult>>? _scanSubscription;  // Cái này để lắng nghe kết quả quét
  String? _userUUID;
  StreamSubscription<List<int>>? _userNotifySub;
  bool _isWaitingWifi = false;


  // Dùng để theo dõi trạng thái kết nối của từng thiết bị, nếu thiết bị kia hủy kết nối thì trên app cũng phải hiển thị ngắt connect luôn
  final Map<String, StreamSubscription<BluetoothConnectionState>> _connectionSubscriptions = {};

  @override
  void initState()  // Khởi tạo trạng thái
  {
    super.initState();
    _getUser();  // Gọi hàm này để kiểm tra trạng thái tài khoản
  }

  Future<void> _getUser() async {
    final userId = FirebaseAuth.instance.currentUser;  // Lấy người dùng hiện tại từ Firebase Authentication, nếu người dùng chưa đăng nhập thì cái này sẽ trả về null

    if (userId != null) // Check xem có null hay không
    {
      try {
        final userDoc = await FirebaseFirestore.instance  // Lấy tài liệu tương ứng với người dùng có id như kia trong collection users
          .collection('users')
          .doc(userId.uid)
          .get();  // Cái này sẽ trả về 1 DocumentSnapshot

        final data = userDoc.data();  // Lấy ra dữ liệu người dùng vưa đọc được, nhưng mà dưới dạng Map<String, dynamic>

        if (data != null && data.containsKey('activated'))  // Nếu như có data và có field 'activated'
        {
           _userUUID = userId.uid;
          setState(()   // Thì set trạng thái tài khoản
          {
            _isAccountActivated = (data['activated'] == true);
          });
        }
      } catch (e) {  // Cái này để bắt lỗi
        print('Error checking activation status: $e');
      }
    }

    setState(()  // Cập nhất biến để kết thúc quá trình load, chả lẽ cứ quay tròn mãi 
    {
      _isLoading = false;
    });
  }

  Future<void> _checkPermissions() async  // Cái này để kiểm tra và yêu cầu quyền truy cập bluetooth để app có thể scan và connect
  {
    await [  // phải đợi đến khi các quyền này được yêu cầu xong mới tiếp tục
      Permission.bluetooth,  // Yêu cầu sử dụng bluetotoh
      Permission.bluetoothScan,   // Dùng để quét bluetooth xung quanh
      Permission.bluetoothConnect,  // Dùng để connect với thiết bị bluetooth
      Permission.locationWhenInUse,  // Trên android, để quét thiết bị BLE thì nó cần quyền truy cập vị trí
    ].request();
  }

  Future<void> _checkBluetooth() async // Cái này để kiểm tra xem Bluetooth của thiết bị có khả dụng hay không
  {
    if (!await FlutterBluePlus.isAvailable) // Kiểm tra xem có Bluetooth không
    {
      throw Exception("Bluetooth không khả dụng");
    }

    if (!await FlutterBluePlus.isOn) // Nếu có thì bật tình yêu lên
    {
      await FlutterBluePlus.turnOn();
    }
  }

  void _startScan() async // Bắt đầu quét
  {
    try 
    { 
      await _checkPermissions(); // Trước khi quét phải yêu cầu quyền
      await _checkBluetooth();  // Và kiểm tra xem bluetooth có sẵn để dùng không

      setState(() // Đặt lại các biến trạng thái
      {
        _isScanning = true;  
        _devices.clear();  // Xóa hết những thiết bị đã quét từ lần trước đi (nếu có)
      });

      await FlutterBluePlus.startScan(timeout: const Duration(seconds: 30)); // Đặt thời gian tự động ngừng quét là 30 giây

      _scanSubscription = FlutterBluePlus.scanResults.listen((results)  // Lắng nghe các thiết bị quét được rồi thêm nó vào _scanSubscription
      {
        for (var result in results) // Cái này là vòng lặp for để duyệt hết mảng các thiết bị nó quét được
        {
          if (!_devices.any((d) => d.device.remoteId == result.device.remoteId)) // Kiểm tra xem thiết bị này có trong danh sách hay chưa, chưa có thì thêm vào
          {
            setState(() 
            {
              _devices.add(result);  // Thêm cái kết quả quét được vào
            });
          }
        }
      });
    } catch (e) {  // In ra lỗi
      print('Error: $e');
      _stopScan();  // Lỗi thì dừng quét
    }
  }

  Future<void> _stopScan() async {
    setState(() {
      _isScanning = false;
    });

    await FlutterBluePlus.stopScan();
    await _scanSubscription?.cancel();

    // Ngắt kết nối với tất cả thiết bị đang connect
    final connectedDevices = await FlutterBluePlus.connectedDevices;
    for (var device in connectedDevices) {
      if (_connectedDeviceIds.contains(device.remoteId.str)) {
        await device.disconnect();
      }
    }

    // Hủy lắng nghe trạng thái kết nối
    for (var sub in _connectionSubscriptions.values) {
      await sub.cancel();
    }

    setState(() {
      _devices.clear();
      _connectedDeviceIds.clear();
      _connectionSubscriptions.clear();
    });
  }


  Future<void> _connectToDevice(BluetoothDevice device) async 
  {
    final deviceId = device.remoteId.str;
    try 
    {
      await device.connect(timeout: const Duration(seconds: 15));

      final isConnected = await device.isConnected;
      if (isConnected) 
      {
        setState(() 
        {
          _connectedDeviceIds.add(deviceId);
        });

        // Hủy subscription cũ nếu có
        await _connectionSubscriptions[deviceId]?.cancel();

        // Lắng nghe trạng thái kết nối
        _connectionSubscriptions[deviceId] = device.connectionState.listen((state) 
        {
          setState(() 
          {
            if (state == BluetoothConnectionState.connected) 
            {
              _connectedDeviceIds.add(deviceId);
            } else 
            {
              _connectedDeviceIds.remove(deviceId);
            }
          });
        });

        _showWifiDialog(device);

        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('Connected ${device.name.isEmpty ? 'No name' : device.name}'),
            backgroundColor: Colors.green,
          ),
        );
      }
    } catch (e) {
      print('Connect Error: $e');
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Can\'t connect: $e')),
      );
    }
  }

  @override
  void dispose() 
  {
    _stopScan();
    for (var sub in _connectionSubscriptions.values) 
    {
      sub.cancel();
    }
    _userNotifySub?.cancel();
    super.dispose();
  }

  // Hàm gửi ssid và password
  Future<void> _sendWifi(BluetoothDevice device, String ssid, String password, String userID, String nameDevice) async {
    print('Gửi ssid và password');

    final serviceUuid = Guid("000000ff-0000-1000-8000-00805f9b34fb");
    final ssidUuid    = Guid("0000ff01-0000-1000-8000-00805f9b34fb");
    
    final passServiceUuid = Guid("000000ee-0000-1000-8000-00805f9b34fb"); // 0x00EE
    final passUuid    = Guid("0000ee01-0000-1000-8000-00805f9b34fb"); // 0xEE01

    final userServiceUuid = Guid("000000dd-0000-1000-8000-00805f9b34fb"); // 0x00EE
    final userUuid    = Guid("0000dd01-0000-1000-8000-00805f9b34fb"); // 0xEE01
    
    // final nameDeviceServiceUuid = Guid("000000aa-0000-1000-8000-00805f9b34fb"); // 0x00AA
    // final nameDeviceUuid    = Guid("0000aa01-0000-1000-8000-00805f9b34fb"); // 0xAA01

    try {
      await device.discoverServices();
      // final services = await device.services.first;
      final services = await device.discoverServices(); 

      // print("Các service tìm thấy:");
      // for (var s in services) {
      //   print("- Service UUID: ${s.uuid}");
      //   for (var c in s.characteristics) {
      //     print("  - Characteristic UUID: ${c.uuid}");
      //   }
      // }

      final ssidService = services.firstWhere(
        (s) => s.uuid == serviceUuid,
        orElse: () => throw Exception("Không tìm thấy service SSID"),
      );
      final passService = services.firstWhere(
        (s) => s.uuid == passServiceUuid,
        orElse: () => throw Exception("Không tìm thấy service PASS"),
      );
      final userService = services.firstWhere(
        (s) => s.uuid == userServiceUuid,
        orElse: () => throw Exception("Không tìm thấy service USER"),
      );
      // final nameDeviceService = services.firstWhere(
      //   (s) => s.uuid == nameDeviceServiceUuid,
      //   orElse: () => throw Exception("Không tìm thấy service NAME"),
      // );

      final ssidChar = ssidService.characteristics.firstWhere(
        (c) => c.uuid == ssidUuid,
        orElse: () => throw Exception("Không tìm thấy characteristic SSID"),
      );
      final passChar = passService.characteristics.firstWhere(
        (c) => c.uuid == passUuid,
        orElse: () => throw Exception("Không tìm thấy characteristic PASS"),
      );
      final userChar = userService.characteristics.firstWhere(
        (c) => c.uuid == userUuid,
        orElse: () => throw Exception("Không tìm thấy characteristic USER"),
      );
      // final nameChar = nameDeviceService.characteristics.firstWhere(
      //   (c) => c.uuid == nameDeviceUuid,
      //   orElse: () => throw Exception("Không tìm thấy characteristic NAME"),
      // );

      // Hủy stream cũ nếu có
      await _userNotifySub?.cancel();
      // Bật Notify và listen thông báo từ ESP32
      await userChar.setNotifyValue(true);
      _userNotifySub = userChar.lastValueStream.listen((data) {
        final message = String.fromCharCodes(data);
        if (message == "Wifi Connected") {
          setState(() {
            _isWaitingWifi = false;
          });
          print("ESP32 đã kết nối Wi-Fi thành công!");
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text("ESP32 đã kết nối Wi-Fi thành công"),
              backgroundColor: Colors.green,
              duration: Duration(seconds: 2),
            ),
          );
        } 
        else if(message == "Wifi Failed")
        {
          setState(() {
            _isWaitingWifi = false;
          });
          ScaffoldMessenger.of(context).showSnackBar(
            const SnackBar(
              content: Text("ESP32 kết nối Wifi thất bại"),
              backgroundColor: Colors.black,
              duration: Duration(seconds: 2),
            ),
          );
        }
        else 
        {
          print("Nhận được BLE notify: $message");
        }
      });

      setState(() {
        _isWaitingWifi = true;
      });


      await ssidChar.write(ssid.codeUnits, withoutResponse: false);
      await passChar.write(password.codeUnits, withoutResponse: false);
      await userChar.write(userID.codeUnits, withoutResponse: false);
      // await nameChar.write(nameDevice.codeUnits, withoutResponse: false);
      
      print("Đã gửi SSID và PASS thành công.");
    } catch (e) {
      print("Lỗi khi gửi dữ liệu BLE: $e");
    }
  }


  // Hiển thị cửa sổ để nhập ssid và password
  Future<void> _showWifiDialog(BluetoothDevice device) async
  {
    final ssidController = TextEditingController();
    final passwordController = TextEditingController();
    final nameDeviceController = TextEditingController();

    await showDialog(
      context: context, 
      builder: (ctx) => AlertDialog(
        title: Text('Setup Wifi'),
        content: SingleChildScrollView(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              TextField(
                controller: ssidController,
                decoration: const InputDecoration(labelText: 'SSID'),
              ),
              TextField(
                controller: passwordController,
                decoration: const InputDecoration(labelText: 'Password'),
                obscureText: true,
              ),
              // TextField(
              //   controller: nameDeviceController,
              //   decoration: const InputDecoration(labelText: 'Name Device'),
              // ),
            ],
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.of(ctx).pop(), 
            child: const Text('Cancel'),
          ),
          TextButton(
            onPressed: () {
              final ssid = ssidController.text.trim();
              final password = passwordController.text.trim();
              final nameDevice = nameDeviceController.text.trim();
              if(ssid.isNotEmpty && password.isNotEmpty && _userUUID != null)
              {
                setState(() {
                  _isWaitingWifi = true;
                });

                Navigator.of(ctx).pop();

                Future.microtask(() {
                  _sendWifi(device, ssid, password, _userUUID!, nameDevice);
                });
              }
            }, 
            child: const Text('OK'),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) 
  {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Device Settings'),
      ),
      drawer: MainDrawerScreen(onSelectScreen: (context, identifier) 
      {
        Navigator.of(context).pop();
        if (identifier == 'Account') 
        {
          Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
        } 
        else if (identifier == 'Main Screen') 
        {
          Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const MainScreen()));
        }
      }),
      body: _isLoading
          ? const Center(child: CircularProgressIndicator())
          : _isAccountActivated
              ? Stack(
                  children: [
                    Column(
                      children: [
                        SwitchListTile(
                          title: const Text('Quét các thiết bị ở gần'),
                          value: _isScanning,
                          onChanged: (bool value) 
                          {
                            if (value) 
                            {
                              _startScan();
                            } 
                            else 
                            {
                              _stopScan();
                            }
                          },
                        ),
                        Expanded(
                          child: ListView.builder(
                            itemCount: _devices.length,
                            itemBuilder: (context, index) 
                            {
                              final device = _devices[index];
                              final deviceId = device.device.remoteId.str;
                              final isConnected = _connectedDeviceIds.contains(deviceId);

                              return Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  ListTile(
                                    title: Text(device.device.name.isEmpty ? "Thiết bị không tên" : device.device.name),
                                    subtitle: Text(deviceId),
                                    onTap: () => _connectToDevice(device.device),
                                  ),
                                  if (isConnected)
                                    Padding(
                                      padding: const EdgeInsets.only(left: 16.0, bottom: 8),
                                      child: Row(
                                        children: const [
                                          Icon(Icons.circle, color: Colors.green, size: 12),
                                          SizedBox(width: 6),
                                          Text('Connected', style: TextStyle(color: Colors.green)),
                                        ],
                                      ),
                                    ),
                                ],
                              );
                            },
                          ),
                        ),
                      ],
                    ),
                    if(_isWaitingWifi)
                      Container(
                        color: Colors.black.withOpacity(0.5),
                        child: const Center(child: CircularProgressIndicator(),),
                      ),
                  ],
                )
              : const Center(
                  child: Text(
                    'Your account has not been activated.\nPlease activate your account to use this feature.',
                    textAlign: TextAlign.center,
                    style: TextStyle(fontSize: 16),
                  ),
                ),
    );
  }
}
