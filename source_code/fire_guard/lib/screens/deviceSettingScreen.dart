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

  void _stopScan() async 
  {
    await FlutterBluePlus.stopScan();  // Dĩ nhiên cái này để dừng scan
    await _scanSubscription?.cancel();  // Nhưng người dừng scan còn phải hủy lắng nghe scanResults, có scan nữa đầu mà nghe
    setState(() 
    {
      _isScanning = false; // Đặt lại trạng thái
      _devices.clear();  // Xóa danh sách thiết bị đã quét được đi
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
    super.dispose();
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
              ? Column(
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
