// import 'package:cloud_firestore/cloud_firestore.dart';
// import 'package:fire_guard/controll/blutetoothControll.dart';
// import 'package:fire_guard/screens/accountSettingScreen.dart';
// import 'package:fire_guard/screens/mainDrawerScreen.dart';
// import 'package:fire_guard/screens/mainScreen.dart';
// import 'package:fire_guard/main.dart';
// import 'package:firebase_auth/firebase_auth.dart';
// import 'package:flutter/material.dart';
// import 'package:get/get.dart';
// import 'dart:io';
// import 'package:flutter_blue_plus/flutter_blue_plus.dart';
// import 'package:get/get_state_manager/src/simple/get_state.dart';
// import 'package:permission_handler/permission_handler.dart';


// class DeviceSettingScreen extends StatefulWidget
// {
//   const DeviceSettingScreen({super.key});

//   @override
//   State<DeviceSettingScreen> createState() {
//     // TODO: implement createState
//     return _DeviceSettingScreenState();
//   }
// }

// class _DeviceSettingScreenState extends State<DeviceSettingScreen>
// {
//   bool _isAccountActivated = false;
//   bool _isLoading = true;
//   bool _isScanning = false;

//   //List<ScanResult> _scanResults = [];

//   @override
//   void initState() {
//     super.initState();
//     _getUser();
//   }

//   Future<void> _getUser() async
//   {
//     final userId = FirebaseAuth.instance.currentUser;
    
//     if(!(userId == null))
//     {
//       try
//       {
//         final userDoc = await FirebaseFirestore.instance
//           .collection('users')
//           .doc(userId.uid)
//           .get();

//         final data = userDoc.data();
//         if(data != null && data.containsKey('activated'))
//         {
//           setState(() 
//           {
//             _isAccountActivated = data['activated'] == true;
//           });
//         }
//       } catch (e) {
//         print('Error checking activation status: $e');
//       }
//     }

//     setState(() {
//       _isLoading = false;
//     });
//   }

//   Widget _buildScanButton(BluetoothControll controller) {
//     return Obx(() => ElevatedButton(
//       onPressed: controller.isScanning.value || !_isAccountActivated
//           ? null
//           : () => _startScan(controller),
//       style: ElevatedButton.styleFrom(
//         minimumSize: const Size(double.infinity, 50),
//       ),
//       child: Text(
//         controller.isScanning.value ? 'Scanning...' : 'Start Scan',
//         style: const TextStyle(fontSize: 18),
//       ),
//     ));
//   }

//   Future<void> _startScan(BluetoothControll controller) async {
//     await _requestPermissions();
//     controller.scanDevices();
//   }

//   Future<void> _requestPermissions() async {
//     await [
//       Permission.bluetooth,
//       Permission.bluetoothScan,
//       Permission.bluetoothConnect,
//       Permission.locationWhenInUse,
//     ].request();
//   }

//   Widget _buildDeviceLists(BluetoothControll controller) {
//     return Expanded(
//       child: ListView(
//         children: [
//           _buildDeviceSection(
//             title: 'BLE Devices',
//             count: controller.bleDevices.length,
//             builder: (context) => _buildBleDevices(controller),
//           ),
//           if (Platform.isAndroid)
//             _buildDeviceSection(
//               title: 'Classic Devices',
//               count: controller.classicDevices.length,
//               builder: (context) => _buildClassicDevices(controller),
//             ),
//         ],
//       ),
//     );
//   }

//   Widget _buildDeviceSection({
//     required String title,
//     required int count,
//     required Widget Function(BuildContext) builder,
//   }) {
//     return Card(
//       child: Column(
//         crossAxisAlignment: CrossAxisAlignment.start,
//         children: [
//           Padding(
//             padding: const EdgeInsets.all(16.0),
//             child: Text(
//               '$title ($count)',
//               style: const TextStyle(
//                 fontSize: 18,
//                 fontWeight: FontWeight.bold,
//               ),
//             ),
//           ),
//           Builder(builder: builder),
//         ],
//       ),
//     );
//   }

//   Widget _buildBleDevices(BluetoothControll controller) {
//     return Obx(() => ListView.builder(
//       shrinkWrap: true,
//       physics: const NeverScrollableScrollPhysics(),
//       itemCount: controller.bleDevices.length,
//       itemBuilder: (context, index) {
//         final device = controller.bleDevices[index].device;
//         final rssi = controller.bleDevices[index].rssi;
//         return ListTile(
//           title: Text(device.name.isNotEmpty ? device.name : 'Unknown BLE'),
//           subtitle: Text(device.id.toString()),
//           trailing: Text('$rssi dBm'),
//         );
//       },
//     ));
//   }

//   Widget _buildClassicDevices(BluetoothControll controller) {
//     return Obx(() => ListView.builder(
//       shrinkWrap: true,
//       physics: const NeverScrollableScrollPhysics(),
//       itemCount: controller.classicDevices.length,
//       itemBuilder: (context, index) {
//         final device = controller.classicDevices[index];
//         return ListTile(
//           title: Text(device.name ?? 'Unknown Classic'),
//           subtitle: Text(device.address),
//         );
//       },
//     ));
//   }

//   // Future<void> _startScan() async
//   // {
//   //   setState(() {
//   //     _scanResults.clear();
//   //     _isScanning = true;
//   //   });

//   //   // Chờ người dùng bật bluetooth mới quét được
//   //   await [
//   //     Permission.bluetooth,
//   //     Permission.bluetoothScan,
//   //     Permission.bluetoothConnect,
//   //     Permission.locationWhenInUse,
//   //   ].request();

//   //   FlutterBlue.instance.startScan(timeout: const Duration(seconds: 5));

//   //   FlutterBlue.instance.scanResults.listen(
//   //     (results)
//   //     {
//   //       setState(() {
//   //         _scanResults = results;
//   //       });
//   //     }
//   //   );

//   //   await Future.delayed(const Duration(seconds: 5));
//   //   FlutterBlue.instance.stopScan();

//   //   setState(() {
//   //     _isScanning = false;
//   //   });
//   // }

//   // Widget _buildScanButton() {
//   //   if (!_isAccountActivated) return const SizedBox.shrink();

//   //   return ElevatedButton(
//   //     onPressed: _isScanning ? null : _startScan,
//   //     child: Text(_isScanning ? "Scanning..." : "Quét thiết bị Bluetooth"),
//   //   );
//   // }

//   // Widget _buildScanResults() 
//   // {
//   //   if (_scanResults.isEmpty) return const Text("Chưa có thiết bị nào.");

//   //   return Column(
//   //     children: _scanResults.map((r) 
//   //     {
//   //       return ListTile(
//   //         title: Text(r.device.name.isNotEmpty ? r.device.name : "(Không tên)"),
//   //         subtitle: Text(r.device.id.id),
//   //       );
//   //     }).toList(),
//   //   );
//   // }

//   @override
//   Widget build(BuildContext context) {
//     // TODO: implement build
//     return Scaffold(
//       appBar: AppBar(
//         title: Text('Device Settings'),
//       ),

//       drawer: MainDrawerScreen(
//         onSelectScreen: (context, identifier) {
//           Navigator.of(context).pop();
//           if(identifier == 'Account')
//           {
//             Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
//           }
//           else if(identifier == 'Main Screen')
//           {
//             Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const MainScreen()));
//           }
//       }),

//       body: _isLoading
//           ? const Center(child: CircularProgressIndicator())
//           : GetBuilder<BluetoothControll>(
//               init: BluetoothControll(),
//               builder: (controller) => Padding(
//                 padding: const EdgeInsets.all(16.0),
//                 child: Column(
//                   children: [
//                     Container(
//                       height: 180,
//                       width: double.infinity,
//                       color: Colors.blue,
//                       child: Center(
//                         child: Text(
//                           'Bluetooth Devices',
//                           style: TextStyle(
//                             color: Colors.white,
//                             fontSize: 30,
//                             fontWeight: FontWeight.bold,
//                           ),
//                         ),
//                       ),
//                     ),
//                     SizedBox(height: 20),
//                     _buildScanButton(controller),
//                     SizedBox(height: 20),
//                     _buildDeviceLists(controller),
//                   ],
//                 ),
//               ),
//             ),

//       // body: Column(
//       //   mainAxisAlignment: MainAxisAlignment.center,
//       //   children: [
//       //     Center(
//       //       child: _isLoading 
//       //       ? const CircularProgressIndicator() 
//       //       : _isAccountActivated 
//       //         ? const Text(
//       //             'Your account has been activated.\n'
//       //             'You can now use all features.',
//       //             textAlign: TextAlign.center,
//       //           )
//       //         : const Text(
//       //             'Your account has not been activated.\n'
//       //             'Please activate your account to use this feature.',
//       //             textAlign: TextAlign.center,
//       //             style: TextStyle(fontSize: 16),
//       //           )
//       //     ),
//       //   ],
//       // ),
//     );
//   }
// }

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
