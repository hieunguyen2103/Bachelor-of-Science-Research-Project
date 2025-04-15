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

class DeviceSettingScreen extends StatefulWidget {
  const DeviceSettingScreen({super.key});

  @override
  State<DeviceSettingScreen> createState() => _DeviceSettingScreenState();
}

class _DeviceSettingScreenState extends State<DeviceSettingScreen> {
  bool _isAccountActivated = false;
  bool _isLoading = true;
  List<ScanResult> _devices = [];
  Set<String> _connectedDeviceIds = {};
  bool _isScanning = false;
  StreamSubscription<List<ScanResult>>? _scanSubscription;

  // Dùng để lưu các stream connection để huỷ khi cần
  final Map<String, StreamSubscription<BluetoothConnectionState>> _connectionSubscriptions = {};

  @override
  void initState() {
    super.initState();
    _getUser();
  }

  Future<void> _getUser() async {
    final userId = FirebaseAuth.instance.currentUser;

    if (userId != null) {
      try {
        final userDoc = await FirebaseFirestore.instance.collection('users').doc(userId.uid).get();
        final data = userDoc.data();
        if (data != null && data.containsKey('activated')) {
          setState(() {
            _isAccountActivated = data['activated'] == true;
          });
        }
      } catch (e) {
        print('Error checking activation status: $e');
      }
    }

    setState(() {
      _isLoading = false;
    });
  }

  Future<void> _checkPermissions() async {
    await [
      Permission.bluetooth,
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ].request();
  }

  Future<void> _checkBluetooth() async {
    if (!await FlutterBluePlus.isAvailable) {
      throw Exception("Bluetooth không khả dụng");
    }

    if (!await FlutterBluePlus.isOn) {
      await FlutterBluePlus.turnOn();
    }
  }

  void _startScan() async {
    try {
      await _checkPermissions();
      await _checkBluetooth();

      setState(() {
        _isScanning = true;
        _devices.clear();
      });

      await FlutterBluePlus.startScan(timeout: const Duration(seconds: 30));

      _scanSubscription = FlutterBluePlus.scanResults.listen((results) {
        for (var result in results) {
          if (!_devices.any((d) => d.device.remoteId == result.device.remoteId)) {
            setState(() {
              _devices.add(result);
            });
          }
        }
      });
    } catch (e) {
      print('Error: $e');
      _stopScan();
    }
  }

  void _stopScan() async {
    await FlutterBluePlus.stopScan();
    await _scanSubscription?.cancel();
    setState(() {
      _isScanning = false;
      _devices.clear();
    });
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    final deviceId = device.remoteId.str;
    try {
      await device.connect(timeout: const Duration(seconds: 15));

      final isConnected = await device.isConnected;
      if (isConnected) {
        setState(() {
          _connectedDeviceIds.add(deviceId);
        });

        // Hủy subscription cũ nếu có
        await _connectionSubscriptions[deviceId]?.cancel();

        // Lắng nghe trạng thái kết nối
        _connectionSubscriptions[deviceId] = device.connectionState.listen((state) {
          setState(() {
            if (state == BluetoothConnectionState.connected) {
              _connectedDeviceIds.add(deviceId);
            } else {
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
  void dispose() {
    _stopScan();
    for (var sub in _connectionSubscriptions.values) {
      sub.cancel();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Device Settings'),
      ),
      drawer: MainDrawerScreen(onSelectScreen: (context, identifier) {
        Navigator.of(context).pop();
        if (identifier == 'Account') {
          Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
        } else if (identifier == 'Main Screen') {
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
                      onChanged: (bool value) {
                        if (value) {
                          _startScan();
                        } else {
                          _stopScan();
                        }
                      },
                    ),
                    Expanded(
                      child: ListView.builder(
                        itemCount: _devices.length,
                        itemBuilder: (context, index) {
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
