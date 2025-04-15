// import 'package:flutter_blue_plus/flutter_blue_plus.dart' as ble;
// import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart' as serial;
// import 'package:get/get.dart';
// import 'package:permission_handler/permission_handler.dart';

// class BluetoothControll extends GetxController
// {
//   //Cho BLE
//   final ble.FlutterBluePlus bleScanner = ble.FlutterBluePlus();
//   final RxList<ble.ScanResult> bleDevices = <ble.ScanResult>[].obs;

//   // Cho Bluetooth Classic
//   final serial.FlutterBluetoothSerial btSerial = serial.FlutterBluetoothSerial.instance;
//   final RxList<serial.BluetoothDevice> classicDevices = <serial.BluetoothDevice>[].obs;
//   final RxBool isScanning = false.obs;

//   Future<void> checkPermissions() async {
//     await [
//       Permission.bluetooth,
//       Permission.bluetoothScan,
//       Permission.bluetoothConnect,
//       Permission.locationWhenInUse,
//     ].request();
//   }

//   Future<void> _startClassicScan() async {
//     await btSerial.requestEnable();
//     btSerial.startDiscovery().listen((serial.BluetoothDiscoveryResult result) {
//       if (!classicDevices.any((d) => d.address == result.device.address)) {
//         classicDevices.add(result.device);
//       }
//     });
//   }

//    void stopScan() {
//     ble.FlutterBluePlus.stopScan();
//     if (GetPlatform.isAndroid) {
//       btSerial.cancelDiscovery();
//     }
//     isScanning.value = false;
//   }

//   @override
//   void onClose() {
//     stopScan();
//     super.onClose();
//   }

//   Future<void> scanDevices() async
//   {
//     if(isScanning.value) return;

//     isScanning.value = true;
//     bleDevices.clear();
//     classicDevices.clear();

//     try {
//       // Start BLE Scan
//       ble.FlutterBluePlus.startScan(
//         timeout: const Duration(seconds: 10),
//         androidUsesFineLocation: false,
//       );

//       ble.FlutterBluePlus.scanResults.listen((results) {
//         bleDevices.assignAll(results);
//       });

//       // Start Classic Scan (Android only)
//       if (GetPlatform.isAndroid) {
//         await _startClassicScan();
//       }

//       await Future.delayed(const Duration(seconds: 10));
//     } finally {
//       stopScan();
//     }
//   }
// }