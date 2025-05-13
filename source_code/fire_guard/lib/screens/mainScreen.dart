// import 'package:cloud_firestore/cloud_firestore.dart';
// import 'package:fire_guard/screens/DeviceSettingScreen.dart';
// import 'package:fire_guard/screens/accountSettingScreen.dart';
// import 'package:fire_guard/screens/mainDrawerScreen.dart';
// import 'package:fire_guard/widget/notifycationIconButton.dart';
// import 'package:firebase_auth/firebase_auth.dart';
// import 'package:flutter/material.dart';

// class MainScreen extends StatefulWidget
// {
//   const MainScreen({super.key});

//   @override
//   State<MainScreen> createState() {
//     // TODO: implement createState
//     return _MainSCreenState();
//   }
// }

// class _MainSCreenState extends State<MainScreen>
// {
//   bool _isAccountActivated = false;
//   bool _isLoading = true;

//   // Số liệu hiển thị
//   double? coLevel;
//   double? smokeLevel;
//   int? temperature;

//   @override
//   void initState() {
//     super.initState();
//     _loadUserAndData();
//   }

//   Future<void> _loadUserAndData() async {
//     final user = FirebaseAuth.instance.currentUser;
//     if (user == null) return;

//     try {
//       final userDoc = await FirebaseFirestore.instance
//           .collection('users')
//           .doc(user.uid)
//           .get();

//       final data = userDoc.data();
//       if (data != null && data['activated'] == true) {
//         setState(() {
//           _isAccountActivated = true;
//         });

//         // Lấy dữ liệu cảm biến nếu đã kích hoạt
//         final sensorDoc = await FirebaseFirestore.instance
//             .collection('users')
//             .doc(user.uid)
//             .collection('data_sensor')
//             .doc('00000000') // Document ID bạn dùng
//             .get();

//         final sensorData = sensorDoc.data();
//         if (sensorData != null) {
//           setState(() {
//             coLevel = (sensorData['Co_level'] as num).toDouble();
//             smokeLevel = (sensorData['Smoke_level'] as num).toDouble();
//             temperature = (sensorData['Temp'] as num).toInt();
//           });
//         }
//       }
//     } catch (e) {
//       print('Error loading user or sensor data: $e');
//     }

//     setState(() {
//       _isLoading = false;
//     });
//   }

//   // Future<void> _getUserActivationStatus() async {
//   //   final userId = FirebaseAuth.instance.currentUser;
//   //   if (userId != null) {
//   //     try {
//   //       final userDoc = await FirebaseFirestore.instance.collection('users').doc(userId.uid).get();
//   //       final data = userDoc.data();
//   //       if (data != null && data.containsKey('activated')) {
//   //         setState(() {
//   //           _isAccountActivated = data['activated'] == true;
//   //           _isLoading = false;  // Dữ liệu đã được tải xong
//   //         });
//   //       }
//   //     } catch (e) {
//   //       print('Error checking activation status: $e');
//   //       setState(() {
//   //         _isLoading = false;  // Nếu có lỗi thì cũng dừng việc tải dữ liệu
//   //       });
//   //     }
//   //   }
//   // }

//   void _setScreen(BuildContext context, String identifier)
//   {
//     Navigator.of(context).pop();
//     if(identifier == 'Account')
//     {
//       Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
//     } 
//     else if (identifier == 'Device') 
//     {
//       Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const DeviceSettingScreen()));
//     }
//   }

//   @override
//   Widget build(BuildContext context) {
//     // TODO: implement build
//     return Scaffold(
//       appBar: AppBar(
//         title: const Text('Home page'),
//         actions: const [
//           NotificationIconButton(),
//           // IconButton(
//           //   onPressed: () async {
//           //     await FirebaseAuth.instance.signOut();  // Gọi phương thức đăng xuất
//           //   }, 
//           //   icon: Icon(
//           //     Icons.exit_to_app,
//           //     color: Theme.of(context).colorScheme.primary,
//           //   ),
//           // )
//         ],
//       ),
//       drawer: MainDrawerScreen(onSelectScreen: _setScreen,),  // Hiển thị cái slide để chọn các trang muốn xem
//       body: Center(
//         child: _isLoading
//           ? const CircularProgressIndicator()
//           : _isAccountActivated
//             ? Column(
//                 mainAxisAlignment: MainAxisAlignment.center,
//                 crossAxisAlignment: CrossAxisAlignment.center,
//                 children: [
//                   Text('Fire Guard', style: Theme.of(context).textTheme.headlineMedium,),
//                   const SizedBox(height: 20,),
//                   Text('CO level: ${coLevel ?? '-'}'),
//                   Text('Smoke Level: ${smokeLevel ?? '-'}'),
//                   Text('Temperature: ${temperature ?? '-'} °C'),
//                 ],
//               )
//             : const Text(
//               'Please activate your account to use features',
//               style: TextStyle(color: Colors.red),
//               ),
//       ),
//         // : Center(
//         //     child: _isAccountActivated
//         //       ? const Text('Fire Guard', style: TextStyle(fontSize: 24),)
//         //       : const Text(
//         //         'Please activate your account to use this feature',
//         //         style: TextStyle(fontSize: 16, color: Colors.red),
//         //         textAlign: TextAlign.center,
//         //       ),
//         //   ),
//     );
//   }
// }


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
  int? temperature;

  @override
  void initState() {
    super.initState();
    _loadUserAndData();
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

        final sensorDoc = await FirebaseFirestore.instance
            .collection('users')
            .doc(user.uid)
            .collection('data_sensor')
            .doc('00000000')
            .get();

        final sensorData = sensorDoc.data();
        if (sensorData != null) {
          setState(() {
            coLevel = (sensorData['Co_level'] as num).toDouble();
            smokeLevel = (sensorData['Smoke_level'] as num).toDouble();
            temperature = (sensorData['Temp'] as num).toInt();
          });
        }
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
          ? Padding(
              padding: const EdgeInsets.all(8.0),
              child: Column(
                children: [
                  Text('Fire Guard',
                    style: Theme.of(context).textTheme.headlineMedium,
                  ),
                  const SizedBox(height: 10),
                  Expanded(

                    // child: Row(
                    //   mainAxisAlignment: MainAxisAlignment.spaceAround,
                    //   children: [
                    //     SemiCircleGauge(
                    //       label: 'Nhiệt độ',
                    //       value: temperature?.toDouble() ?? 0,
                    //       unit: '°C',
                    //       min: 0,
                    //       max: 100,
                    //       colors: const [Colors.green, Colors.orange, Colors.red],
                    //     ),
                    //     SemiCircleGauge(
                    //       label: 'Khói',
                    //       value: smokeLevel ?? 0,
                    //       unit: '',
                    //       min: 0,
                    //       max: 100,
                    //       colors: const [Colors.green, Colors.yellow, Colors.red],
                    //     ),
                    //     SemiCircleGauge(
                    //       label: 'CO',
                    //       value: coLevel ?? 0,
                    //       unit: '',
                    //       min: 0,
                    //       max: 100,
                    //       colors: const [Colors.green, Colors.orange, Colors.red],
                    //     ),
                    //   ],
                    // ),

                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceAround,
                      children: [
                        Expanded(child: buildGauge('Temperature', temperature?.toDouble() ?? 0, 0, 100, [
                          GaugeRange(startValue: 0, endValue: 35, color: Colors.green),
                          GaugeRange(startValue: 36, endValue: 50, color: Colors.orange),
                          GaugeRange(startValue: 51, endValue: 100, color: Colors.red),
                        ], '°C')),

                        Expanded(child: buildGauge('Smoke', smokeLevel ?? 0, 0, 100, [
                          GaugeRange(startValue: 0, endValue: 20, color: Colors.green),
                          GaugeRange(startValue: 21, endValue: 60, color: Colors.orange),
                          GaugeRange(startValue: 61, endValue: 100, color: Colors.red),
                        ], '')),

                        Expanded(child: buildGauge('CO', coLevel ?? 0, 0, 100, [
                          GaugeRange(startValue: 0, endValue: 10, color: Colors.green),
                          GaugeRange(startValue: 11, endValue: 35, color: Colors.orange),
                          GaugeRange(startValue: 36, endValue: 100, color: Colors.red),
                        ], '')),
                      ],
                    ),
                  ),
                ],
              ),
            )
          : const Center(
              child: Text(
                'Please activate your account to use features',
                style: TextStyle(color: Colors.red),
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
        textStyle: const TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
      ),
      axes: <RadialAxis>[
        RadialAxis(
          minimum: min,
          maximum: max,
          ranges: ranges,
          pointers: <GaugePointer>[
            NeedlePointer(value: value),
          ],
          annotations: <GaugeAnnotation>[
            GaugeAnnotation(
              widget: Text(
                '${value.toStringAsFixed(1)} $unit',
                style: const TextStyle(fontSize: 14, fontWeight: FontWeight.bold),
              ),
              angle: 90,
              positionFactor: 0.75,
            ),
          ],
        ),
      ],
    );
  }

}
