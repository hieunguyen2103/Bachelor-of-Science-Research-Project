import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/screens/DeviceSettingScreen.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/widget/notifycationIconButton.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class MainScreen extends StatefulWidget
{
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() {
    // TODO: implement createState
    return _MainSCreenState();
  }
}

class _MainSCreenState extends State<MainScreen>
{
  bool _isAccountActivated = false;
  bool _isLoading = true;

  // Số liệu hiển thị
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

        // Lấy dữ liệu cảm biến nếu đã kích hoạt
        final sensorDoc = await FirebaseFirestore.instance
            .collection('users')
            .doc(user.uid)
            .collection('data_sensor')
            .doc('00000000') // Document ID bạn dùng
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

  // Future<void> _getUserActivationStatus() async {
  //   final userId = FirebaseAuth.instance.currentUser;
  //   if (userId != null) {
  //     try {
  //       final userDoc = await FirebaseFirestore.instance.collection('users').doc(userId.uid).get();
  //       final data = userDoc.data();
  //       if (data != null && data.containsKey('activated')) {
  //         setState(() {
  //           _isAccountActivated = data['activated'] == true;
  //           _isLoading = false;  // Dữ liệu đã được tải xong
  //         });
  //       }
  //     } catch (e) {
  //       print('Error checking activation status: $e');
  //       setState(() {
  //         _isLoading = false;  // Nếu có lỗi thì cũng dừng việc tải dữ liệu
  //       });
  //     }
  //   }
  // }

  void _setScreen(BuildContext context, String identifier)
  {
    Navigator.of(context).pop();
    if(identifier == 'Account')
    {
      Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
    } 
    else if (identifier == 'Device') 
    {
      Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const DeviceSettingScreen()));
    }
  }

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: const Text('Home page'),
        actions: const [
          NotificationIconButton(),
          // IconButton(
          //   onPressed: () async {
          //     await FirebaseAuth.instance.signOut();  // Gọi phương thức đăng xuất
          //   }, 
          //   icon: Icon(
          //     Icons.exit_to_app,
          //     color: Theme.of(context).colorScheme.primary,
          //   ),
          // )
        ],
      ),
      drawer: MainDrawerScreen(onSelectScreen: _setScreen,),  // Hiển thị cái slide để chọn các trang muốn xem
      body: Center(
        child: _isLoading
          ? const CircularProgressIndicator()
          : _isAccountActivated
            ? Column(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Text('Fire Guard', style: Theme.of(context).textTheme.headlineMedium,),
                  const SizedBox(height: 20,),
                  Text('CO level: ${coLevel ?? '-'}'),
                  Text('Smoke Level: ${smokeLevel ?? '-'}'),
                  Text('Temperature: ${temperature ?? '-'} °C'),
                ],
              )
            : const Text(
              'Please activate your account to use features',
              style: TextStyle(color: Colors.red),
              ),
      ),
        // : Center(
        //     child: _isAccountActivated
        //       ? const Text('Fire Guard', style: TextStyle(fontSize: 24),)
        //       : const Text(
        //         'Please activate your account to use this feature',
        //         style: TextStyle(fontSize: 16, color: Colors.red),
        //         textAlign: TextAlign.center,
        //       ),
        //   ),
    );
  }
}