import 'package:fire_guard/screens/DeviceSettingScreen.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/widget/notifycationIconButton.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class MainScreen extends StatelessWidget
{
  const MainScreen({super.key});

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
      body: const Center(
        child: Text('Fire Guard'),
      ),
    );
  }
}