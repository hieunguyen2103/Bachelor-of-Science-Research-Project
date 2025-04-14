import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/main.dart';
import 'package:fire_guard/screens/DeviceSettingScreen.dart';
import 'package:fire_guard/screens/activateAccountScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class AccountSettingScreen extends StatefulWidget
{
  const AccountSettingScreen({super.key});

  @override
  State<AccountSettingScreen> createState() {
    // TODO: implement createState
    return _AccountSettingScreenState();
  }
}

class _AccountSettingScreenState extends State<AccountSettingScreen>
{
  bool _isAccountActivated = false;
  bool _isLoading = true;

  Future<void> _getUser() async
  {
    final userId = FirebaseAuth.instance.currentUser;
    
    if(!(userId == null))
    {
      try
      {
        final userDoc = await FirebaseFirestore.instance
          .collection('users')
          .doc(userId.uid)
          .get();

        final data = userDoc.data();
        if(data != null && data.containsKey('activated'))
        {
          setState(() 
          {
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

  void initState() 
  {
    super.initState();
    _getUser();
  }

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: Text('Account Manage'),
      ),
      drawer: MainDrawerScreen(
        onSelectScreen: (context, identifier) {
          Navigator.of(context).pop();
          if(identifier == 'Device')
          {
            Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const DeviceSettingScreen()));
          }
          if(identifier == 'Main Screen')
          {
            Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const MainScreen()));
          }
      }),
      body: Center(
        child: _isLoading ? CircularProgressIndicator() : _isAccountActivated ? Text('Your account is already activated')
        : Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Text(
              'No account information found!\nActivate now?', 
              style: TextStyle(fontSize: 16), 
              textAlign: TextAlign.center,
            ),
            SizedBox(height: 16,),
            ElevatedButton(
              style: ElevatedButton.styleFrom(  // Tạo màu nền cho cái nút nổi này
                backgroundColor: Theme.of(context).colorScheme.primaryContainer
              ),
              onPressed: () {
                Navigator.of(context).push(
                  MaterialPageRoute(builder: (ctx) => const ActivateAccountScreen()),
                );
              }, 
              child: Text('Activate'),
            )
          ],
        ),
      ),
    );
  }
}