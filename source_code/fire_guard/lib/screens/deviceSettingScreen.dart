import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:fire_guard/screens/mainDrawerScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:fire_guard/main.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class DeviceSettingScreen extends StatefulWidget
{
  const DeviceSettingScreen({super.key});

  @override
  State<DeviceSettingScreen> createState() {
    // TODO: implement createState
    return _DeviceSettingScreenState();
  }
}

class _DeviceSettingScreenState extends State<DeviceSettingScreen>
{
  bool _isAccountActivated = false;
  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _getUser();
  }

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

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Scaffold(
      appBar: AppBar(
        title: Text('Device Settings'),
      ),

      drawer: MainDrawerScreen(
        onSelectScreen: (context, identifier) {
          Navigator.of(context).pop();
          if(identifier == 'Account')
          {
            Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const AccountSettingScreen()));
          }
          else if(identifier == 'Main Screen')
          {
            Navigator.of(context).push(MaterialPageRoute(builder: (ctx) => const MainScreen()));
          }
      }),

      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Center(
            child: _isLoading 
            ? const CircularProgressIndicator() 
            : _isAccountActivated 
              ? const Text(
                  'Your account has been activated.\n'
                  'You can now use all features.',
                  textAlign: TextAlign.center,
                )
              : const Text(
                  'Your account has not been activated.\n'
                  'Please activate your account to use this feature.',
                  textAlign: TextAlign.center,
                  style: TextStyle(fontSize: 16),
                )
          ),
        ],
      ),
    );
  }
}