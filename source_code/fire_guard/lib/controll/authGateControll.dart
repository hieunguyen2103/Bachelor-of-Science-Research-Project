// import 'package:fire_guard/screens/authScreen.dart';
// import 'package:fire_guard/screens/mainScreen.dart';
// import 'package:fire_guard/screens/splashScreen.dart';
// import 'package:firebase_auth/firebase_auth.dart';
// import 'package:flutter/widgets.dart';

// class AuthGateControll extends StatelessWidget {
//   const AuthGateControll({super.key});

//   @override
//   Widget build(BuildContext context) {
//     return StreamBuilder<User?>(
//       stream: FirebaseAuth.instance.authStateChanges(),
//       builder: (ctx, snapshot) {
//         if (snapshot.connectionState == ConnectionState.waiting) {
//           return const SplashScreen();
//         }

//         if (snapshot.hasData) {
//           return const MainScreen();  
//         }

//         return const AuthScreen();
//       },
//     );
//   }
// }

import 'package:fire_guard/screens/authScreen.dart';
import 'package:fire_guard/screens/mainScreen.dart';
import 'package:fire_guard/screens/splashScreen.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class AuthGateControll extends StatefulWidget 
{
  const AuthGateControll({super.key});

  @override
  State<AuthGateControll> createState() => _AuthGateControllState();
}

class _AuthGateControllState extends State<AuthGateControll> 
{
  bool _isUserValid = true;

  @override
  void initState()
  {
    super.initState();
    _validateUser();
  }

  Future<void> _validateUser() async 
  {
    final user = FirebaseAuth.instance.currentUser;

    if (user != null) 
    {
      try 
      {
        await user.reload();
        if (FirebaseAuth.instance.currentUser == null) 
        {
          await FirebaseAuth.instance.signOut();
        }
      } catch (e) 
      {
        await FirebaseAuth.instance.signOut();
      }
    }

    setState(() 
    {
      _isUserValid = false;
    });
  }

  @override
  Widget build(BuildContext context) 
  {
    if (_isUserValid) 
    {
      return const SplashScreen();
    }

    return StreamBuilder<User?>(
      stream: FirebaseAuth.instance.authStateChanges(),
      builder: (ctx, snapshot) {
        if (snapshot.connectionState == ConnectionState.waiting) 
        {
          return const SplashScreen();
        }

        if (snapshot.hasData) 
        {
          return const MainScreen();
        }

        return const AuthScreen();
      },
    );
  }
}

