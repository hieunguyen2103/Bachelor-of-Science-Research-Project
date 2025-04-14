import 'package:fire_guard/controll/authGateControll.dart';
import 'package:fire_guard/screens/authScreen.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class MainDrawerScreen extends StatelessWidget
{
  const MainDrawerScreen({super.key, required this.onSelectScreen});

  final void Function(BuildContext context, String identifier) onSelectScreen;

  @override
  Widget build(BuildContext context) {
    // TODO: implement build
    return Drawer(
      child: Column(  // Hiên thị nội dung trong drawer
        children: [
          DrawerHeader(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              gradient: LinearGradient(
                colors: [
                  Theme.of(context).colorScheme.primaryContainer,
                  Theme.of(context).colorScheme.primaryContainer.withOpacity(0.8),
                ],
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
              ),
            ),
            child: Row(
              children: [
                Icon(Icons.menu, size: 48, color: Theme.of(context).colorScheme.primary,),
                const SizedBox(width: 18,),
                Text('Menu', style: Theme.of(context).textTheme.titleLarge!.copyWith(
                  color: Theme.of(context).colorScheme.primary,
                ),),
              ],
            ),
          ),

          ListTile(
            leading: Icon(Icons.monitor, size: 26, color: Theme.of(context).colorScheme.primary,),
            title: Text('Main Screen', style: Theme.of(context).textTheme.titleSmall!.copyWith(
            color: Theme.of(context).colorScheme.onBackground,
            fontSize: 24,
              )
            ),
            onTap: () {
              onSelectScreen(context, 'Main Screen');
            },   
          ),

          ListTile(
            leading: Icon(Icons.account_box_rounded, size: 26, color: Theme.of(context).colorScheme.primary,),
            title: Text('Account', style: Theme.of(context).textTheme.titleSmall!.copyWith(
            color: Theme.of(context).colorScheme.onBackground,
            fontSize: 24,
              )
            ),
            onTap: () {
              onSelectScreen(context, 'Account');
            },   
          ),

          ListTile(
            leading: Icon(Icons.settings, size: 26, color: Theme.of(context).colorScheme.primary,),
            title: Text('Device', style: Theme.of(context).textTheme.titleSmall!.copyWith(
            color: Theme.of(context).colorScheme.onBackground,
            fontSize: 24,
              )
            ),
            onTap: () {
              onSelectScreen(context, 'Device');
            },   
          ),

          ListTile(
            leading: Icon(Icons.exit_to_app, size: 26, color: Theme.of(context).colorScheme.primary,),
            title: Text('Log out', style: Theme.of(context).textTheme.titleSmall!.copyWith(
            color: Theme.of(context).colorScheme.onBackground,
            fontSize: 24,
              )
            ),
            onTap: () async {
              try {
                Navigator.of(context).pop();
                await FirebaseAuth.instance.signOut(); // Dòng này để SignOut

                // Navigator.of(context).pushAndRemoveUntil(
                //   MaterialPageRoute(builder: (ctx) => const AuthGateControll()),
                //   (route) => false, // remove tất cả các route
                // );

                Navigator.of(context).popUntil((route) => route.isFirst);  // cái này là nó sẽ loại bỏ toàn bộ route và quay về cái route ban đầu chính là AuthScreen 

                ScaffoldMessenger.of(context).showSnackBar(
                  const SnackBar(content: Text('Logged out')),
                );
              } catch (error) {
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(content: Text('Error: ${error.toString()}')),
                );
              }
            },   
          ),
        ],
      ),
    );
  }
}