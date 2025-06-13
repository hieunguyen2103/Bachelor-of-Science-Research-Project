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

  String originalFullName = '';
  String originalPhone = '';
  String originalEmail = '';
  String originalFloor = '';
  String currentPassword = ''; 

  String newPassword = '';
  String fullName = '';
  String phone = '';
  String email = '';
  String password = '';
  String selectedFloor = '';
  final _formKey = GlobalKey<FormState>();

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
            fullName = data['fullname'] ?? '';
            phone = data.containsKey('phone') ? data['phone'] ?? '' : '';
            email = data['email'];
            selectedFloor = data['level_id'] ?? '';

            originalFloor = selectedFloor;
            originalFullName = fullName;
            originalPhone = phone;
            originalEmail = email;
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

  Future<void> _changePassword() async {
    try {
      final user = FirebaseAuth.instance.currentUser!;
      final cred = EmailAuthProvider.credential(
        email: user.email!,
        password: currentPassword,
      );

      await user.reauthenticateWithCredential(cred);
      await user.updatePassword(newPassword);

      // Sau khi đổi mật khẩu thành công thì sign out và quay về màn hình Auth
      await FirebaseAuth.instance.signOut();
      Navigator.of(context).popUntil((route) => route.isFirst);  // Quay về AuthScreen

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Đổi mật khẩu thành công. Vui lòng đăng nhập lại.')),
      );
    } catch (e) {
      print('Password change failed: $e');
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Lỗi: $e')),
      );
    }
  }

  Future<void> _updateUser() async {
    if (!_formKey.currentState!.validate()) return;

    try {
      final user = FirebaseAuth.instance.currentUser!;
      final uid = user.uid;

      // So sánh với dữ liệu cũ để chỉ update nếu thay đổi
      Map<String, dynamic> updateData = {};

      if (fullName.isNotEmpty && fullName != originalFullName) {
        updateData['fullname'] = fullName;
      }

      if (phone != originalPhone && phone.isNotEmpty) {
        updateData['phone'] = phone;
      }

      if (selectedFloor != originalFloor && selectedFloor.isNotEmpty) {
        updateData['level_id'] = selectedFloor;
      }

      if (updateData.isNotEmpty) {
        await FirebaseFirestore.instance.collection('users').doc(uid).update(updateData);
      }

      if (email != originalEmail) {
        await user.updateEmail(email);
      }

      if (password.isNotEmpty) {
        await user.updatePassword(password);
      }

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('Cập nhật thành công'),
          backgroundColor: Colors.green,
          duration: Duration(seconds: 2),
        ),
      );
    } catch (e) {
      print('Update failed: $e');
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Lỗi: $e')),
      );
    }
  }

  void initState() 
  {
    super.initState();
    _getUser();
  }

  InputDecoration inputDecoration(String label, IconData icon) {
    return InputDecoration(
      labelText: label,
      prefixIcon: Icon(icon),
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(12),
      ),
      contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 12),
    );
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
        child: _isLoading
            ? CircularProgressIndicator()
            : _isAccountActivated
                ? Padding(
                    padding: const EdgeInsets.all(16.0),
                    child: Form(
                      key: _formKey,
                      child: ListView(
                        children: [
                          TextFormField(
                            initialValue: fullName,
                            decoration: inputDecoration('Họ và tên', Icons.person),
                            onChanged: (value) => fullName = value.trim(),
                          ),
                          SizedBox(height: 12),
                          TextFormField(
                            initialValue: phone,
                            decoration: inputDecoration('Số điện thoại', Icons.phone),
                            onChanged: (value) => phone = value.trim(),
                          ),
                          SizedBox(height: 12),
                          TextFormField(
                            initialValue: email,
                            decoration: inputDecoration('Email', Icons.email),
                            onChanged: (value) => email = value.trim(),
                            validator: (value) =>
                                value!.contains('@') ? null : 'Email không hợp lệ',
                          ),
                          SizedBox(height: 12),
                          DropdownMenu<String>(
                            initialSelection: selectedFloor,
                            label: const Text('Tầng'),
                            width: MediaQuery.of(context).size.width - 32, // same width as form
                            leadingIcon: const Icon(Icons.apartment),
                            onSelected: (value) {
                              setState(() {
                                selectedFloor = value!;
                              });
                            },
                            dropdownMenuEntries: List.generate(9, (index) {
                              final value = (index + 1).toString().padLeft(2, '0');
                              return DropdownMenuEntry(
                                value: value,
                                label: 'Tầng ${index + 1}',
                              );
                            }),
                          ),

                          SizedBox(height: 20),
                          ElevatedButton(
                            onPressed: _updateUser,
                            child: Text('Cập nhật thông tin'),
                          ),
                          SizedBox(height: 30),
                          Divider(),
                          Text(
                            'Đổi mật khẩu',
                            style:
                                TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
                          ),
                          SizedBox(height: 12),
                          TextFormField(
                            decoration: inputDecoration(
                                'Mật khẩu hiện tại', Icons.lock_outline),
                            onChanged: (value) => currentPassword = value,
                            obscureText: true,
                          ),
                          SizedBox(height: 12),
                          TextFormField(
                            decoration:
                                inputDecoration('Mật khẩu mới', Icons.lock),
                            onChanged: (value) => newPassword = value,
                            obscureText: true,
                          ),
                          SizedBox(height: 12),
                          ElevatedButton(
                            onPressed: () {
                              if (currentPassword.isEmpty ||
                                  newPassword.isEmpty) {
                                ScaffoldMessenger.of(context).showSnackBar(SnackBar(
                                  content: Text(
                                      'Vui lòng nhập cả mật khẩu hiện tại và mật khẩu mới'),
                                ));
                              } else {
                                _changePassword();
                              }
                            },
                            child: Text('Đổi mật khẩu'),
                          ),
                        ],
                      ),
                    ),
                  )
                : Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Text(
                        'No account information found!\nActivate now?',
                        style: TextStyle(fontSize: 16),
                        textAlign: TextAlign.center,
                      ),
                      SizedBox(height: 16),
                      ElevatedButton(
                        style: ElevatedButton.styleFrom(
                          backgroundColor:
                              Theme.of(context).colorScheme.primaryContainer,
                        ),
                        onPressed: () {
                          Navigator.of(context).push(MaterialPageRoute(
                              builder: (ctx) => const ActivateAccountScreen()));
                        },
                        child: Text('Activate'),
                      ),
                    ],
                  ),
      )
    );
  }
}