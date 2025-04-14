import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

final _firebase = FirebaseAuth.instance;  // Lấy đối tượng firebase trong flutter

class AuthScreen extends StatefulWidget
{
  const AuthScreen({super.key});

  static const routeName = '/login';

  @override
  State<AuthScreen> createState() {
    return _AuthScreenState();
  }
}

class _AuthScreenState extends State<AuthScreen>
{
  final _formKey = GlobalKey<FormState>();  // GlobalKey<FormState> là “remote control” để kiểm tra toàn bộ form. Đơn giản là cái biến formKey đó sẽ lấy ra trạng thái của Form

  var _isLogin = true;
  var _enteredEmail = '';
  var _enteredPassWord = '';
  var _enteredUsername = '';

  Future<void> _resetPassword(String email) async
  {
    print('Email reset: $email');

    try
    {
      await _firebase.sendPasswordResetEmail(email: email);
      ScaffoldMessenger.of(context).showSnackBar(
        const SnackBar(
          content: Text('Check your email to confirm the password reset request.'),
        ),
      );
    } on FirebaseAuthException catch(error)
    {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text(error.message ?? 'Failed to reset password'))
      );
    }
  }

  void _submit() async
  {
    final _isValid = _formKey.currentState!.validate();  // Thêm cái dấu ! để báo cho flutter biết là nó sẽ không rỗng
    if(!_isValid)
    {
      return;
    }
    
    _formKey.currentState!.save();
    
    try
    {
      if(_isLogin)
      {
        final userCredentials = await _firebase.signInWithEmailAndPassword(
          email: _enteredEmail, 
          password: _enteredPassWord,
        );
      }
      else
      {
        final userCredentials = await _firebase.createUserWithEmailAndPassword(
          email: _enteredEmail, 
          password: _enteredPassWord
        );

        //Tạo mã OTP để activate
        String generateOTP()
        {
          final random = DateTime.now().millisecondsSinceEpoch;
          return (random % 1000000).toString().padLeft(6, '0');
        }

        String otp = generateOTP();

        // Đảm bảo không trùng activateOTP trong Firestore
        bool _isDuplicate = true;
        while (_isDuplicate) 
        {
          final querySnapshot = await FirebaseFirestore.instance
              .collection('users')
              .where('activateOTP', isEqualTo: otp)
              .get();

          if (querySnapshot.docs.isEmpty) 
          {
            _isDuplicate = false;
          } 
          else 
          {
            otp = generateOTP(); // tạo mới nếu trùng
          }
        }

        await FirebaseFirestore.instance    // Cái này sẽ truy cập vào 1 cái collection, hoặc tạo mới nếu nó chưa tồn tại
          .collection('users')
          .doc(userCredentials.user!.uid)
          .set(    // Xác định dữ liệu sẽ được lưu trữ trong tài liệu đó
            {
              'usersname': _enteredUsername,
              'email' : _enteredEmail,
              'activated': false,
              'activateOTP': otp,
            }
          ); 
      }
    } on FirebaseAuthException catch (error)
    {
      if(error.code == 'email-already-in-use')
      {

      }
      ScaffoldMessenger.of(context).clearSnackBars();
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text(
            error.message ?? 'Authentication failed'
          ),
        ),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Theme.of(context).colorScheme.primary,  // Đặt màu nền cho giao diện này
      // Căn giữa phần body của giao diện đăng nhập này
      body: Center(
        child: SingleChildScrollView(   // Cái này để cuộn văn bản 
          child: Column(  // Tạo 1 cái Column để hiển thị những thành phần bên trong cho có trật tự hơn (xếp từ trên xuống dưới)
            mainAxisAlignment: MainAxisAlignment.center,  // Căn giữa toàn bộ nội dung trong Column
            children: [  // Phần này là liệt kê những widget muốn hiển thị trong Column
              Container(  // Thêm cái này vào như một cái hộp chứa các tiện ích, căn lề trong cái box này
                margin: EdgeInsets.only(  // Tạo khoảng cách căn 4 hướng trong box
                  top: 30,
                  bottom: 20,
                  left: 20,
                  right: 20,
                ),
                width: 200,  // Đây là chiều rộng của Container
                child: Image.asset('assets/images/logo.png'),
              ),
              Card(  // Thêm cái này vào để kiểu dáng phần đăng nhập nó đẹp mắt
                margin: const EdgeInsets.all(20),
                child: SingleChildScrollView(
                  child: Padding(
                    padding: const EdgeInsets.all(16),
                    child: Form(
                      key: _formKey,  // Gán cái formKey đã khai báo bên trên vào key của cái Form này
                      child: Column(
                        mainAxisSize: MainAxisSize.min,  // Đảm bảo cột này chỉ chiếm lượng không gian vừa đủ theo nội dung của nó
                        children: [  // Những cái sẽ hiển thị trong cái chỗ log in
                          TextFormField(  
                            decoration: InputDecoration(
                              labelText: 'Email Address',
                            ),
                            keyboardType: TextInputType.emailAddress,
                            autocorrect: false,  // Không tự sửa ký tự sai
                            textCapitalization: TextCapitalization.none,  // Đảm bảo email không được viết hoa
                            validator: (value) {  // Cái value là flutter cung cấp cho, chính là cái chuỗi nhập vào form
                              if(value == null || value.trim().isEmpty || !value.contains('@'))   // Nếu như có lỗi
                              {
                                return 'Please enter a valid email address';
                              }

                              return null;  // Không có lỗi gì thì trả về null
                            },
                            onSaved: (value)  // Trong cái hàm submit, nếu _isValid == true thì nó sẽ gọi cái hàm save, chính là kích hoạt cái onSaved này, nó sẽ tự động lấy giá trị hiện tại được nhập vào
                            {
                              _enteredEmail = value!;
                            },
                          ),

                          if(!_isLogin)  // Nếu đăng ký tài khoản mới thì mới cần hiện Username ở form
                            TextFormField(
                              decoration: const InputDecoration(
                                labelText: 'Username'
                              ),
                              enableSuggestions: false,  // Đảm bảo bàn phím không đưa ra gợi ý để nhập
                              validator: (value)
                              {
                                if(value == null || value.isEmpty || value.trim().length < 4)
                                {
                                  return 'Please enter at least 4 characters';
                                }
                                return null;
                              },
                              onSaved: (value)
                              {
                                _enteredUsername = value!;
                              },
                            ),

                          TextFormField(
                            decoration: InputDecoration(
                              labelText: 'Password',
                            ),
                            obscureText: true,  // Ẩn các ký tự mà người dùng nhập vào
                            validator: (value) {  // Cái value là flutter cung cấp cho, chính là cái chuỗi nhập vào form
                              if(value == null || value.trim().length < 6)   // Nếu như độ dài password nhỏ hơn 6 (6 là độ dài tối thiểu mà firebase yêu cầu)
                              {
                                return 'Password must be at least 6 characters long.';
                              }

                              return null;  // Không có lỗi gì thì trả về null
                            },

                            onSaved: (value)
                            {
                              _enteredPassWord = value!;
                            },
                          ),

                          const SizedBox(  // Thêm khoảng cách đến các nút
                            height: 12,
                          ),
                          ElevatedButton(  // Nút có bóng đổ, nhìn giống như nổi lên khỏi màn hình
                            onPressed: _submit, 
                            style: ElevatedButton.styleFrom(  // Tạo màu nền cho cái nút nổi này
                              backgroundColor: Theme.of(context).colorScheme.primaryContainer
                            ),
                            child: Text(_isLogin ? 'Login' : 'Sign up'),
                          ),
                          
                          if(_isLogin)
                          TextButton(
                            onPressed: () {
                              final _emailController = TextEditingController(); // Cái này để lấy giá trị email mà người dùng nhập

                              showDialog(
                                context: context, 
                                builder: (ctx) => AlertDialog(
                                  title: const Text('Forgot Password'),
                                  content: TextField(  // Cái textField này để người dùng điền email reset password
                                    controller: _emailController,  // Tạo 1 controller điều khiển giá trị văn bản trong TextField
                                    decoration: const InputDecoration(
                                      labelText: 'Enter your registered email',
                                    ),
                                    keyboardType: TextInputType.emailAddress,
                                    autofocus: true,  // cái này là tự động nhấp nháy sẵn vào ô cần nhập, không cần phải nhấn vào textfield
                                  ),
                                  actions: [
                                    TextButton(
                                      onPressed: () {
                                        final _enteredEmailReset = _emailController.text.trim();
                                        if(_enteredEmailReset.isEmpty || !_enteredEmailReset.contains('@'))
                                        {
                                          ScaffoldMessenger.of(context).showSnackBar(
                                            const SnackBar(content: Text('Please enter a valid email address'))
                                          );
                                          return;
                                        }
                                        Navigator.of(ctx).pop();  // Tắt dialog

                                        _resetPassword(_enteredEmailReset);  // Gọi hàm reset password
                                      }, 
                                      child: const Text('OK'),
                                    ),
                                    TextButton(
                                      onPressed: () => Navigator.of(ctx).pop(),  // Nhấn cancel thì tắt cái điền email reset mật khẩu đi
                                      child: const Text('Cancel'),
                                    ),
                                  ],
                                ),
                              );
                            }, 
                            child: const Text('Forgot Password'), // Cái này là hiển thị nội dung trên nút outlineButton
                          ),
                        ],
                      ),
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 20),
              
              OutlinedButton(
                style: OutlinedButton.styleFrom(
                  foregroundColor: Colors.white,   // chữ trắng
                  side: const BorderSide(color: Colors.white),  // viền trắng
                ),
                onPressed: () {
                  setState(() {
                    _isLogin = !_isLogin;
                  });
                }, 
                child: Text(_isLogin ? 'Create an account' : 'I already have an account.'),
              ),
            ],
          ),
        ),
      ),
    );
  }
}