import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:fire_guard/main.dart';
import 'package:fire_guard/screens/accountSettingScreen.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';

class ActivateAccountScreen extends StatefulWidget
{
  const ActivateAccountScreen({super.key});

  @override
  State<ActivateAccountScreen> createState() {
    // TODO: implement createState
    return _ActivateAccountScreenState();
  }
}

class _ActivateAccountScreenState extends State<ActivateAccountScreen>
{
  // cái focusNode này giúp tự động nhảy sang ô kế tiếp sau khi nhập
  /* Đoạn generate, 6 là độ dài (tức là mã OTP kích hoạt này gồm 6 số).
  List.generate(length, (index) => expression) cấu trúc của nó là như này thì cái index là hàm 
  ứng với mỗi phần tử nhập vào, ở đây thì không cần hàm ứng với mỗi phần tử nhập vào, đơn giản chỉ là nhập số vào thôi nên 
  để (_) tức là không quan tâm đến index */
  final List<FocusNode> _focusNodes = List.generate(6, (_) => FocusNode());
  
  // Cái này cũng tương tự, nhưng là để lấy giá trị mà người dùng nhập vào
  final List<TextEditingController> _controllers = List.generate(6, (_) => TextEditingController());

  @override
  void dispose()  // Phần này để hủy các đối tượng dùng tài nguyên hệ thống sau khi không sử dụng nữa
  {
    for (var ctrl in _controllers) // Duyệt qua các _controllers đã tạo
    {
      ctrl.dispose(); // Hủy từng _controllers
    }
    for (var node in _focusNodes) // Duyệt qua các _focusNodes đã tạo
    {
      node.dispose();   // Hủy từng _focusNodes()
    }
    super.dispose();
  }

  void _onChanged(String value, int index) // Xử lý khi mà dữ liệu trong các ô thay đổi
  {
    if (value.length == 1 && index < 5) // Nếu người dùng đã nhập dữ liệu vào 1 ô và ô đó không phải ô cuối cùng (tức là ô thứ 5)
    {
      FocusScope.of(context).requestFocus(_focusNodes[index + 1]);  // Thì nhảy focus sang ô kế tiếp
    } 
    if (value.isEmpty && index > 0) // Nếu người dùng xóa dữ liệu của ô và ô đó không phải ô đầu tiên
    {
      FocusScope.of(context).requestFocus(_focusNodes[index - 1]);  // Thì nhảy focus sang ô trước đó
    }
  }

  void _submitOTP() async
  {
    /* Controller là 1 danh sách chứa TextEditingController ứng với từng ô OTP. Dùng map để lấy dữ liệu từng ô trả về kiểu text.
    Sau khi lấy xong từng ô thì ghép chúng lại bằng cách dùng join() */
    final otp = _controllers.map((c) => c.text).join();

    print("OTP Entered: $otp"); // sau này sẽ gửi lên Firebase ở đây

    final uid = FirebaseAuth.instance.currentUser?.uid;

    if(uid == null)
    {
      print("Your acconut has been deleted!");
      return;
    }
    
    final doc = await FirebaseFirestore.instance.collection('users').doc(uid).get();
    final correctOTP = doc.data()?['activateOTP'];

    if (otp == correctOTP) {
      // Cập nhật trạng thái kích hoạt
      await FirebaseFirestore.instance.collection('users').doc(uid).update({
        'activated': true,
      });

      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Tài khoản đã được kích hoạt!')),
      );

      Navigator.pushReplacement(
        context,
        MaterialPageRoute(builder: (context) => const AccountSettingScreen()),
      );

    } else {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Sai mã OTP!')),
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Activate Account')),
      body: Padding(
        padding: const EdgeInsets.all(24.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text('Enter the 6-digit code sent to your email/phone:'),
            const SizedBox(height: 20),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: List.generate(6, (index) {
                return SizedBox(
                  width: 40,
                  child: TextField(
                    controller: _controllers[index],
                    focusNode: _focusNodes[index],
                    keyboardType: TextInputType.number,
                    textAlign: TextAlign.center,
                    maxLength: 1,
                    decoration: const InputDecoration(
                      counterText: '',
                    ),
                    onChanged: (value) => _onChanged(value, index),
                  ),
                );
              }),
            ),
            const SizedBox(height: 30),
            ElevatedButton(
              onPressed: _submitOTP,
              child: const Text('OK'),
              style: ElevatedButton.styleFrom(  // Tạo màu nền cho cái nút nổi này
                backgroundColor: Theme.of(context).colorScheme.primaryContainer
              ),
            )
          ],
        ),
      ),
    );
  }
}