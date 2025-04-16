import 'package:flutter/material.dart';
import 'dart:math';

class SemiCircleGauge extends StatelessWidget {
  final String label;
  final double value;
  final String unit;
  final double min;
  final double max;
  final List<Color> colors;

  const SemiCircleGauge({
    super.key,
    required this.label,
    required this.value,
    required this.unit,
    this.min = 0,
    this.max = 100,
    required this.colors,
  });

  @override
  Widget build(BuildContext context) {
    double percent = ((value - min) / (max - min)).clamp(0.0, 1.0);

    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        Text(label,
            style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
        const SizedBox(height: 10),
        SizedBox(
          width: 120,
          height: 70,
          child: Stack(
            alignment: Alignment.center,
            children: [
              CustomPaint(
                size: const Size(120, 60),
                painter: SemiCirclePainter(
                  percent: percent,
                  colors: colors,
                ),
              ),
              Text(
                '${value.toStringAsFixed(1)} $unit',
                style: const TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: Colors.white),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

class SemiCirclePainter extends CustomPainter {
  final double percent;
  final List<Color> colors;

  SemiCirclePainter({required this.percent, required this.colors});

  @override
  void paint(Canvas canvas, Size size) {
    final Paint background = Paint()
      ..color = Colors.grey.shade800
      ..style = PaintingStyle.stroke
      ..strokeWidth = 10;

    final Paint foreground = Paint()
      ..shader = SweepGradient(
        startAngle: -pi,
        endAngle: 0,
        colors: colors,
      ).createShader(Rect.fromCircle(center: Offset(size.width / 2, size.height), radius: size.width / 2))
      ..style = PaintingStyle.stroke
      ..strokeWidth = 10
      ..strokeCap = StrokeCap.round;

    final center = Offset(size.width / 2, size.height);
    final radius = size.width / 2;

    canvas.drawArc(
        Rect.fromCircle(center: center, radius: radius),
        -pi,
        pi,
        false,
        background);

    canvas.drawArc(
        Rect.fromCircle(center: center, radius: radius),
        -pi,
        pi * percent,
        false,
        foreground);
  }

  @override
  bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}
