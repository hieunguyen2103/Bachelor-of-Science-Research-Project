import 'package:flutter/material.dart';

class NotificationIconButton extends StatelessWidget {
  final VoidCallback? onTap;

  const NotificationIconButton({super.key, this.onTap});

  @override
  Widget build(BuildContext context) {
    return IconButton(
      icon: const Icon(Icons.notifications),
      color: Theme.of(context).colorScheme.primary,
      onPressed: onTap ??
          () {
            showDialog(
              context: context,
              builder: (ctx) => AlertDialog(
                title: const Text('Notifications'),
                content: const Text('No fire alerts at the moment.'),
                actions: [
                  TextButton(
                    onPressed: () => Navigator.of(ctx).pop(),
                    child: const Text('Close'),
                  )
                ],
              ),
            );
          },
    );
  }
}
