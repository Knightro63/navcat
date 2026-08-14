import 'dart:async';
import 'package:examples/src/common/base.dart';
import 'package:examples/src/common/debug.dart';
import 'package:examples/src/gui.dart';
import 'package:flutter/material.dart';
import 'package:three_js/three_js.dart' as three;

class Demo extends StatefulWidget {
  const Demo({super.key});
  @override
  createState() => _State();
}

class _State extends State<Demo> {
  late ExampleBase base;
  three.Camera get camera => base.camera;
  three.Scene get scene => base.scene;
  three.PeripheralsState get domElement => base.domElement;
  ThreeDebug get debug => base.debug;
  late Gui gui;

  @override
  void initState() {
    base = ExampleBase(
      setState,
      setup,
    );
    gui = Gui((){setState(() {});});
    super.initState();
  }
  @override
  void dispose() {
    base.dispose();
    three.loading.clear();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          base.build,
          if(base.threeJs.mounted)Positioned(
            top: 20,
            right: 20,
            child: SizedBox(
              height: base.threeJs.height,
              width: 240,
              child: gui.render()
            )
          )  
        ],
      ) 
    );
  }

  Future<void> setup() async {

  }
}
