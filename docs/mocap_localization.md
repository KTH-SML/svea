# MoCap Localization (Qualisys)

To start the Qualisys driver for motion capture localization, you need to launch the `mocap_qualisys` node with the QTM server address. 

For more information, please check the README on the [DISCOWER motion_capture_system git repository](https://github.com/DISCOWER/motion_capture_system).

Run the following command:

```bash
ros2 launch mocap_qualisys qualisys.launch.py server_address:=10.0.0.10
```