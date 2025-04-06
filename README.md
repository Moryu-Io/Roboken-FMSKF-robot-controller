# Roboken-FMSKF-robot-controller
## 概要
Teensy4.1を使用した新ROBO-剣出場ロボット[Quinque](https://www.robo-one.com/rankings/view/1127)のHW制御統括マイコンFW  
主な役割は以下
- Ethernet経由Micro-ROSで、ロボット全体を統括する上位LinuxPCと情報をやり取りする
- 5軸ロボットアームへの指示、情報収集
- 4輪メカナムホイールモータの速度制御
- 距離センサを用いて床や相手ロボットの検知を簡易的に行う
- 相手ロボット認識用のカメラをPan/Tiltさせるサーボモータを制御


## SW構成(src内)
すべてFreeRTOSのタスクベースでフォルダ分け

| フォルダ名   | 説明                                                                                            |
| ------------ | ----------------------------------------------------------------------------------------------- |
|RobotManager|Ethernet経由Micro-ROSで、上位のLinuxPCと情報をやり取りしながらHW行動を決定するモジュール|
|ArmDrive|5軸ロボットアームの制御を行う。各関節ごとに[ドライバ](https://github.com/Moryu-Io/Bldc_Servo_Driver_Prj)内で位置/トルク制御を行うため、このモジュールでは指示を統括|
|VehicleDrive|4輪メカナムホイールのモータ制御を行う。こちらはDJIのM2006,C610を使用しており、ドライバ内でのトルク制御を行うため、モータの速度制御から上位の指示/制御を統括|
|FloorDetect|競技エリアから落ちないように距離センサで床面をセンシングするモジュール|
|CameraGimbal|カメラをPan/Tiltさせる汎用サーボモータの制御モジュール|
|Logger|Print文をSDカードに保存するためのDebug用モジュール|
|Debug|DebugPrintやRTOSのリソース状況を出力するDebug用モジュール|
|Utility|汎用モジュール。自作の計算処理など|

## 使用ライブラリ
| フォルダ名   | 説明                                                                                            |
| ------------ | ----------------------------------------------------------------------------------------------- |
|[FreeRTOS-Teensy4](https://github.com/juliandesvignes/FreeRTOS-Teensy4)|RTOS|
|[micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)|Micro-ROS Quinque用のカスタムメッセージを含めてビルドしたものを同梱|
|[IcsClass_V210](https://kondo-robot.com/faq/ics-library-a2)|一部使用している近藤科学製サーボモータ制御用|
|[TsyDMASPI](https://github.com/hideakitai/TsyDMASPI)|IMU用SPIだが現状ほぼ未使用|

# 雑多なメモ
## 新規Msgの追加方法
### msgファイルの作成

`micro_ros_arduino\extras\library_generation\extra_packages`直下にmsgフォルダを作る

[公式チュートリアル](https://micro.ros.org/docs/tutorials/advanced/create_new_type/)に従って、.msgファイルの作成とCMakeLists.txtへの登録を行う

ここには記載がないが、package.xml末尾に以下を追加する必要もある
```
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

### micro_ros_arduinoライブラリを作成する
wsl2にて以下のコマンドを発行
```
sudo service docker start

docker pull microros/micro_ros_static_library_builder:foxy
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:foxy -p teensy4
```
※2022/05/08 wsl2で上記コマンドでdocker使用時に名前解決ができなくなっていた。原因不明。

[このサイト](https://qiita.com/takkeybook/items/37af683bfb1a4eca6aa2)に従って、`/etc/resolv.conf`に記載のDNSサーバーを変更する


### PC側での準備
通信するPC側では同様のmsg定義フォルダを作成
```
colcon build --symlink-install --packages-up-to interfaces
. install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 9999
```

### Micro-ROSテスト方法memo
- topic
```
ros2 topic echo /VehicleInfo
```

- service
```
ros2 service call /ProcStatus interfaces/srv/ProcStatus "{id: 1}"
```

## FreeRTOS

こちらのリポジトリをベースに、libフォルダに格納
https://github.com/juliandesvignes/FreeRTOS-Teensy4

変更点
* heap3からheap4に
    * heap3はC言語のmallocなどのラッパだが、Teensyduinoではmallocで確保されるのは通常RAM。
    * heap4でucHeapをTCMに配置することで高速性を担保
* 別途configファイルを設定
