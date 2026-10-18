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

## アーム実行進捗

`ArmExecutionProgress`（相対topic、reliable）で、`header.stamp`は通知時刻、
`id`はTimeAngle ID、`elapsed_ms`はADTで実際に指令した補間位置、
`status`はRUNNING=0 / COMPLETED=1 / ABORTED=2を送信する。
受信・queue投入時点ではRUNNINGにしない。終端イベントは16件の専用queueに保持し、
RMTタスクから送信する。進行中のsnapshotはcritical sectionで受け渡す。
ArmInfoとProcStatusは引き続き利用できる。

TimeAngleは5軸の点数・各点の時刻が一致する1～32点、時刻は非減少であること。
受信データはADT向けmessageへコピーし、次のROS callbackによる上書きを防ぐ。
不正入力、buffer満杯、POSITIONING_SEQモード外の要求はABORTEDを通知する。
INIT/READYの実動作完了後にMOVE_STARTを送り、その後TimeAngleを送ること。
初期化の途中に送ったMOVE_STARTは実モード遷移を保証せず、早いTimeAngleは拒否される。

RELAX/QUIT_PGは通常message bufferを迂回する停止flagでADT次周期（10ms）に強制OFFし、
実行中・queue済みの軌道をABORTEDにする。pending要求も破棄し、古いMOVE_STARTによる
再始動を防ぐ。停止と同じ制御周期内の再開要求も破棄され得るので、停止確認後に再開する。

### micro-ROS生成物の再生成

同梱版はmicro_ros_arduino 2.0.7-humble。既存のRMW・transport・既存メッセージを維持し、
ArmExecutionProgress専用の生成headerと4個のobjectだけを静的archiveへ追加している。
`extras/library_generation/extra_packages/interfaces/msg/ArmExecutionProgress.msg`が生成元。
以下はローカルにあるDocker builderを使用する（未取得の場合は対応するhumble imageを取得し、
`MICROROS_BUILDER_IMAGE`へ指定）。初回のfirmware workspace作成にはソース取得の通信が必要。

```sh
bash lib/micro_ros_arduino/extras/library_generation/regenerate_arm_progress.sh /tmp/arm-progress-microros
pio run
```

再生成scriptは既存の作業workspaceを再利用する。既存runtime headerとの宣言一致を確認してから
新型objectを追加する。検証使用image ID:
`sha256:bab8476bc71f52332596f2ce15417696ce8ecfbd5a4d3b569c716a5bd34327a4`。
2026-10-09にHumble builderで68 packages生成、PlatformIO teensy41のリンクまで成功。
RMW publisher上限10のうち3枠を使用する。

### メモリ予算と検証範囲

32点×5軸copyによりMSG_REQは32→780 bytes、3件のADT message bufferは
96→2353 bytes（今回はmessage長の管理領域も確保）。共有受信MSG_REQも748 bytes増加する。
終端queueのpayloadは16×12=192 bytesとqueue管理領域。
FreeRTOS heapは36→40 KiBへ増量。設定された9タスクのstack合計は28,160 bytes、
kernel idle/timerのstackは追加1,080 bytes。40 KiBからこれらとADT buffer・終端payloadを引いた
約9 KiBがTCB、他message buffer、queue管理領域等に充てられる（実測空きheapではない）。
RMT stackは既存2304 words、ADTは1024 wordsを維持。

PlatformIOの静的集計: FLASH code 405,988 / data 100,600 bytes、
RAM1 variables 210,020 / code 271,324 / padding 23,588 / local空き19,356 bytes、
RAM2 variables 12,512 / malloc空き511,776 bytes。
実機への書込み・動作試験は未実施で、runtime heap最小値・stack high-waterは未計測。
