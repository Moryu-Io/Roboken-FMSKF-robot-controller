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

## FreeRTOSの初期化修正とFault診断

2026-11-01、初回INITでの再起動を引き起こしていたMSP初期化を修正した。
TeensyduinoのRAMベクタテーブルはslot 0も`unused_interrupt_vector`で埋められる。
従来のFreeRTOSポートは`VTOR → vector[0]`を初期MSPとしていたため、割り込みの
スタック保存がITCM内の`pinMode()`等を上書きしていた。
`prvPortStartFirstTask()`はTeensyのResetHandlerと同じ`_estack`を直接ロードする。
修正後、ユーザーの実機試験で症状が解消したとの報告あり。

RT1062のNVIC優先度も実装に合わせて4bitとし、syscall maskを`0x20`、
kernel priorityを`0xF0`に修正済み。これらの修正は診断defineに依存せず常時有効。
回帰チェックは `python3 test/check_rtos_priorities.py` と、build後の
`python3 test/check_rtos_msp.py`（生成命令で`_estack`のロードを確認）。

再利用するFault診断は `src/Debug/fault_trace.hpp` のdefineで切り替える。
既定はすべて無効。通常ビルドには独自Faultハンドラ・保存RAM・起動診断ログ・
MPU設定変更を含めない。Teensy core標準のFault処理は維持する。

| build flag | 機能 | 既定値 |
| --- | --- | --- |
| `-D ENABLE_FAULT_DIAGNOSTICS=1` | PSP/MSPに対応したFault記録、レジスタ・命令列保存、起動時表示 | `0` |
| `-D ENABLE_ITCM_WRITE_GUARD=1` | 診断有効時、MPUで命令RAMへのCPU書き込みを捕捉 | `0` |

PlatformIOの既存`build_flags`へ必要なflagを追記して使う。ITCM保護には両方を指定する。
ソフトウェアbreakpoint等で実行中のコードを書き換える場合、ITCM保護は無効にする。

診断有効時はUSBシリアル接続後に`[BOOT]`、`[RTOS]`、前回の`[FAULT]` / `CrashReport`を表示する。
接続待ちで起動を止めない。位置特定には`[FAULT] stacked PC`を使う。
core標準のCrashReportはMSP固定なので、FreeRTOSタスクのPCと異なる場合がある。
Fault時のPC周辺64 bytesをFlash内の元コードと比較し、`[CODE]`で出力する。
不正なstackやコード範囲は参照しない。記録はリセットで消えないRAMに識別値・checksum付きで保存する。
既存のカスタムFaultハンドラは置き換えず、標準の報告・自動再起動処理へ戻す。
再起動だけではWatchdogと断定せず、リセット要因とFault種別を確認する。

調査時は実機へ書き込んだ`firmware.elf`とログを対で保存する。再buildするとアドレスが変わる。
記録・範囲チェック・MPU設定保持のホスト検査は `python3 test/check_fault_trace.py`。
GPIO32/34専用のDWT監視、初回INITの段階ログ、旧BASEPRI値の比較検査は撤去した。

micro-ROSは時刻同期に失敗しても購読を維持し、20msの試行を1秒間隔で繰り返す。
同期までは終端イベントをADTの既存16件queueに保持し、同期後に送信する。
接続生成等の長い処理後は周期待ちの基準を更新して、他タスクを圧迫する追い付き実行を避ける。
これらは接続処理の修正として維持する。

## WebUIのマニュアル操作

`Command=12`で専用の手動モードへ入ります。モード切り替え自体は腕の移動やトルクONを行いません。
WebUIは停止指令を先に送り、必要な初期化（4）と構え（1）は別操作にします。
手動では構えの実動作完了後、最初のTimeAngle受信時にPOSITIONING_SEQへ遷移します。
自律開始（2）を送る必要はありません。実行中のTimeAngleと重なる要求は拒否します。

`manual_control/hardware_state`（std_msgs/UInt8）をVehicleInfoと同じ周期で配信します。
0=手動無効、1=手動有効・構え未完了、2=構え完了・打突受付可能、3=打突実行中です。
0（停止・トルクOFF）・2（自律開始）・3（終了）で手動を解除し、1（構え）・4（初期化）では維持します。
走行の床・IMU保護とcmd_velの500 ms watchdogは既存どおりです。

対応するROS側のinterfaces・arm_action_plan・vehicle_action・robot_web_uiと併せて更新してください。
通常の`pio run -e teensy41`でビルドできます。既存のCommandと標準UInt8を使うため、
この変更に伴うmicro_ros_arduinoライブラリの再生成は不要です。書き込みは機体の停止を確認して別途行います。

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
