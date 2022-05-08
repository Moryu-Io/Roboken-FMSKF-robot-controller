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
