## 新規Msgの追加方法

`micro_ros_arduino\extras\library_generation\extra_packages`直下にmsgフォルダを作る

[公式チュートリアル](https://micro.ros.org/docs/tutorials/advanced/create_new_type/)に従って、.msgファイルの作成まで行う

micro_ros_arduinoライブラリを作成する
wsl2にて以下のコマンドを発行

```
sudo service docker start

docker pull microros/micro_ros_static_library_builder:foxy
docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:foxy -p teensy4
```

通信するPC側では同様のmsg定義フォルダを作成し
```
colcon build --symlink-install --packages-up-to my_messages
. install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 9999
```