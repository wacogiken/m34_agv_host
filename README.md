# m34_agv_host
2016年の物流展向けAGVデモ機用ROSプログラム。  
画面表示を行うPC上のROSで動作する。

![agv](img/agv.jpg)

### 1. hardware

種別|品名
----|----
PC|i7-4790(3.6GHz) + GTX750Ti / Ubuntu 14.04LTS + ROS Indigo
Wi-Fi Router|NEC Aterm WG1200HS
920MHz Wireless|Interplan IM920XT + IM315-USB-RX
joystick|ELECOM JC-AS01BK(改) + Arduino UNO

### 2. serial device rules

ベンダID|デバイス名
----|----
2341|/dev/arduino
0403|/dev/im920

### 3. package & Patch
* ros_controllers/diff_drive_controller  
  走行／旋回と左右軸の変換に使用している。  
  現在位置(odometry)を設定するためにパッチを当てている。  
  バージョンは0.9.3(2016-02-12)
* jsk_visualization_packages  
  オーバレイ波形表示とアイコン表示に使用している。  
  現在(2016_09_24)はjsk_visualizationですが動作は未確認。

### 4. Arduino  
ELECOMのゲームパッドは制御基板を外し、Joystich/Switchの接点をArduino UNOの入力に接続している。

### 5. MQTT
hostとraspi間の通信は5GHz Wi-Fi/MQTTと920MHz Wireless/serialで行う。
ROSの分散システムも考えたが、AGVはバッテリ交換時に電源が切れることと展示会場でのWi-Fiの混信を懸念し用いていない。
メイン通信はhostで動作しているMQTTサーバで、バックアップを920MHzとしている。

実機AGV無しで動かす場合、arduinoとim920rのノードを起動せず、MQTTサーバへ以下のトピックを発行し続ければ可能と思う。

topic: /agv  
message: 走行速度[min-1]:旋回速度[min-1]:電圧[V]:番地(0～33):角度[0.1deg]

### 6. ROS node

ノード名|ソース名|内容
----|----|----
arduino|serial_node.py|Joystich/Switchの状態を読み出す。
shutdown|system_stop.py|MQTTによるホストPCのシャットダウンを行う。
camera|camera.py|joystickによるrvizの視点移動等を行う。
im920r|im920r.py|raspyのim920sとの通信を行う。
agv_recv|agv_recv.py|MQTTとim920sのデータよりagvデータを生成。
agv_ctrl|agv_ctrl.cpp/agv_hard.cpp|agvデータより左右軸とテーブル角度を生成。
control_spawner|spawner|(ROS package)
robot_state_publisher|state_publisher|(ROS package)
map_server|map_server|(ROS package)
map_odom_broadcaster|static_transform_publisher|(ROS package)
rviz|rviz|(ROS package)
