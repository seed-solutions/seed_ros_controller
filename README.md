# seed_ros_controller

## SEED-Driverの設定
[HP](http://seed-solutions.net/?q=seed_jp/node/7)よりSEED Editorをダウンロードし、
アクチュエータのID設定、原点復帰(キャリブレーション)スクリプトの作成等を行う。

## サンプル概要
本パッケージには、サンプルとして下記ジョイントを有するスカラロボットのURDFファイルがある。   
```
- torso_joint
- shoulder_joint
- elbow_joint
- eef_joint
```
ID１〜４まで設定されたSEED-Driverを有するアクチュエータを接続し、下記コマンドを実行すれば起動する。
```
$ roslaunch seed_ros_controller bringup.launch
```
原点復帰スクリプトが記述されていない場合、下記コマンドで実行すること。
**その場合、電源投入時の位置が原点となる。**
```
$ roslaunch seed_ros_controller bringup.launch calibration:=false
```

各軸の動作確認時は、`` rqt_joint_trajectory_controller``などを使えば良い。

## オリジナルロボットへの適用
オリジナルロボットにSEED-Driverを組み込む場合、下記手順を行うこと。
1. ジョイント名を記述したモデルファイル（urdfファイル）を作成する  
``/urdf/sample_robot.urdf``を参照
2. ジョイント名と制御方式を記述したコントローラ設定ファイルを作成する  
``/config/controller.yaml``を参照
3. ジョイント名とSEED-Driverの設定項目を記述したハードウェア設定ファイルを作成する  
``/config/hardware.yaml``を参照

### hardware.yamlについて
``name:``の順番と``id:``、``actuator:``の順番が紐付けられている。  
たとえば下記設定の場合、``torso_joint``へ指令値が送られると、ID1のHAシリーズが動作する。
```yaml
hardware_interface:
  joint_settings:
    name:
      - torso_joint
      - shoulder_joint
      - elbow_joint
      - eef_joint
    id:
      - 1
      - 2
      - 3
      - 4
    actuator:
      - HA
      - BA
      - HA
      - BA42
```
actuatorの種類は[HP](http://seed-solutions.net/?q=seed_jp/node/72)を参照のこと。
