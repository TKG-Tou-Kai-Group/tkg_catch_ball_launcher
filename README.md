# tkg_catch_ball_launcher

Core 2025 自動ロボットでキャッチボールするためのプログラム

## 特徴
- 6mm弾エアガンの安全基準である0.98Jで発射速度を設定

  補足するとボールに0.98Jを加える際のローラーの回転数約3800rpmに射出時に減速する分約1000rpmを加えた4800rpmで制御(ボール質量0.03kg、ローラー半径を0.02mと仮定)

- 測域センサにより2mから4mの範囲内で最も近い目標にボールを投射
- 画像処理で認識している物体の大まかな領域を切り出すサンプルあり
- デプスセンサで対象物の大まかな高さを認識して適切な高さに投射

## 使い方

1. 以下のコマンドでROS 2ワークスペースに本リポジトリをクローンしてください。
   ```bash
   git clone xxx
   ```

2. プログラムをビルドし、インストールする
   ```bash
   colcon build
   source install/setup.sh
   ```

3. 以下のコマンドでセンサ、モータコントローラ、制御プログラムを一度に立ち上げられます。
   ```bash
   ros2 launch tkg_catch_ball_launcher robot_launcher.launch.py
   ```

注：モータコントローラにはTKGが独自で開発した[プログラム](https://github.com/TKG-Tou-Kai-Group/tkg_autorobot_controller)を使用しています。
　　ちなみに、運営が提供している[サンプルプログラム](https://github.com/scramble-robot/CoRE_AutoRobot_2024_sample)ではローラが3800rpmでうまく回転しなかったので手を加えました。
　　利用したいモータコントローラに合わせて適宜変更してください。

注：デフォルトでは誤射を防ぐためにconfigに記述された測域センサの検知範囲は狭くなっています。使用環境に合わせて適宜変更してください。
