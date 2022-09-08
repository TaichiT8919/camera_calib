０．クイックスタート
０．１　変換行列の作成
$ python3 gen_cameras_matrix.py

★変更を要するグローバル変数（10 - 19行目）
・save_path：変換行列を保存するパス名
・CAMERA_ID_n：使用するRealSenseのID
・MESUREMENT_TIME：マーカーを観測する時間
・MEASUREMENT_NUM：観測回数


０．２ rvizによる映像の表示
$ roslaunch t_camera cameras_launch.launch

★変更を要する変数
./src/realsense_transform.cpp(101行目)
・input_csv_file_path_12：変換行列ファイル（.csv）のパス

./launch/caeras_launch.launch(2,3行目)
・camera1_id/camera2_id：使用するRealSenseのID

０．３ 手動による補正
完璧に座標を一致させることができないため、映像表示を確認しながら、
手動による細かい縦・横・奥行きの位置合わせを実施する。

======================================================================
１．ファイルの説明

１．１　scripts
１．１．１　gen_camtrans_matrix.py
２つのRealsenseの視覚内にマーカーを設置することで、変換行列をcsvで吐き出す。
計算には疑似逆行列を使用。
完全に座標が一致しないため、手動による調整が必要で

１．１．２　gen_marker.py
変換行列算出で使用するためのマーカーを生成。
マーカー０〜３を使用。

１．２　src
１．２．１ pointcloud_save.cpp
$rosrun t_camera pointcloud_save input:=/rs1/depth/color/transformed/points _prefix:=(保存するディレクトリパス)

cameras_launch.launchをローンチ後に実行することで.pcd, .plyファイルで点群を保存する。

１．２．２　realsense_transform.cpp
subscribeした点群データを変換行列を使って一つのカメラ座標に統合する。
