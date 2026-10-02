# Tennis Bot — ROS 2 Otonom Mobil Robot

ROS 2 ve Gazebo Sim kullanılarak geliştirilen, tenis toplarını algılayıp otonom olarak bu hedeflere ulaşmayı amaçlayan diferansiyel sürüşlü mobil robot projesi.

## Özellikler

* URDF/Xacro ile robot modelleme
* Gazebo Sim üzerinde robot ve ortam simülasyonu
* RGB-D kamera ile OpenCV tabanlı tenis topu algılama
* Depth verisi ve TF2 ile 3D konum hesaplama
* Lidar ve IMU verilerinin EKF ile birleştirilmesi
* SLAM Toolbox ile haritalama
* AMCL ile robot konumlandırma
* Nav2 ile otonom navigasyon
* `ros2_control` ile tekerlek kontrolü

## Kullanılan Teknolojiler

ROS 2 Humble, Python, RViz2, OpenCV, TF2, Nav2, SLAM Toolbox, AMCL, EKF, `ros2_control`, Linux ve WSL2.

## Proje Mimarisi

```text
RGB-D Kamera → OpenCV → 3D Top Konumu → TF2
                                          ↓
Lidar + Odometry + IMU → SLAM / EKF → Localization
                                          ↓
                               Top Hedefi → Nav2
                                          ↓
                               diff_drive_controller
```

## Kurulum ve Çalıştırma

```bash
cd ~/tennis_bot_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source install/setup.bash
ros2 launch tennis_bot_bringup tennis_bot.launch.py
```

## Geliştirme Alanları

* Top algılama doğruluğunu ve kararlılığını artırma
* Hedef seçimi ve navigasyon davranışlarını iyileştirme
* Farklı engel senaryolarında testler yapma
* Gerçek robot üzerinde çalışmayı değerlendirme
