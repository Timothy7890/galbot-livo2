# galbot-livo2

在 Galbot G1 的 HPU（Jetson AGX Orin）上实时运行 FAST-LIVO2。传感器来自 Galbot SDK：
底盘 Mid-360 点云与内置 IMU、4 路环视相机（选一路给 FAST-LIVO2，其余原样发布供 3DGS 使用）。
不停用任何 Galbot 服务。

## 目录

- `catkin_ws/src/galbot_livo2`：SDK→ROS 桥接（`sdk_bridge.py`）、标定生成（`gen_config.py`）、参数与启动文件
- `catkin_ws/src/FAST-LIVO2`：官方代码，加了 `lidar_type: 8`（LIVOX_PC2，读 PointCloud2 的 `offset_nsec` 逐点时间）
- `catkin_ws/src/rpg_vikit`、`vision_opencv`：FAST-LIVO2 依赖；cv_bridge 从源码编译以使用 JetPack 的 OpenCV 4.5
- `thirdparty/Sophus`：FAST-LIVO2 要求的 a621ff 版本（`so2.cpp` 改了两行以适配新编译器）

## 安装与编译

```bash
sudo bash scripts/install_ros.sh          # ROS Noetic 及依赖，走 17897 代理
cd thirdparty/Sophus && mkdir -p build && cd build \
  && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PWD/../../install && make -j8 install
cd catkin_ws && P=$(cd .. && pwd) L=$(cd .. && pwd)/thirdparty/install/lib \
  && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release \
       -DSophus_DIR=$P/thirdparty/Sophus/build \
       "-DCMAKE_EXE_LINKER_FLAGS=-L$L -Wl,-rpath,$L" "-DCMAKE_SHARED_LINKER_FLAGS=-L$L -Wl,-rpath,$L" \
  && catkin build -j4
```

## 运行

```bash
source catkin_ws/devel/setup.bash
S=catkin_ws/src/galbot_livo2/scripts
$S/sdk_env.sh python3 $S/gen_config.py --camera left_front --overlay /tmp/overlay.jpg   # 外参变了才需要重跑
roslaunch galbot_livo2 mapping.launch
```

`--overlay` 把点云按外参投影到图像上，用来目检标定。`sdk_env.sh` 让 SDK 只加载系统里没有的 Galbot 库，避免顶替系统库。
