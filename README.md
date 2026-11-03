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

## 录制网页（7010）

```bash
web/web.sh start       # 启动 7010 录制与 7011 底盘遥控；web/web.sh autostart 设为开机自启（已设置）
```

浏览器打开 `http://<HPU>:7010`（本机经 ssh 转发为 17710）：开始/停止录制、实时俯视地图（1.6 m 以下的点，按高度着色）、主相机画面、历史记录。
命令行等价操作：`./record.sh start | stop | status`。每次录制保存在 `~/yx/Data/galbot_livo2/<开始时间>/`：

- `raw.bag`：点云、IMU、4 路环视相机 JPEG，约 38 GB/小时
- `map.pcd`（5 cm 降采样彩色地图）、`map_raw.pcd`、`trajectory.csv`、`lidar_poses.txt`、`config/`、`logs/`、`info.txt`

前端改动后在有 node 的机器上 `cd web/frontend && npm install && npm run build`，把 `web/dist` 一起提交。

## 底盘遥控网页（7011）

浏览器打开 `http://<HPU>:7011`（本机经 ssh 转发为 17711）。按住按钮或键盘 W/S 前后、A/D 横移、Q/E 转向，空格急停。

- 避障（默认开）：`web/safety.py` 用 Mid-360（0.05–1.6 m 高的点）与 8 个超声波，按运动方向在 0.8/0.6 m 内减速、0.25/0.30 m 内停车，退离障碍不受限；可关闭（需确认）。Galbot 自带避障 `navigate_with_velocity` 需先有 Galbot 地图并定位，否则返回 WAIT_INITIALIZED
- 底盘 600 x 600 mm（投影圆 625 mm）。手臂等伸出底盘时，在四周 0.8 m 内空旷处点「标定车身轮廓」，结果存 `catkin_ws/src/galbot_livo2/config/self_mask.json`
- 每条指令只持续 0.5 s；服务端 0.35 s 内没收到新指令（松手、切走页面、断线）立即 `stop_base`
- 急停会锁定，解除前忽略一切速度指令；默认限速 0.5 m/s、1.0 rad/s（`web/teleop_server.py`）

## 自主建图（7010「自主建图」或 `./record.sh start --explore`）

从零开始自己走遍封闭场地：原地环视一圈 → 反复选最近、最大的边界（已知可通行与未知的交界）并前往 → 没有边界或超过时长上限后回到起点 → 自动停止录制。

- `scripts/explore_grid.py`：FAST-LIVO2 位姿 + Mid-360 射线投射的 2D 占据栅格（10 cm），边界检测，Dijkstra 规划（障碍膨胀车身半径 + 0.1 m，路线尽量走中间）
- `scripts/explorer.py`：纯追踪跟随（车头尽量朝前进方向），卡住（8 s 几乎没动或 4 s 一直被避障挡住）则后退、拉黑目标、换下一个；位姿中断暂停，位姿跳变（定位跑飞）停止探索
- 速度经 7011 遥控服务下发，避障、加减速、急停都在那里；7011 急停期间探索暂停，解除后继续
- 默认限速 0.15 m/s、0.4 rad/s，时长上限 30 分钟：`EXPLORE_V` / `EXPLORE_W` / `EXPLORE_MINUTES` 环境变量可改；`EXPLORE_DRY_RUN=1` 只算不动
- 录制目录另存 `explore.json`（过程记录）与 `explore_grid.png`（2D 栅格）
