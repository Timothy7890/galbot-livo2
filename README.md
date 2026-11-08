# galbot-livo2

在 Galbot G1 的 HPU（Jetson AGX Orin）上实时运行 FAST-LIVO2。传感器来自 Galbot SDK：
底盘 Mid-360 点云与内置 IMU、4 路环视相机（选一路给 FAST-LIVO2，其余原样发布供 3DGS 使用）、
头部双目相机（只录包供 3DGS 使用）。不停用任何 Galbot 服务。

环视相机、头部相机和手臂都装在躯干上，Mid-360 与 IMU 在底盘上：躯干升降（7011 页面）或头部转动后相机-雷达外参会变。
`record.sh` 每次开始录制都会从 SDK 现读外参写进录制目录的 `config/`，FAST-LIVO2 用的就是这份；录制中关节变化会记警告。

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
$S/sdk_env.sh python3 $S/gen_config.py --camera left_front --overlay /tmp/overlay.jpg   # 手动启动前、外参变了要重跑
roslaunch galbot_livo2 mapping.launch
```

`gen_config.py` 默认写包内 `config/`（`--out` 可改），同时写 `sensors_all.json`：6 路相机的内参、在底盘 / 雷达 / IMU 系下的位姿和 leg / head 关节角
（SDK 给的头部相机外参朝向与图像不符，离线需重新标定）。`record.sh` 不用包内这份，而是每次以 `--out <录制目录>/config` 重新生成。
`--overlay` 把点云按外参投影到图像上，用来目检标定。`sdk_env.sh` 让 SDK 只加载系统里没有的 Galbot 库，避免顶替系统库。

## 录制网页（7010）

```bash
web/web.sh start       # 启动 7010 录制与 7011 底盘遥控；web/web.sh autostart 设为开机自启（已设置）
```

浏览器打开 `http://<HPU>:7010`（本机经 ssh 转发为 17710）：开始/停止录制、实时俯视地图（1.6 m 以下的点，按高度着色）、主相机画面、历史记录。
命令行等价操作：`./record.sh start | stop | status`。每次录制保存在 `~/yx/Data/galbot_livo2/<开始时间>/`：

- `raw.bag`：点云、IMU、4 路环视（10 Hz）与 2 路头部（15 Hz）相机 JPEG、leg / head 关节角 `/galbot/joint_states`（2 Hz），约 90 GB/小时
- `config/`：开始录制时从 SDK 现读的标定（`g1_calib.yaml`、`camera_g1.yaml`、`sensors_all.json`）与 FAST-LIVO2 参数
- `info.txt`：`joints_moved=yes` 表示录制中躯干 / 头部动过，之后的相机外参与 `config/` 不符（见 `logs/bridge.log`）
- `map.pcd`（5 cm 降采样彩色地图）、`map_raw.pcd`、`trajectory.csv`、`lidar_poses.txt`、`logs/`（`bridge.log` 每 10 s 记各路帧数与丢帧）

前端改动后在有 node 的机器上 `cd web/frontend && npm install && npm run build`，把 `web/dist` 一起提交。

## 底盘遥控网页（7011）

浏览器打开 `http://<HPU>:7011`（本机经 ssh 转发为 17711）。按住按钮或键盘 W/S 前后、A/D 横移、Q/E 转向，空格急停。

- 避障（默认开）：`web/safety.py` 用 Mid-360（0.05–1.6 m 高的点）与 8 个超声波，按运动方向 0.25/0.30 m 内停车，更远时限速为按 0.3 m/s² 能在停车距离处停下的速度（0.4 m/s 约在 0.5 m 处开始减速），退离障碍不受限；可关闭（需确认）。Galbot 自带避障 `navigate_with_velocity` 需先有 Galbot 地图并定位，否则返回 WAIT_INITIALIZED
- 底盘 600 x 600 mm（投影圆 625 mm）。手臂等伸出底盘时，在四周 0.8 m 内空旷处点「标定车身轮廓」，结果存 `catkin_ws/src/galbot_livo2/config/self_mask.json`
- 速度以 `duration=0` 10 Hz 持续下发（SDK 的 `set_base_velocity` 带 duration 会阻塞整个时长且不释放 GIL，实测 0.5 s → 卡住整个进程 520 ms）；看门狗 `web/base_watchdog.py` 读心跳 `/dev/shm/galbot_teleop_heartbeat`，7011 崩溃或卡住 0.6 s 以上且在动就 `stop_base`；输出按加减速限制平顺变化；松手立即减速停车，0.5 s 没收到新指令（切走页面、断线）开始减速，1 s 强制 `stop_base`
- 急停会锁定，解除前忽略一切速度指令；默认限速 0.5 m/s、1.0 rad/s（`web/teleop_server.py`）
- 躯干高度：低位 / 中位 / 站立三档（`LEG_POSES`，leg 关节组，0.1 rad/s）。录制中、底盘运动中不可用，升降期间底盘指令被拒绝；急停时停止关节轨迹并停在当前位置（不用 `stop_controller`，否则躯干可能下坠）。升降后需重新标定车身轮廓（手臂相对 Mid-360 的位置变了），相机外参在下次开始录制时自动更新

## 自主建图（录制中在 7010 点「进入自主建图」）

录制期间可随时进入 / 退出自主建图：机器人自己走遍封闭场地。
进入后：原地环视一圈（本次录制第一次进入时）→ 反复选最近、最大的边界（已知可通行与未知的交界）并前往
→ 没有边界或超过时长上限后回到录制起点 → 退出自主建图并倒计时 2 分钟，没人点「取消自动停止」就停止录制。

- 自主建图程序 `scripts/explorer.py` 随每次录制启动，平时只建 2D 栅格、不发速度；中途进入时已手动走过的区域都已知
- 7011 上有人按方向键（人工接管）立即退出自主建图；7011 急停期间暂停，解除后继续
- `scripts/explore_grid.py`：FAST-LIVO2 位姿 + Mid-360 射线投射的 2D 占据栅格（10 cm），边界检测，Dijkstra 规划（障碍膨胀车身半径 + 0.1 m，路线尽量走中间）后拉直成几段直线；目标至少离机器人 1 m，选定后一直沿用到到达或附近边界已看清，路线没被挡住就不换（避免目标来回跳、走 S 形）；到达后仍看不清的边界不再去
- FAST-LIVO2 里程计改为带数据时间戳（原为处理完成时刻）；探索时位姿比数据落后超过 0.8 s（处理不过来）就暂停等它追上，点云与同一时刻的位姿配对再建栅格 / 实时地图；朝向变化与 IMU 陀螺仪不一致（3 s 内差 40° 以上）判定定位跑飞并退出；30 s 内实际转了 540° 以上却几乎没走判定原地打转，换目标，3 次退出
- FAST-LIVO2 参数：`point_filter_num: 2`、局部地图滑动窗口 ±20 m（原来全点、全图，探索 8 分钟后单帧超 100 ms，位姿落后到 4.5 s）
- 跟随用纯追踪（前视点不越过下一个拐点，不切内角），车头朝前进方向：偏离超过 45° 先原地转向、小偏离边走边转；此处转不开（贴着障碍）才朝远离最近障碍的方向平移挪出去（卡住后的后退同样远离最近障碍）。规划把拐点和目标尽量放在离障碍 ≥ 0.6 m、能原地转身的地方；卡住（8 s 几乎没动或 4 s 一直被避障挡住）则后退、拉黑目标、换下一个；位姿中断暂停，位姿跳变（定位跑飞）退出
- 速度经 7011 遥控服务下发，避障、加减速、急停都在那里
- 默认限速 0.4 m/s、0.8 rad/s，每次进入时长上限 30 分钟：`EXPLORE_V` / `EXPLORE_W` / `EXPLORE_MINUTES` 环境变量可改；
  `EXPLORE_DRY_RUN=1 ./record.sh start` 只算不动；`./record.sh start --explore` 开始录制并立即进入
- 录制目录另存 `explore.json`（过程记录）与 `explore_grid.png`（2D 栅格）
