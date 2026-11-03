#!/usr/bin/env bash
# ============================================================================
#  Galbot G1 录制 + FAST-LIVO2 实时建图（传感器经 Galbot SDK 读取，不停用 Galbot 服务）
#
#  用法:
#    ./record.sh start              开始录制（手动或遥控开车）
#    ./record.sh start --explore    开始录制并自主探索：机器人自己走遍场地，回到起点后自动停止录制
#                                   （底盘经 7011 遥控服务控制，需 web/web.sh 已启动；
#                                     EXPLORE_V / EXPLORE_W / EXPLORE_MINUTES 可改限速与时长上限）
#    ./record.sh stop     停止录制，保存地图、轨迹、原始包
#    ./record.sh status   查看录制状态
#
#  每次录制输出到 $DATA_ROOT/<开始时间>/:
#    raw.bag         Mid-360 点云与 IMU、4 路环视相机 JPEG（供 3DGS 等离线处理）
#    map.pcd         FAST-LIVO2 彩色地图（降采样）   map_raw.pcd  未降采样
#    explore.json  explore_grid.png  自主探索的过程记录与 2D 栅格（--explore 时）
#    trajectory.csv  FAST-LIVO2 位姿（IMU 系，10 Hz 以上）   lidar_poses.txt  FAST-LIVO2 原始输出
#    config/  logs/  info.txt
# ============================================================================

PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA_ROOT=${GALBOT_DATA_ROOT:-$HOME/yx/Data/galbot_livo2}
RUN_DIR=$PROJECT_DIR/run              # 运行时状态（当前录制、各进程 PID）
STATE_FILE=$RUN_DIR/state
PID_FILE=$RUN_DIR/pids
LOCK_DIR=$RUN_DIR/lock
PKG_DIR=$PROJECT_DIR/catkin_ws/src/galbot_livo2
LIVO_PCD_DIR=$PROJECT_DIR/catkin_ws/src/FAST-LIVO2/Log/pcd
SDK_ENV=$PKG_DIR/scripts/sdk_env.sh
CAMERAS=(left_front right_front left_rear right_rear)

mkdir -p "$RUN_DIR" "$DATA_ROOT"

# ---------------------------------------------------------------- 环境
# 去掉 conda，避免 rospy 节点用到 conda 里的 python
PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/noetic/setup.bash
source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311

log() { echo "[$(date +%H:%M:%S)] $*"; }

state_get() { [[ -f $STATE_FILE ]] && grep -m1 "^$1=" "$STATE_FILE" | cut -d= -f2-; }
state_set() {
    touch "$STATE_FILE"
    sed -i "/^$1=/d" "$STATE_FILE"
    echo "$1=$2" >> "$STATE_FILE"
}

boot_time() { awk -v now="$(date +%s)" '{printf "%d", now - $1}' /proc/uptime; }

# 获取互斥锁，防止网页和命令行同时操作
lock() {
    # 断电时残留的锁（早于本次开机）直接清掉
    [[ -d $LOCK_DIR ]] && (($(stat -c %Y "$LOCK_DIR") < $(boot_time))) && rmdir "$LOCK_DIR"
    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        echo "另一个 start/stop 操作正在进行，请稍后再试" >&2
        exit 2
    fi
    trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT
}

# 以独立进程组启动，便于整组发送 SIGINT
launch() {
    local name=$1; shift
    setsid "$@" > "$SESSION/logs/$name.log" 2>&1 < /dev/null &
    echo "$name $!" >> "$PID_FILE"
}

pid_of() { [[ -f $PID_FILE ]] && awk -v n="$1" '$1 == n {print $2}' "$PID_FILE"; }

# 向进程组发 SIGINT，最多等待 $2 秒，超时则 SIGKILL
stop_proc() {
    local name=$1 timeout=${2:-15} pid
    pid=$(pid_of "$name")
    [[ -z $pid ]] && return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill -INT -- "-$pid" 2>/dev/null
    for ((i = 0; i < timeout * 2; i++)); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.5
    done
    log "  $name 未在 ${timeout}s 内退出，强制结束"
    kill -KILL -- "-$pid" 2>/dev/null
}

wait_topic() {
    local topic=$1 timeout=$2
    timeout "$timeout" rostopic echo -n 1 "$topic/header/stamp" > /dev/null 2>&1
}

is_recording() { [[ $(state_get status) == recording || $(state_get status) == stopping ]]; }

any_alive() {
    local _ p
    [[ -f $PID_FILE ]] || return 1
    while read -r _ p; do kill -0 "$p" 2>/dev/null && return 0; done < "$PID_FILE"
    return 1
}

# 断电/重启或进程异常退出后，状态文件还在但录制早已停止：标记该次录制为中断并清理状态
clear_stale() {
    [[ -n $(state_get status) ]] || return 0
    if (($(state_get started) < $(boot_time))) || { is_recording && ! any_alive; }; then
        local session started
        session=$(state_get session)
        started=$(state_get started)
        if [[ -d $session && ! -f $session/info.txt ]]; then
            cat > "$session/info.txt" << EOF
status=failed
start=$(date -d "@$started" '+%Y-%m-%d %H:%M:%S')
error=录制中断（断电、重启或进程退出），raw.bag 可能未正常关闭（raw.bag.active，可用 rosbag reindex 修复）
EOF
        fi
        rm -f "$STATE_FILE" "$PID_FILE"
        echo "上次录制异常结束（断电、重启或进程退出），已清理" >&2
    fi
}

# ---------------------------------------------------------------- stop（内部）
shutdown_all() {
    # 先停探索让底盘停下，再停 rosbag 保证包完整，再停 FAST-LIVO2 触发保存地图，最后停 SDK 桥接和 roscore
    stop_proc explorer 5
    stop_proc rosbag 30
    stop_proc live 5
    stop_proc trajectory 10
    stop_proc fastlivo 300
    stop_proc bridge 15
    stop_proc roscore 10
    rm -f "$PID_FILE"
}

# ---------------------------------------------------------------- start
cmd_start() {
    local explore=false
    [[ ${1:-} == --explore ]] && explore=true
    lock
    clear_stale

    if is_recording; then
        echo "已在录制中: $(state_get session)" >&2
        exit 1
    fi
    if rostopic list > /dev/null 2>&1; then
        echo "检测到已有 ROS 在运行（可能是手动启动的 mapping.launch），请先关闭" >&2
        exit 1
    fi
    if ! pgrep -f /data/galbot/bin/service_lidar_capture > /dev/null; then
        echo "Galbot 雷达服务 service_lidar_capture 未运行，请检查机器人是否正常开机" >&2
        exit 1
    fi
    if $explore && ! python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:7011/api/status', timeout=5)" 2>/dev/null; then
        echo "7011 底盘遥控服务未运行，无法自主探索（web/web.sh start）" >&2
        exit 1
    fi
    if [[ ! -f $PKG_DIR/config/g1_calib.yaml ]]; then
        echo "缺少标定配置，请先运行 scripts/gen_config.py（见 README）" >&2
        exit 1
    fi

    SESSION="$DATA_ROOT/$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$SESSION/logs" "$SESSION/config"
    cp "$PKG_DIR"/config/*.yaml "$SESSION/config/"
    export ROS_LOG_DIR="$SESSION/logs/ros"
    rm -f "$PID_FILE" "$STATE_FILE" "$LIVO_PCD_DIR"/*.pcd "$LIVO_PCD_DIR/lidar_poses.txt"

    state_set status starting
    state_set session "$SESSION"
    state_set started "$(date +%s)"
    state_set mode "$($explore && echo explore || echo manual)"
    log "录制目录: $SESSION"

    fail() {
        log "启动失败: $1（日志见 $SESSION/logs/）"
        shutdown_all
        state_set status idle
        echo "status=failed" >> "$SESSION/info.txt"
        echo "error=$1" >> "$SESSION/info.txt"
        rm -f "$STATE_FILE"
        exit 1
    }

    log "启动 roscore"
    launch roscore roscore
    for ((i = 0; i < 30; i++)); do rostopic list > /dev/null 2>&1 && break; sleep 0.5; done
    rostopic list > /dev/null 2>&1 || fail "roscore 未启动"
    rosparam load "$PKG_DIR/config/fast_livo2_g1.yaml"
    rosparam load "$PKG_DIR/config/g1_calib.yaml"
    rosparam load "$PKG_DIR/config/camera_g1.yaml" /laserMapping

    log "启动 Galbot SDK 桥接"
    launch bridge "$SDK_ENV" rosrun galbot_livo2 sdk_bridge.py
    wait_topic /livox/lidar 20 || fail "未收到雷达点云 /livox/lidar"
    wait_topic /livox/imu 10 || fail "未收到 IMU 数据 /livox/imu"
    wait_topic /galbot/camera/image 10 || fail "未收到主相机图像 /galbot/camera/image"

    log "启动 FAST-LIVO2 实时建图"
    launch fastlivo rosrun fast_livo fastlivo_mapping __name:=laserMapping
    wait_topic /aft_mapped_to_init 30 || fail "FAST-LIVO2 未输出里程计 /aft_mapped_to_init"

    log "启动网页实时地图推送"
    launch live nice -n 10 python3 "$PROJECT_DIR/scripts/live_map.py"

    log "启动轨迹记录"
    launch trajectory python3 "$PROJECT_DIR/scripts/record_odom.py" "_output_file:=$SESSION/trajectory.csv"

    log "启动原始数据录制"
    local topics=(/livox/lidar /livox/imu)
    for c in "${CAMERAS[@]}"; do topics+=("/galbot/$c/image/compressed"); done
    launch rosbag rosbag record -O "$SESSION/raw.bag" "${topics[@]}"

    if $explore; then
        log "启动自主探索"
        launch explorer python3 "$PROJECT_DIR/scripts/explorer.py" "_session:=$SESSION"
    fi

    state_set status recording
    log "录制中。停止请运行: $0 stop"
}

# ---------------------------------------------------------------- stop
cmd_stop() {
    lock
    clear_stale
    if ! is_recording; then
        echo "当前没有在录制" >&2
        exit 1
    fi
    SESSION=$(state_get session)
    local started end
    started=$(state_get started)
    state_set status stopping
    log "停止录制，正在保存（地图较大时需等待一段时间）..."

    shutdown_all
    end=$(date +%s)

    if [[ -f $LIVO_PCD_DIR/all_downsampled_points.pcd ]]; then
        mv "$LIVO_PCD_DIR/all_downsampled_points.pcd" "$SESSION/map.pcd"
        mv "$LIVO_PCD_DIR/all_raw_points.pcd" "$SESSION/map_raw.pcd" 2>/dev/null
    else
        log "警告: FAST-LIVO2 未生成地图文件"
    fi
    mv "$LIVO_PCD_DIR/lidar_poses.txt" "$SESSION/" 2>/dev/null

    local points traj_rows bag_size
    points=$(grep -a -m1 '^POINTS' "$SESSION/map.pcd" 2>/dev/null | awk '{print $2}')
    traj_rows=$(($(wc -l < "$SESSION/trajectory.csv" 2>/dev/null || echo 1) - 1))
    bag_size=$(du -h "$SESSION/raw.bag" 2>/dev/null | cut -f1)
    cat > "$SESSION/info.txt" << EOF
status=done
start=$(date -d "@$started" '+%Y-%m-%d %H:%M:%S')
end=$(date -d "@$end" '+%Y-%m-%d %H:%M:%S')
duration_s=$((end - started))
bag_size=${bag_size:-无}
map_points=${points:-0}
trajectory_poses=$traj_rows
mode=$(state_get mode)
EOF
    rm -f "$STATE_FILE"
    log "已保存到 $SESSION"
    cat "$SESSION/info.txt"
}

# ---------------------------------------------------------------- status
cmd_status() {
    local status session started now
    clear_stale 2>/dev/null
    status=$(state_get status)
    if [[ -z $status ]]; then
        echo "status=idle"
        return
    fi
    # 进程意外退出后状态文件残留：以 rosbag 进程是否存活为准
    if [[ $status == recording ]] && ! kill -0 "$(pid_of rosbag)" 2>/dev/null; then
        status=error
    fi
    session=$(state_get session)
    started=$(state_get started)
    now=$(date +%s)
    echo "status=$status"
    echo "session=$session"
    echo "elapsed_s=$((now - started))"
    echo "mode=$(state_get mode)"
    echo "size=$(du -sh "$session" 2>/dev/null | cut -f1)"
}

case ${1:-} in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)
        echo "用法: $0 start [--explore] | stop | status"
        exit 1
        ;;
esac
