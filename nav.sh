#!/usr/bin/env bash
# ============================================================================
#  Galbot G1 成熟地图内导航（不录制）：FAST-LIVO2 里程计 + 地图重定位 → 点选目标 → 规划前往
#
#  用法（一般直接用 7010 页面的「导航」）:
#    ./nav.sh start <录制目录或其 nav/ 目录>   启动导航（地图由 scripts/nav_map.py 生成在 <录制目录>/nav/）
#    ./nav.sh stop                             停止
#    ./nav.sh status
#  NAV_DRY_RUN=1 只算不动；NAV_V / NAV_W 限速（默认 0.3 m/s、0.6 rad/s）
#
#  与录制互斥（同一时间只能有一套 ROS / FAST-LIVO2）。底盘经 7011 遥控服务控制，需 web/web.sh 已启动。
#  运行状态在 run/nav_state、日志在 run/nav_logs/。
# ============================================================================

PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$PROJECT_DIR/run
STATE_FILE=$RUN_DIR/nav_state
PID_FILE=$RUN_DIR/nav_pids
LOCK_DIR=$RUN_DIR/nav_lock
LOG_DIR=$RUN_DIR/nav_logs
CONF=$RUN_DIR/nav_config
PKG_DIR=$PROJECT_DIR/catkin_ws/src/galbot_livo2
SDK_ENV=$PKG_DIR/scripts/sdk_env.sh

mkdir -p "$RUN_DIR"

PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/noetic/setup.bash
source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311

log() { echo "[$(date +%H:%M:%S)] $*"; }
state_get() { [[ -f $STATE_FILE ]] && grep -m1 "^$1=" "$STATE_FILE" | cut -d= -f2-; }
state_set() { touch "$STATE_FILE"; sed -i "/^$1=/d" "$STATE_FILE"; echo "$1=$2" >> "$STATE_FILE"; }
boot_time() { awk -v now="$(date +%s)" '{printf "%d", now - $1}' /proc/uptime; }

lock() {
    [[ -d $LOCK_DIR ]] && (($(stat -c %Y "$LOCK_DIR") < $(boot_time))) && rmdir "$LOCK_DIR"
    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        echo "另一个导航 start/stop 操作正在进行，请稍后再试" >&2
        exit 2
    fi
    trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT
}

launch() {
    local name=$1; shift
    setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    echo "$name $!" >> "$PID_FILE"
}
pid_of() { [[ -f $PID_FILE ]] && awk -v n="$1" '$1 == n {print $2}' "$PID_FILE"; }
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
ROS_TOOL="python3 $PROJECT_DIR/scripts/ros_tool.py"     # 一个进程加载全部参数 / 等多个话题
any_alive() {
    local _ p
    [[ -f $PID_FILE ]] || return 1
    while read -r _ p; do kill -0 "$p" 2>/dev/null && return 0; done < "$PID_FILE"
    return 1
}
# 断电重启或进程全部退出后残留的状态：清掉
clear_stale() {
    [[ -n $(state_get status) ]] || return 0
    if (($(state_get started) < $(boot_time))) || ! any_alive; then
        rm -f "$STATE_FILE" "$PID_FILE"
    fi
}
shutdown_all() {
    stop_proc navigator 5
    stop_proc fastlivo 20
    stop_proc bridge 15
    stop_proc roscore 10
    rm -f "$PID_FILE"
}

cmd_start() {
    local nav=${1:-}
    [[ -n $nav ]] || { echo "用法: $0 start <录制目录或 nav/ 目录>" >&2; exit 1; }
    nav=$(cd "$nav" 2>/dev/null && pwd) || { echo "目录不存在: $1" >&2; exit 1; }
    [[ -f $nav/meta.json ]] || nav=$nav/nav
    [[ -f $nav/meta.json && -f $nav/grid.npz && -f $nav/cloud.npz ]] || { echo "不是导航地图（缺 meta.json / grid.npz / cloud.npz）: $nav" >&2; exit 1; }
    lock
    clear_stale
    if [[ -n $(state_get status) ]]; then
        echo "导航已在运行（地图 $(state_get map)）" >&2
        exit 1
    fi
    if [[ $("$PROJECT_DIR/record.sh" status | head -1) != status=idle ]]; then
        echo "正在录制，导航与录制不能同时进行" >&2
        exit 1
    fi
    if timeout 1 bash -c '</dev/tcp/127.0.0.1/11311' 2> /dev/null; then
        echo "检测到已有 ROS 在运行，请先关闭" >&2
        exit 1
    fi
    if ! python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:7011/api/status', timeout=5)" 2>/dev/null; then
        echo "7011 底盘遥控服务未运行（web/web.sh start）" >&2
        exit 1
    fi
    local main_cam
    main_cam=$(awk '/main_camera:/ {print $2; exit}' "$PKG_DIR/config/g1_calib.yaml" 2>/dev/null)
    main_cam=${main_cam:-left_front}
    rm -rf "$LOG_DIR" "$CONF"
    mkdir -p "$LOG_DIR" "$CONF"
    export ROS_LOG_DIR="$LOG_DIR/ros"
    rm -f "$PID_FILE" "$STATE_FILE"
    state_set status starting
    state_set map "$nav"
    state_set started "$(date +%s)"
    log "导航地图: $nav"

    fail() {
        log "启动失败: $1（日志见 $LOG_DIR/）"
        shutdown_all
        rm -f "$STATE_FILE"
        exit 1
    }

    log "读取标定、启动 ROS（同时进行）"
    launch roscore roscore
    "$SDK_ENV" python3 "$PKG_DIR/scripts/gen_config.py" --camera "$main_cam" --out "$CONF" \
        > "$LOG_DIR/gen_config.log" 2>&1 || fail "生成标定失败（gen_config.py）"
    cp "$PKG_DIR/config/fast_livo2_g1.yaml" "$CONF/"
    # 导航不存地图（长时间运行也不累积点云）
    sed -i 's/^\([[:space:]]*pcd_save_en:[[:space:]]*\)true/\1false/' "$CONF/fast_livo2_g1.yaml"
    $ROS_TOOL load "$CONF/fast_livo2_g1.yaml" "$CONF/g1_calib.yaml" "$CONF/camera_g1.yaml:/laserMapping" \
        > "$LOG_DIR/params.log" 2>&1 || fail "roscore 未启动或加载参数失败"

    log "启动 Galbot SDK 桥接（只发主相机）"
    launch bridge "$SDK_ENV" rosrun galbot_livo2 sdk_bridge.py _publish_all_cameras:=false _publish_head_cameras:=false
    local miss
    miss=$($ROS_TOOL wait 30 /livox/lidar /livox/imu /galbot/camera/image) || fail "传感器没有数据：${miss:-超时}"

    log "启动 FAST-LIVO2"
    launch fastlivo rosrun fast_livo fastlivo_mapping __name:=laserMapping
    miss=$($ROS_TOOL wait 30 /aft_mapped_to_init) || fail "FAST-LIVO2 未输出里程计 /aft_mapped_to_init"

    log "启动导航程序$([[ ${NAV_DRY_RUN:-} == 1 ]] && echo "（只算不动）")"
    export EXPLORE_V=${NAV_V:-0.3} EXPLORE_W=${NAV_W:-0.6} NAV_DRY_RUN=${NAV_DRY_RUN:-}
    launch navigator python3 "$PROJECT_DIR/scripts/navigator.py" "_nav_dir:=$nav"
    state_set status running
    log "导航运行中，约 3 s 后自动重定位。停止: $0 stop"
}

cmd_stop() {
    lock
    clear_stale
    if [[ -z $(state_get status) ]]; then
        echo "导航没有在运行" >&2
        exit 1
    fi
    log "停止导航"
    shutdown_all
    rm -f "$STATE_FILE"
    log "已停止"
}

cmd_status() {
    clear_stale 2>/dev/null
    local status
    status=$(state_get status)
    if [[ -z $status ]]; then
        echo "status=idle"
        return
    fi
    echo "status=$status"
    echo "map=$(state_get map)"
    echo "elapsed_s=$(($(date +%s) - $(state_get started)))"
}

case ${1:-} in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)      echo "用法: $0 start <地图> | stop | status"; exit 1 ;;
esac
