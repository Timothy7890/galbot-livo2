#!/usr/bin/env bash
# ============================================================================
#  Galbot G1 成熟地图内导航（不录制）：FAST-LIVO2 里程计 + 地图重定位 → 点选目标 → 规划前往
#
#  用法（一般直接用 7010 页面的「导航」）:
#    ./nav.sh start <录制目录或其 nav/ 目录> [--no-vision] [--dry-run]
#                                              启动导航（地图由 scripts/nav_map.py 生成在 <录制目录>/nav/）
#    ./nav.sh stop                             停止
#    ./nav.sh status
#  --no-vision  FAST-LIVO2 只用雷达 + IMU（不开相机）：不必从 SDK 现读相机外参，启动更快、占用更低
#  --dry-run    只算不动（同 NAV_DRY_RUN=1）；NAV_V / NAV_W 限速（默认 0.3 m/s、0.6 rad/s，网页可调线速度）
#  启动进度写在 run/nav_state 的 step（网页显示），各步耗时记在 run/nav_logs/start_timing.log
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
# status 只读状态文件，不需要 ROS 环境：网页每 0.5–2 s 查一次，省掉两次 source（机器忙时各要零点几秒）
if [[ ${1:-} != status ]]; then
    source /opt/ros/noetic/setup.bash
    source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
fi
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
    # 启动中（刚写状态、进程还没起来）不按"进程都不在"判残留，否则网页轮询 status 会把正在启动的状态清掉
    # （启动脚本被杀掉的话状态会一直是 starting：超过 200 s、又没有启动锁，才算残留）
    local st age
    st=$(state_get status)
    age=$(($(date +%s) - $(state_get started)))
    if (($(state_get started) < $(boot_time))) || { [[ $st == running ]] && ! any_alive; } \
        || { [[ $st == starting ]] && ((age > 200)) && [[ ! -d $LOCK_DIR ]]; }; then
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

# 启动进度：网页（7010 导航页）轮询 status 显示到第几步；各步耗时记在 nav_logs/start_timing.log
START_STEPS=4
STEP_T0=0
STEP_NAME=""
step() {
    local i=$1 text=$2 now
    now=$(date +%s.%N)
    if [[ -n $STEP_NAME ]]; then
        printf '%-28s %5.1f s\n' "$STEP_NAME" "$(awk -v a="$now" -v b="$STEP_T0" 'BEGIN {print a - b}')" >> "$LOG_DIR/start_timing.log"
    fi
    STEP_T0=$now
    STEP_NAME=$text
    [[ $i -gt 0 ]] || return 0
    state_set step "$i/$START_STEPS $text"
    state_set step_t "$(date +%s)"
    log "[$i/$START_STEPS] $text"
}
master_up() { timeout 1 bash -c '</dev/tcp/127.0.0.1/11311' 2> /dev/null; }

cmd_start() {
    local nav="" vision=true dry=false a
    [[ ${NAV_DRY_RUN:-} == 1 ]] && dry=true
    for a in "$@"; do
        case $a in
            --no-vision) vision=false ;;
            --dry-run)   dry=true ;;
            -*)          echo "未知选项: $a" >&2; exit 1 ;;
            *)           nav=$a ;;
        esac
    done
    [[ -n $nav ]] || { echo "用法: $0 start <录制目录或 nav/ 目录> [--no-vision] [--dry-run]" >&2; exit 1; }
    local arg=$nav
    nav=$(cd "$nav" 2>/dev/null && pwd) || { echo "目录不存在: $arg" >&2; exit 1; }
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
    if master_up; then
        echo "检测到已有 ROS 在运行，请先关闭" >&2
        exit 1
    fi
    if ! timeout 2 bash -c '</dev/tcp/127.0.0.1/7011' 2> /dev/null; then
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
    state_set vision "$vision"
    state_set dry_run "$dry"
    state_set started "$(date +%s)"
    log "导航地图: $nav（视觉 $($vision && echo 开 || echo 关)$($dry && echo "，只算不动")）"

    fail() {
        log "启动失败: $1（日志见 $LOG_DIR/）"
        shutdown_all
        rm -f "$STATE_FILE"
        exit 1
    }

    # 雷达与 IMU 都在底盘上，二者之间、与底盘之间的外参是固定的；相机在躯干上，外参随躯干升降变。
    # 开视觉：标定由 SDK 桥接自己顺手读出来（~calib_out），不另起 gen_config.py：SDK 初始化一次要好几秒，
    #         两个进程同时初始化还互相拖慢（实测并行并不省时间）。
    # 关视觉：FAST-LIVO2 只用雷达 + IMU，沿用建图那次录制的标定（没有就用包内的），不读 SDK、不开相机。
    step 1 "$($vision && echo "读取标定、启动 ROS 与传感器" || echo "启动 ROS 与传感器（不开相机）")"
    launch roscore roscore
    local calib_arg=()
    if $vision; then
        calib_arg=("_calib_out:=$CONF")
    else
        local src
        src=$(dirname "$nav")/config
        [[ -f $src/g1_calib.yaml && -f $src/camera_g1.yaml ]] || src=$PKG_DIR/config
        [[ -f $src/g1_calib.yaml && -f $src/camera_g1.yaml ]] || fail "没有可用的标定（$src），请先开视觉启动一次或运行 gen_config.py"
        cp "$src/g1_calib.yaml" "$src/camera_g1.yaml" "$CONF/"
        echo "不开视觉：沿用 $src 的标定（雷达-IMU-底盘外参固定）" > "$LOG_DIR/gen_config.log"
    fi
    local i
    for ((i = 0; i < 150; i++)); do master_up && break; sleep 0.1; done
    master_up || fail "roscore 未启动"
    launch bridge "$SDK_ENV" rosrun galbot_livo2 sdk_bridge.py _publish_all_cameras:=false _publish_head_cameras:=false \
        "_main_camera:=$main_cam" "_publish_main_camera:=$vision" "${calib_arg[@]}"
    if $vision; then
        for ((i = 0; i < 300; i++)); do
            [[ -e $CONF/.calib_ok ]] && break
            [[ -e $CONF/.calib_fail ]] && fail "生成标定失败：$(cat "$CONF/.calib_fail")"
            kill -0 "$(pid_of bridge)" 2>/dev/null || fail "SDK 桥接退出了（见 nav_logs/bridge.log）"
            sleep 0.1
        done
        [[ -e $CONF/.calib_ok ]] || fail "30 s 内没读到标定（Galbot SDK 未就绪？见 nav_logs/bridge.log）"
    fi
    cp "$PKG_DIR/config/fast_livo2_g1.yaml" "$CONF/"
    # 导航不存地图（长时间运行也不累积点云）
    sed -i 's/^\([[:space:]]*pcd_save_en:[[:space:]]*\)true/\1false/' "$CONF/fast_livo2_g1.yaml"
    $vision || sed -i 's/^\([[:space:]]*img_en:[[:space:]]*\)1/\10/' "$CONF/fast_livo2_g1.yaml"
    $ROS_TOOL load "$CONF/fast_livo2_g1.yaml" "$CONF/g1_calib.yaml" "$CONF/camera_g1.yaml:/laserMapping" \
        > "$LOG_DIR/params.log" 2>&1 || fail "加载参数失败（见 nav_logs/params.log）"

    # 导航程序只等参数：这时就启动，载入地图（约 1 s）与等传感器同时进行
    export EXPLORE_V=${NAV_V:-0.3} EXPLORE_W=${NAV_W:-0.6}
    NAV_DRY_RUN=$($dry && echo 1) launch navigator python3 "$PROJECT_DIR/scripts/navigator.py" "_nav_dir:=$nav"

    step 2 "等待雷达、IMU$($vision && echo "、相机")数据"
    local miss topics=(/livox/lidar /livox/imu)
    $vision && topics+=(/galbot/camera/image)
    miss=$($ROS_TOOL wait 30 "${topics[@]}") || fail "传感器没有数据：${miss:-超时}"

    step 3 "启动 FAST-LIVO2$($vision || echo "（雷达 + IMU）")"
    launch fastlivo rosrun fast_livo fastlivo_mapping __name:=laserMapping
    miss=$($ROS_TOOL wait 30 /aft_mapped_to_init) || fail "FAST-LIVO2 未输出里程计 /aft_mapped_to_init"

    step 4 "确认导航程序就绪"
    for ((i = 0; i < 100; i++)); do
        kill -0 "$(pid_of navigator)" 2>/dev/null || fail "导航程序退出了（见 nav_logs/navigator.log）"
        python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:7014/state', timeout=1)" 2>/dev/null && break
        sleep 0.2
    done
    ((i < 100)) || fail "导航程序没有就绪（见 nav_logs/navigator.log）"

    step 0 ""
    printf '%-28s %5d s\n' "合计" "$(($(date +%s) - $(state_get started)))" >> "$LOG_DIR/start_timing.log"
    sed -i '/^step/d' "$STATE_FILE"
    state_set status running
    log "导航运行中，约 1 s 后自动重定位。停止: $0 stop"
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
    local now
    now=$(date +%s)
    echo "status=$status"
    echo "map=$(state_get map)"
    echo "vision=$(state_get vision)"
    echo "dry_run=$(state_get dry_run)"
    echo "elapsed_s=$((now - $(state_get started)))"
    if [[ $status == starting ]]; then
        echo "step=$(state_get step)"
        local st
        st=$(state_get step_t)
        echo "step_elapsed_s=$((now - ${st:-$now}))"
    fi
}

case ${1:-} in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)      echo "用法: $0 start <地图> [--no-vision] [--dry-run] | stop | status"; exit 1 ;;
esac
