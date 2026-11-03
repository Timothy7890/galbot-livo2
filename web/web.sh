#!/usr/bin/env bash
# 网页服务：7010 录制（server.py）
# 用法: web/web.sh start | stop | restart | status | autostart
WEB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$WEB_DIR/../run
mkdir -p "$RUN_DIR"

# 名称 脚本 端口
SERVERS=("web server.py 7010")

running() { [[ -f $RUN_DIR/$1.pid ]] && kill -0 "$(cat "$RUN_DIR/$1.pid")" 2> /dev/null; }

start_one() {
    local name=$1 script=$2 port=$3
    if running "$name"; then echo "$script 已在运行，端口 $port"; return 0; fi
    setsid /usr/bin/python3 "$WEB_DIR/$script" >> "$RUN_DIR/$name.log" 2>&1 < /dev/null &
    echo $! > "$RUN_DIR/$name.pid"
    sleep 1
    if running "$name"; then echo "$script 已启动，端口 $port"; else echo "$script 启动失败，见 run/$name.log"; return 1; fi
}

stop_one() {
    local name=$1 script=$2
    running "$name" && kill "$(cat "$RUN_DIR/$name.pid")" && echo "$script 已停止"
    rm -f "$RUN_DIR/$name.pid"
}

each() { local fn=$1 s; for s in "${SERVERS[@]}"; do $fn $s; done; }

case ${1:-} in
    start)   each start_one ;;
    stop)    each stop_one ;;
    restart) each stop_one; each start_one ;;
    status)
        for s in "${SERVERS[@]}"; do
            set -- $s
            if running "$1"; then echo "$2 运行中 (pid $(cat "$RUN_DIR/$1.pid"))，端口 $3"; else echo "$2 未运行"; fi
        done
        ;;
    autostart)
        # 写入 crontab @reboot（不需要 sudo），重复执行不会重复添加
        line="@reboot sleep 20 && $WEB_DIR/web.sh start"
        (crontab -l 2>/dev/null | grep -vF "$WEB_DIR/web.sh"; echo "$line") | crontab -
        echo "已设置开机自启: $line"
        ;;
    *)
        echo "用法: $0 start | stop | restart | status | autostart"
        exit 1
        ;;
esac
