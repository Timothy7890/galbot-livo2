#!/bin/bash
# 在 Galbot SDK 可加载的环境里运行命令：sdk_env.sh <命令...>
# SDK 依赖的库都在 /data/galbot/lib，但其中很多和系统库同名（boost、pcl、python、blas 等），
# 整个目录加进 LD_LIBRARY_PATH 会顶替系统库（例如 numpy 会崩）。这里只链接系统里没有的那些
SDK=${GALBOT_SDK:-$HOME/galbot_sdk_1.9.1}
LIB=/tmp/galbot_sdk_lib-$(id -u)
mkdir -p "$LIB"
# 上千个库逐个判断、链接要 1 s：链接过、且 /data/galbot/lib 之后没变过就跳过（/tmp 重启会清，届时自动重做）
if [ ! -e "$LIB/.done" ] || [ /data/galbot/lib -nt "$LIB/.done" ]; then
    for f in /data/galbot/lib/*.so*; do
        n=${f##*/}
        [ -e /usr/lib/aarch64-linux-gnu/$n ] || [ -e /lib/aarch64-linux-gnu/$n ] || ln -sfn "$f" "$LIB/$n"
    done
    touch "$LIB/.done"
fi
export LD_LIBRARY_PATH=$SDK/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}:$LIB
export PYTHONUNBUFFERED=1  # roslaunch 下 stdout 是管道，不关缓冲日志就看不到
export PYTHONPATH=$SDK/lib/python${PYTHONPATH:+:$PYTHONPATH}
exec "$@"
