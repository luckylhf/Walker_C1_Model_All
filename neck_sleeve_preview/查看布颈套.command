#!/bin/zsh
set -eu
if [[ -x /Library/Developer/CommandLineTools/usr/bin/otool ]]; then
  export DEVELOPER_DIR=/Library/Developer/CommandLineTools
fi
mjpython_bin="$(command -v mjpython)" || mjpython_bin="$HOME/miniconda3/envs/xSIM/bin/mjpython"
if [[ ! -x "$mjpython_bin" ]]; then
  print -u2 "找不到 mjpython。请在安装了 MuJoCo 的 Python 环境中运行本目录的启动脚本。"
  exit 1
fi
exec "$mjpython_bin" "${0:A:h}/view_neck.py"
