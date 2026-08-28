#!/bin/bash
# =============================================================
# easy_simt 环境配置脚本（bash 版）
# 用法:  source setup.sh        （路径任意，bash 会话内执行）
# 效果:  设置 PROJ_ROOT 为 easy_simt 仓库根目录
# 说明:  csh/tcsh 会话请改用  source setup.csh
# =============================================================

# bash 可直接定位脚本自身位置（source 时 BASH_SOURCE 有效）
_setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

if [ -f "$_setup_dir/setup.sh" ] && [ -d "$_setup_dir/top" ]; then
    export PROJ_ROOT="$_setup_dir"
    echo "PROJ_ROOT = $PROJ_ROOT"
else
    # 兜底：自当前目录向上查找含 setup.sh 与 top/ 的仓库根
    _d="$PWD"
    _depth=0
    while [ ! -f "$_d/setup.sh" ] || [ ! -d "$_d/top" ]; do
        _p="$(dirname "$_d")"
        if [ "$_p" = "$_d" ] || [ "$_depth" -gt 16 ]; then
            echo "setup.sh: 无法定位仓库根目录，请在仓库内的目录下 source 本脚本" >&2
            _d=""
            break
        fi
        _d="$_p"
        _depth=$((_depth + 1))
    done
    if [ -n "$_d" ]; then
        export PROJ_ROOT="$_d"
        echo "PROJ_ROOT = $PROJ_ROOT"
    fi
fi

unset _setup_dir _d _p _depth

# PDK_ROOT：nangate45 工艺库目录（仓库外，见 README「工艺库（nangate45）」）。
# 未设置时自动探测 ~/pdk（要求其中含 nangate45/）；也可在 source 前自行
# export PDK_ROOT 覆盖。
if [ -z "$PDK_ROOT" ]; then
    if [ -d "$HOME/pdk/nangate45" ]; then
        export PDK_ROOT="$HOME/pdk"
        echo "PDK_ROOT  = $PDK_ROOT（自动探测）"
    fi
fi
