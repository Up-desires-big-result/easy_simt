#!/bin/bash
# =============================================================
# easy_simt 环境配置脚本（bash 版）
# 用法:  source setup.sh        （路径任意，bash 会话内执行）
# 效果:  设置 PROJ_ROOT 为 easy_simt 仓库根目录；
#        third_party/ 内依赖就位时导出
#        PDK_ROOT / GPGPU_SIM_ROOT / OPENRAM_HOME / OPENRAM_TECH
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

# ------------------------------------------------------------
# third_party/：仓库内安装位（make deps 拉取，内容不入库）。
# 目录就位时 PDK_ROOT / GPGPU_SIM_ROOT / OPENRAM_HOME / OPENRAM_TECH 指向仓库内；否则回退
# 旧逻辑（PDK_ROOT 自动探测 ~/pdk）。手动 export 永远优先。
# ------------------------------------------------------------
if [ -n "$PROJ_ROOT" ]; then
    _tp="$PROJ_ROOT/third_party"

    # orfs 已克隆而软链缺失时补软链（非网络操作，幂等）
    if [ -d "$_tp/orfs/flow/platforms/nangate45" ] && [ ! -e "$_tp/nangate45" ]; then
        ln -s orfs/flow/platforms/nangate45 "$_tp/nangate45"
    fi

    if [ -z "$PDK_ROOT" ]; then
        if [ -d "$_tp/nangate45" ]; then
            export PDK_ROOT="$_tp"
            echo "PDK_ROOT  = $PDK_ROOT（third_party）"
        elif [ -d "$HOME/pdk/nangate45" ]; then
            export PDK_ROOT="$HOME/pdk"
            echo "PDK_ROOT  = $PDK_ROOT（自动探测）"
        fi
    fi

    if [ -z "$GPGPU_SIM_ROOT" ] && [ -d "$_tp/gpgpu-sim" ]; then
        export GPGPU_SIM_ROOT="$_tp/gpgpu-sim"
        echo "GPGPU_SIM_ROOT = $GPGPU_SIM_ROOT（third_party）"
    fi

    if [ -z "$OPENRAM_HOME" ] && [ -f "$_tp/openram/sram_compiler.py" ]; then
        export OPENRAM_HOME="$_tp/openram/compiler"
        echo "OPENRAM_HOME = $OPENRAM_HOME（third_party）"
    fi

    # OpenRAM 需 OPENRAM_HOME（compiler/）与 OPENRAM_TECH（technology/），
    # 并把 OPENRAM_HOME 加进 PYTHONPATH（均幂等，手动 export 优先）
    if [ -z "$OPENRAM_TECH" ] && [ -d "$_tp/openram/technology" ]; then
        export OPENRAM_TECH="$_tp/openram/technology"
        echo "OPENRAM_TECH = $OPENRAM_TECH（third_party）"
    fi
    if [ -n "$OPENRAM_HOME" ]; then
        case ":$PYTHONPATH:" in
            *":$OPENRAM_HOME:"*) ;;
            *) PYTHONPATH="$OPENRAM_HOME${PYTHONPATH:+:$PYTHONPATH}"
               export PYTHONPATH ;;
        esac
    fi

    unset _tp
fi
