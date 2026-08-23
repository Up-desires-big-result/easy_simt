# =============================================================
# easy_simt 环境配置脚本（csh / tcsh 通用）
# 用法:  在仓库内任意目录下执行  source setup.csh
#        （脚本自当前目录向上查找仓库根，无需 cd 到根目录）
# 效果:  设置 PROJ_ROOT 为 easy_simt 仓库根目录
# 说明:  bsd-csh 无法在脚本内获取 source 时的路径参数，
#        故采用"当前目录向上查找"定位，要求 source 时位于仓库内
# =============================================================

set _setup_dir = "$cwd"
set _depth = 0

while ( 1 )
    if ( -f "$_setup_dir/setup.csh" && -d "$_setup_dir/top" ) goto setup_found
    set _parent = `dirname "$_setup_dir"`
    if ( "$_parent" == "$_setup_dir" ) goto setup_fail
    set _setup_dir = "$_parent"
    @ _depth ++
    if ( $_depth > 16 ) goto setup_fail
end

setup_found:
setenv PROJ_ROOT `cd "$_setup_dir" && pwd`
echo "PROJ_ROOT = $PROJ_ROOT"
goto setup_end

setup_fail:
echo "setup.csh: 无法定位仓库根目录，请在仓库内的目录下 source 本脚本"

setup_end:
unset _setup_dir _depth
if ( $?_parent ) unset _parent

# （后续环境变量在此追加）
