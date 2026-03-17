@echo off
echo 正在清理 Keil 编译数据，请稍候...

:: 删除编译产生的列表文件和调试信息
del /s /q *.obj
del /s /q *.lst
del /s /q *.m51
del /s /q *.crf
del /s /q *.lnp
del /s /q *.d
del /s /q *.bak
del /s /q *.__i
del /s /q *.dep

:: 删除 IDE 界面状态文件 (包含历史打开的文档和窗口布局)
del /s /q *.uvgui.*
del /s /q *.uvguix.*

:: 删除输出文件 (如果你想保留 hex/bin，请注释掉下面两行)
del /s /q *.hex
del /s /q *.axf
del /s /q *.htm
del /s /q *.build_log.htm

:: 删除特定的文件夹 (如 Listing 和 Objects 文件夹中的内容)
:: 如果你的 Keil 设置了独立的文件夹，可以用下面的命令直接清空
:: rd /s /q Listings
:: rd /s /q Objects

echo 清理完成！
pause