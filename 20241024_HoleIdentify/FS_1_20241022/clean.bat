:: 清零工程过程文件，用于打包

:: 删除VSCode编译过程文件及文件夹
rd /q /s build

:: 删除MDK编译过程文件夹
rd /q /s .\MDK-ARM\DebugConfig
rd /q /s .\MDK-ARM\Listings
rd /q /s .\MDK-ARM\Objects
rd /q /s .\MDK-ARM\RTE

:: 删除JLink相关文件
del /q /s .\MDK-ARM\*JLink*

:: 删除项目编译过程文件
del /s .\MDK-ARM\*.uvgui*
del /s .\MDK-ARM\*.dep
del /s .\MDK-ARM\*.bak

del /s .\MDK-ARM\*.axf
del /s .\MDK-ARM\*.htm
del /s .\MDK-ARM\*.lnp
del /s .\MDK-ARM\*.map
del /s .\MDK-ARM\*.sct
del /s .\MDK-ARM\*.dep
del /s .\MDK-ARM\*.crf
del /s .\MDK-ARM\*.iex
del /s .\MDK-ARM\*.o
del /s .\MDK-ARM\*.d

:: 删除 hex 和 bin文件 (自己决定是否需要删除)
del /s .\MDK-ARM\*.hex
del /s .\MDK-ARM\*.bin

:: 退出
exit
