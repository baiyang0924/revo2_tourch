@echo off
chcp 936 >nul
title 灵巧手遥操

echo ============================================================
echo   灵巧手遥操：摄像头识别人手，机器人手实时跟随
echo ============================================================
echo.
echo   会弹出一个新窗口连机器人。
echo   提示密码时输入机器人密码。
echo   等新窗口出现「监听 9910」后，把手放到摄像头前。
echo.
pause

echo.
echo [1/2] 启动机器人端接收服务（新窗口）...
start "robot-server" ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_teleop_server.py --hand right"

echo [2/2] 等待服务就绪（8 秒）...
timeout /t 8 >nul

echo.
echo 启动本机摄像头识别（q 退出，+/- 调灵敏度）...
D://sim//.venv//Scripts//python.exe D://sim//hand_teleop_client.py

echo.
echo 已退出。请关闭 robot-server 窗口以释放手。
pause
