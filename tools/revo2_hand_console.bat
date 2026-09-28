@echo off
chcp 65001 >nul
title Revo2 Hand Control

:setup
ssh -o BatchMode=yes -o ConnectTimeout=6 bxi@172.16.10.47 "exit" >nul 2>&1
if not errorlevel 1 goto menu

echo.
echo   ============================================================
echo    FIRST RUN - setting up passwordless login
echo    robot : bxi@172.16.10.47
echo    passwd: ask your supervisor (typing below)
echo   ============================================================
echo.
if not exist "%USERPROFILE%\.ssh" mkdir "%USERPROFILE%\.ssh"
if not exist "%USERPROFILE%\.ssh\id_ed25519" ssh-keygen -t ed25519 -N "" -f "%USERPROFILE%\.ssh\id_ed25519" -q
type "%USERPROFILE%\.ssh\id_ed25519.pub" | ssh bxi@172.16.10.47 "mkdir -p ~/.ssh; chmod 700 ~/.ssh; touch ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys; cat >> ~/.ssh/authorized_keys; sort -u ~/.ssh/authorized_keys -o ~/.ssh/authorized_keys"
if errorlevel 1 goto setupfail
echo.
echo    [OK] passwordless login configured
goto menu

:setupfail
echo.
echo    [FAIL] cannot reach the robot. Check power and network.
pause
exit /b 1

:menu
echo.
echo   ============================================================
echo    Revo2 dexterous hand - gesture showcase
echo    robot: bxi@172.16.10.47
echo   ============================================================
echo.
echo    [1] Full showcase   ALL 15 acts             about 60s
echo    [2] Pick one act    enter act number 1-15
echo    [3] List acts       show all act names
echo    [4] Gesture lite    ripple + numbers + thumbs up
echo    [5] Wave fist       wave fist x3
echo    [6] Single fist     fist - hold - open
echo    [7] Hand info       read model / firmware   NO motion
echo    [0] Exit
echo.
set "sel="
set /p "sel=Select and press Enter: "

if "%sel%"=="1" goto L1
if "%sel%"=="2" goto L2
if "%sel%"=="3" goto L3
if "%sel%"=="4" goto L4
if "%sel%"=="5" goto L5
if "%sel%"=="6" goto L6
if "%sel%"=="7" goto L7
if "%sel%"=="0" goto END

echo   Invalid input, try again.
goto menu

:L1
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_showcase.py"
pause
goto menu

:L2
echo.
set "act="
set /p "act=Act number 1-15: "
if "%act%"=="" goto menu
echo   running act %act% ...
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_showcase.py --only %act%"
pause
goto menu

:L3
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_showcase.py --list"
pause
goto menu

:L4
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_gestures.py"
pause
goto menu

:L5
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_wave.py"
pause
goto menu

:L6
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_fist.py"
pause
goto menu

:L7
ssh -t bxi@172.16.10.47 "sudo bash /home/bxi/run.sh /home/bxi/hand_fist.py --read-only"
pause
goto menu

:END
exit /b 0
