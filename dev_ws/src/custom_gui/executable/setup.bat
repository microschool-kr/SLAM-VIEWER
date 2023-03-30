call c:\opt\ros\noetic\x64\setup.bat
call c:\dev_ws\devel\setup.bat
for /f "tokens=2 delims=:" %%b in ('ipconfig^|find /i "ipv4"') do set fsip=%%b
set fsip=%fsip: =%
set ROS_HOSTNAME=%fsip%
set ROS_MASTER_URI=http://%fsip%:11311