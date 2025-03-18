@echo off
REM Set the destination IP address.
set ip_addr=192.168.1.35

echo Transferring files...

REM Use WinSCP to SFTP all files recursively while excluding the "target" directory.
REM You might need to adjust the -hostkey parameter based on your server’s key.
winscp.com /command ^
"open sftp://robot@%ip_addr%/ -hostkey=* " ^
"option exclude target" ^
"put -r . /home/robot/prog/cam/" ^
"exit"

REM Check if WinSCP encountered an error.
if errorlevel 1 (
echo An error occurred when transferring files.
exit /b 1
) else (
echo File transfer done.
)

echo Starting remote compilation...

REM Execute the remote command using plink.
plink robot@%ip_addr% "cd /home/robot/prog/cam; export PATH='/home/robot/.cargo/bin:$PATH'; cargo run"

if errorlevel 1 (
echo An error occurred during remote compilation.
exit /b 1
) else (
echo Project started.
)
pause