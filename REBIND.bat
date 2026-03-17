@echo off
setlocal

if "%~1" == "" (
    echo Usage: Drag and drop a model file onto this script.
    echo It will rebind the model using the 'new_bind' animation pose.
    pause
    exit /b 1
)

set "INPUT=%~1"
set "BASENAME=%~dpn1"
set "EXT=%~x1"
set "OUTPUT=%BASENAME%_rebound%EXT%"

echo Rebinding: %INPUT%
echo Pose:      new_bind
echo Output:    %OUTPUT%
echo.

"%~dp0gsrebind.exe" "%INPUT%" "new_bind" "%OUTPUT%"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo REBIND FAILED with error code %ERRORLEVEL%.
    echo (Make sure the 'new_bind' animation exists and is static!)
) else (
    echo.
    echo REBIND SUCCESSFUL: %OUTPUT% generated.
)

pause
