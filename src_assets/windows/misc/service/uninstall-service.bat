@echo off
setlocal enabledelayedexpansion

set "SERVICE_CONFIG_DIR=%LOCALAPPDATA%\LizardByte\Sunshine"
set "SERVICE_CONFIG_FILE=%SERVICE_CONFIG_DIR%\service_start_type.txt"

rem Save the current service start type to a file if the service exists
sc qc SunshineService >nul 2>&1
if %ERRORLEVEL%==0 (
    if not exist "%SERVICE_CONFIG_DIR%\" mkdir "%SERVICE_CONFIG_DIR%\"

    rem Get the start type
    for /f "tokens=3" %%i in ('sc qc SunshineService ^| findstr /C:"START_TYPE"') do (
        set "CURRENT_START_TYPE=%%i"
    )

    rem Set the content to write
    if "!CURRENT_START_TYPE!"=="2" (
        sc qc SunshineService | findstr /C:"(DELAYED)" >nul
        if !ERRORLEVEL!==0 (
            set "CONTENT=2-delayed"
        ) else (
            set "CONTENT=2"
        )
    ) else if "!CURRENT_START_TYPE!" NEQ "" (
        set "CONTENT=!CURRENT_START_TYPE!"
    ) else (
        set "CONTENT=unknown"
    )

    rem Write content to file
    echo !CONTENT!> "%SERVICE_CONFIG_FILE%"
)

call :stop_and_delete_service sunshinesvc
call :stop_and_delete_service SunshineService
exit /b 0

:stop_and_delete_service
set "_SVC_NAME=%~1"

sc query "%_SVC_NAME%" >nul 2>&1
if not %ERRORLEVEL%==0 exit /b 0

sc stop "%_SVC_NAME%" >nul 2>&1

set /a wait_count=0
:wait_loop
set /a wait_count+=1
sc query "%_SVC_NAME%" | findstr /C:"STATE" | findstr /C:"STOPPED" >nul
if %ERRORLEVEL%==0 goto delete_service
if %wait_count% GEQ 20 goto force_kill
ping -n 2 127.0.0.1 >nul
goto wait_loop

:force_kill
taskkill /F /FI "SERVICES eq %_SVC_NAME%" >nul 2>&1

:delete_service
sc delete "%_SVC_NAME%" >nul 2>&1
exit /b 0
