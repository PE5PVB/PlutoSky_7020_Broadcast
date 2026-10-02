@echo off
rem =============================================================================
rem install_on_pluto.bat - installs the PlutoSky 7020 Broadcast control software on a Pluto (Windows)
rem
rem Put this file in the SAME folder as skypluto-ctl, skypluto-mask, activate.sh and the
rem *.sh scripts (the "pluto" folder of the release), then double-click it, or run:
rem     install_on_pluto.bat <ip-address>
rem Needs the OpenSSH client and tar, both part of Windows 10/11 (Settings > Optional features).
rem The Pluto asks for the root password once (default: analog).
rem =============================================================================
setlocal
cd /d "%~dp0"
echo.
echo PlutoSky 7020 Broadcast - install the control software on the Pluto
echo.

where ssh >nul 2>nul
if errorlevel 1 (
    echo The OpenSSH client was not found. Install it: Settings - Apps - Optional features - OpenSSH Client.
    pause
    exit /b 1
)
where tar >nul 2>nul
if errorlevel 1 (
    echo tar was not found. It is part of Windows 10 version 1803 and later.
    pause
    exit /b 1
)

for %%f in (skypluto-ctl skypluto-mask activate.sh autorun.sh skypluto-supervise.sh skypluto-autocal.sh skypluto-wfm.sh skypluto-cmd.sh) do (
    if not exist "%%f" (
        echo Missing file: %%f
        echo Put this batch file in the release's "pluto" folder, next to the other files.
        pause
        exit /b 1
    )
)

set "PLUTO=%~1"
if "%PLUTO%"=="" set /p PLUTO=IP address of the Pluto:
if "%PLUTO%"=="" (
    echo No address given.
    pause
    exit /b 1
)

set "PKG=%TEMP%\skypluto-pkg.tar"
tar --format ustar -cf "%PKG%" skypluto-ctl skypluto-mask activate.sh autorun.sh skypluto-supervise.sh skypluto-autocal.sh skypluto-wfm.sh skypluto-cmd.sh
if errorlevel 1 (
    echo Could not create the package.
    pause
    exit /b 1
)

echo.
echo Connecting to root@%PLUTO% - enter the password when asked (default: analog).
echo.
ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=NUL -o LogLevel=ERROR -o ConnectTimeout=10 root@%PLUTO% "rm -rf /tmp/pkg && mkdir -p /tmp/pkg && tar xf - -C /tmp/pkg && tr -d '\r' < /tmp/pkg/activate.sh > /tmp/pkg/a.sh && sh /tmp/pkg/a.sh" < "%PKG%"
set "RC=%ERRORLEVEL%"
del "%PKG%" >nul 2>nul

echo.
if not "%RC%"=="0" (
    echo FAILED: the installation did not complete, error code %RC%. Check the address, the password and that the Pluto is on the network.
) else (
    echo Done. Web interface: http://%PLUTO%/
    echo Power-cycle the Pluto once to check the complete start-up.
)
echo.
pause
exit /b %RC%
