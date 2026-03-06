@echo off
setlocal

set "ROOT=%~dp0"
set "UV4=D:\keil 5 MDK\UV4\UV4.exe"
set "PROJ=basic_framework.uvprojx"
set "TARGET=basic_framework"
set "LOG=.vscode\uv4.log"

if not exist "%UV4%" (
  echo [ERROR] UV4 not found: "%UV4%"
  exit /b 1
)

pushd "%ROOT%MDK-ARM" || (
  echo [ERROR] Cannot enter MDK-ARM directory.
  exit /b 1
)

echo [INFO] Building "%PROJ%" target "%TARGET%"...
"%UV4%" -b "%PROJ%" -j0 -t "%TARGET%" -o "%LOG%"
set "RET=%ERRORLEVEL%"

echo.
echo [INFO] Build log: %CD%\%LOG%
popd

exit /b %RET%
