@echo off
setlocal

set "ROOT=%~dp0"
for %%I in ("%ROOT%.") do set "PROJECT=%%~fI"
for %%I in ("%ROOT%..") do set "WORKSPACE=%%~fI"
set "CCS=D:\Ti_M0\CCS\ccs\eclipse\ccs-server-cli.bat"
set "GMAKE=D:\Ti_M0\CCS\ccs\utils\bin\gmake.exe"
set "OUT=%ROOT%BuildProfiles"

if not exist "%OUT%" mkdir "%OUT%"

call "%CCS%" -workspace "%WORKSPACE%" -application projectBuild ^
  -ccs.locations "%PROJECT%" -ccs.buildType full -ccs.autoImport -ccs.autoOpen -ccs.listProblems
if errorlevel 1 exit /b 1

copy /y "%ROOT%Debug\MSPM0G3507_ZF.out" "%OUT%\MSPM0G3507_ZF_line_car.out" >nul
if errorlevel 1 exit /b 1
copy /y "%ROOT%Debug\MSPM0G3507_ZF.map" "%OUT%\MSPM0G3507_ZF_line_car.map" >nul
if errorlevel 1 exit /b 1

pushd "%ROOT%Debug"
"%GMAKE%" clean
if errorlevel 1 goto :fail
"%GMAKE%" -j4 all GEN_OPTS__FLAG="-UEC_APP_PROFILE -DEC_APP_PROFILE=1"
if errorlevel 1 goto :fail
copy /y "MSPM0G3507_ZF.out" "%OUT%\MSPM0G3507_ZF_hardware_test.out" >nul
if errorlevel 1 goto :fail
copy /y "MSPM0G3507_ZF.map" "%OUT%\MSPM0G3507_ZF_hardware_test.map" >nul
if errorlevel 1 goto :fail

"%GMAKE%" clean
if errorlevel 1 goto :fail
"%GMAKE%" -j4 all
if errorlevel 1 goto :fail
popd

echo Built hardware-test and line-car profiles in "%OUT%".
exit /b 0

:fail
popd
exit /b 1
