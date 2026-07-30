@echo off
setlocal

set "SYSCONFIG_GUI=D:\application_restore\ti\SysConfig\sysconfig_gui.bat"
set "MSPM0_PRODUCT=D:\application_restore\ti\mspm0_sdk\mspm0_sdk_2_05_00_05\.metadata\product.json"
set "SYSCFG_SCRIPT=%~dp0knexus_mspm0.syscfg"

if not exist "%SYSCONFIG_GUI%" (
    echo SysConfig was not found: %SYSCONFIG_GUI%
    exit /b 1
)

if not exist "%MSPM0_PRODUCT%" (
    echo MSPM0 SDK product metadata was not found: %MSPM0_PRODUCT%
    exit /b 1
)

call "%SYSCONFIG_GUI%" --product "%MSPM0_PRODUCT%" --script "%SYSCFG_SCRIPT%"
