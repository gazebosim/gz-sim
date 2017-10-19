
:: NOTE: This script is only meant to be used as part of the ignition developers' CI system
:: Users and developers should build and install this library using cmake and Visual Studio

:: Install dependencies
call %win_lib% :install_ign_project ign-cmake WindowsUpdate-Oct2017

:: Set configuration variables
@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

:: Go to the directory that this file configure.bat file exists in
cd %~dp0

:: Create a build directory and configure
md build
cd build
cmake .. -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="%WORKSPACE_INSTALL_DIR%" -DCMAKE_BUILD_TYPE="%build_type%" -DENABLE_TESTS_COMPILATION:BOOL=True

:: If the caller wants to build and/or install, they should do so after calling this script
