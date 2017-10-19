@set build_type=Release
@if not "%1"=="" set build_type=%1
@echo Configuring for build type %build_type%

cmake .. -G "NMake Makefiles" -DCMAKE_INSTALL_PREFIX="%~d0\install" -DCMAKE_BUILD_TYPE="%build_type%" -DENABLE_TESTS_COMPILATION:BOOL=True
