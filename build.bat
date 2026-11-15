@echo off
rem Builds a driver package with cmake\driver-package.cmake. See build.sh for the options.
rem Recreated 2026-11-15; the original wrapper was not preserved. Not run on Windows.
cmake -DDRIVER_ACTION=build -DDRIVER_PACKAGE=radeon -DDRIVER_BUILD_TYPE=Release %* -P "%~dp0cmake\driver-package.cmake"
