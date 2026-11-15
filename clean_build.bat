@echo off
rem Removes the owned build intermediates. Recreated 2026-11-15. Not run on Windows.
cmake -DDRIVER_ACTION=clean %* -P "%~dp0cmake\driver-package.cmake"
