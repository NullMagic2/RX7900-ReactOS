@echo off
rem Module: rx7900.tests.guest_installer.scenario
rem Purpose: Run the radeonctl physical-installer scenario against the inert
rem   rx7900-stub device in the ReactOS installer-rig VM and record every step.
rem Created: 2026-10-24
rem Architecture: Three phases separated by ReactOS restarts (phase 1 binds,
rem   phase 2 exercises the bound driver, phase 3 checks the post-removal state).
rem   RIGTOOL.EXE (installer_rig_tool.cpp) guards, runs radeonctl with captured
rem   stdout/stderr/exit codes, snapshots PnP/registry/file state and mirrors
rem   all of it to the kernel debug log and COM2. Results go to the RIGRESULT
rem   FAT disk (\RESULTS) or, without it, to %SystemDrive%\RIGRESULTS. The host
rem   checker work/root-tools/installer-rig/check_results.py compares them with
rem   the expected matrix. radeonctl is used unchanged from \PKGA and \PKGB.
rem   Usage: RIG.BAT [auto]   (auto = register RunOnce and restart between phases)
setlocal

set RIG_TOOL=%~dp0RIGTOOL.EXE
if not exist "%RIG_TOOL%" set RIG_TOOL=RIGTOOL.EXE
if not exist "%RIG_TOOL%" goto notool

"%RIG_TOOL%" locate "%SystemDrive%\RIGVARS.BAT"
if errorlevel 1 goto nomedia
call "%SystemDrive%\RIGVARS.BAT"
if "%RIG_RESULTS%"=="" set RIG_RESULTS=%SystemDrive%\RIGRESULTS
if not exist "%RIG_RESULTS%\." mkdir "%RIG_RESULTS%"
set RIG_LOG=%RIG_RESULTS%\results.txt
set PKGA=%RIG_MEDIA%\PKGA
set PKGB=%RIG_MEDIA%\PKGB
set RIG_MODE=%1

rem Phase markers live on the system disk (the qcow2 overlay), so a results
rem volume that is missing or renamed in a later boot can never restart the
rem scenario; the results location recorded in phase 1 must stay the same.
set RIG_STATE=%SystemDrive%\RIGSTATE
if not exist "%RIG_STATE%\." mkdir "%RIG_STATE%"
if not exist "%RIG_STATE%\RESULTS.BAT" echo set RIG_RESULTS_RECORDED=%RIG_RESULTS%> "%RIG_STATE%\RESULTS.BAT"
call "%RIG_STATE%\RESULTS.BAT"
if /i not "%RIG_RESULTS_RECORDED%"=="%RIG_RESULTS%" goto moved

if exist "%RIG_STATE%\PHASE3.DONE" goto alldone

"%RIG_TOOL%" guard "%RIG_LOG%"
if errorlevel 1 goto refused

if exist "%RIG_STATE%\PHASE2.DONE" goto phase3
if exist "%RIG_STATE%\PHASE1.DONE" goto phase2

rem ---------------------------------------------------------------------
rem Phase 1: default binding, refusals, the first binding.
rem ---------------------------------------------------------------------
:phase1
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 1 begin media=%RIG_MEDIA% results=%RIG_RESULTS%
"%RIG_TOOL%" settle "%RIG_LOG%"
"%RIG_TOOL%" snapshot P1-00 "%RIG_LOG%"
"%RIG_TOOL%" stubvars "%SystemDrive%\RIGSTUB.BAT"
call "%SystemDrive%\RIGSTUB.BAT"
"%RIG_TOOL%" note "%RIG_LOG%" DEFAULT-BINDING service=%RIG_STUB_SERVICE% started=%RIG_STUB_STARTED% problem=%RIG_STUB_PROBLEM%

call :run P1-01 "%PKGA%" status
call :run P1-02 "%PKGA%" install --dry-run
call :run P1-03 "%PKGA%" install --2d
call :run P1-04 "%PKGA%" install --asic --2d-compute
if "%RIG_STUB_SERVICE%"=="" goto p1unbound

"%RIG_TOOL%" note "%RIG_LOG%" MODE foreign
call :run P1-05 "%PKGA%" install --force-replace RigNoSuchService
call :run P1-06 "%PKGA%" install --asic --force-replace %RIG_STUB_SERVICE%
call :run P1-07 "%PKGA%" install --rom-override --force-replace %RIG_STUB_SERVICE%
call :run P1-08 "%PKGA%" install --dry-run --force-replace %RIG_STUB_SERVICE%
call :run P1-09 "%PKGA%" install
call :run P1-10 "%PKGA%" install --force-replace %RIG_STUB_SERVICE%
goto p1bound

:p1unbound
"%RIG_TOOL%" note "%RIG_LOG%" MODE unbound
call :run P1-06 "%PKGA%" install --asic
call :run P1-07 "%PKGA%" install --rom-override
call :run P1-09 "%PKGA%" install

:p1bound
call :run P1-11 "%PKGA%" status
call :run P1-12 "%PKGA%" install
call :run P1-13 "%PKGA%" recover
echo done> "%RIG_STATE%\PHASE1.DONE"
echo done> "%RIG_RESULTS%\PHASE1.DONE"
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 1 end - restart ReactOS, then run RIG.BAT again
goto restart

rem ---------------------------------------------------------------------
rem Phase 2: bound driver loaded, firmware gate, upgrade/rollback/recover,
rem simulated interruption, removal.
rem ---------------------------------------------------------------------
:phase2
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 2 begin
"%RIG_TOOL%" snapshot P2-00 "%RIG_LOG%"
call :run P2-01 "%PKGA%" status
call :run P2-02 "%PKGA%" install --asic
call :run P2-03 "%PKGA%" firmware-copy %RIG_MEDIA%\FWBAD
call :run P2-04 "%PKGA%" firmware-copy %RIG_MEDIA%\FIRMWARE
call :run P2-05 "%PKGA%" firmware-copy %RIG_MEDIA%\FIRMWARE
call :run P2-06 "%PKGA%" install --asic --dry-run
call :run P2-07 "%PKGA%" status
call :run P2-08 "%PKGA%" upgrade
call :run P2-09 "%PKGA%" rollback
call :run P2-10 "%PKGB%" upgrade
call :run P2-11 "%PKGB%" recover
call :run P2-12 "%PKGB%" status
call :run P2-13 "%PKGA%" rollback
call :run P2-14 "%PKGA%" upgrade
rem Forced restart-deferred replacement: radeondisp.dll is held open (no share
rem delete) during the PKGB upgrade, then rolled back with nothing held.
"%RIG_TOOL%" hold "%SystemRoot%\System32\radeondisp.dll" run "%RIG_RESULTS%" P2-20 "%PKGB%\radeonctl.exe" upgrade
"%RIG_TOOL%" snapshot P2-20 "%RIG_LOG%"
call :run P2-21 "%PKGA%" rollback
"%RIG_TOOL%" simulate-interrupt "%RIG_LOG%"
"%RIG_TOOL%" snapshot P2-15 "%RIG_LOG%"
call :run P2-16 "%PKGA%" status
call :run P2-17 "%PKGA%" recover
call :run P2-18 "%PKGA%" remove
call :run P2-19 "%PKGA%" status
echo done> "%RIG_STATE%\PHASE2.DONE"
echo done> "%RIG_RESULTS%\PHASE2.DONE"
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 2 end - restart ReactOS, then run RIG.BAT again
goto restart

rem ---------------------------------------------------------------------
rem Phase 3: after removal and a restart ReactOS re-detects the stub.
rem ---------------------------------------------------------------------
:phase3
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 3 begin
"%RIG_TOOL%" settle "%RIG_LOG%"
"%RIG_TOOL%" snapshot P3-00 "%RIG_LOG%"
call :run P3-01 "%PKGA%" status
call :run P3-02 "%PKGA%" recover
call :run P3-03 "%PKGA%" remove
call :run P3-04 "%PKGA%" install --dry-run
echo done> "%RIG_STATE%\PHASE3.DONE"
echo done> "%RIG_RESULTS%\PHASE3.DONE"
"%RIG_TOOL%" note "%RIG_LOG%" PHASE 3 end - SCENARIO COMPLETE; shut ReactOS down and run check_results.py
echo Scenario complete. Shut ReactOS down (Start, Shut Down) so the results disk is flushed.
goto end

:restart
if /i not "%RIG_MODE%"=="auto" goto manualrestart
reg add "HKLM\Software\Microsoft\Windows\CurrentVersion\RunOnce" /v RX7900InstallerRig /t REG_SZ /d "%RIG_MEDIA%\RIG.BAT auto" /f
shutdown -r -t 5
goto end

:manualrestart
echo Phase finished. Restart ReactOS, then run %RIG_MEDIA%\RIG.BAT again.
goto end

:alldone
echo The scenario already completed; results are in %RIG_RESULTS%.
goto end

:moved
echo REFUSED: results were recorded in %RIG_RESULTS_RECORDED% but this boot found %RIG_RESULTS%.
echo Attach the same RIGRESULT disk again (or start over with a fresh overlay and results disk).
goto end

:refused
echo REFUSED: the installer-rig guard failed (see %RIG_LOG%). Nothing was run.
goto end

:notool
echo RIGTOOL.EXE was not found beside RIG.BAT.
goto end

:nomedia
echo The installer-rig ISO (RIGMEDIA.TXT) was not found on any drive.
goto end

rem run STEP PACKAGE-DIR radeonctl-arguments...: one recorded radeonctl call
rem followed by a state snapshot labelled with the same step.
:run
set RIG_STEP=%1
set RIG_PACKAGE=%~2
shift
shift
"%RIG_TOOL%" run "%RIG_RESULTS%" %RIG_STEP% "%RIG_PACKAGE%\radeonctl.exe" %1 %2 %3 %4 %5 %6
"%RIG_TOOL%" snapshot %RIG_STEP% "%RIG_LOG%"
goto :eof

:end
if /i not "%RIG_MODE%"=="auto" pause
endlocal
