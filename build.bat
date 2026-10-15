@echo off
rem Builds the FPV Drone GML plugin and its in-game test harness.
rem   build.bat            build into build\
rem   build.bat install    also copy the plugin into the game (refuses while the game runs)
rem
rem The drone's and goggles' meshes and textures are encrypted (tools\pack, a new key every build) and
rem built into FPVDrone.dll (src\FPVDrone.rc), so Assets\ must exist first: generate it from the 3DMA
rem downloads with tools\prepare_assets.py.
rem Needs the GML SDK 2.2+ (github.com/RemArrow/geronimo-mod-loader): the external\gml submodule, or
rem set GML_SDK to its include folder. Set GML_GAME_DIR to your ...\Geronimo\Binaries\Win64 if the game
rem is not in the default Steam library.
setlocal
set ROOT=%~dp0
set OUT=%ROOT%build
set SDK=%ROOT%external\gml\include
if not exist "%SDK%\GML\GML.hpp" set SDK=%ROOT%..\ModLoader\include
if defined GML_SDK set SDK=%GML_SDK%
set GAME=C:\Program Files (x86)\Steam\steamapps\common\GERONIMO\Geronimo\Binaries\Win64
if defined GML_GAME_DIR set GAME=%GML_GAME_DIR%

if not exist "%SDK%\GML\GML.hpp" (
  echo GML SDK not found at %SDK%
  echo Run: git submodule update --init   or set GML_SDK
  exit /b 1
)
if not exist "%ROOT%Assets\drone_body.obj" (
  echo Assets\ is missing - generate it with tools\prepare_assets.py first ^(it is built into the DLL^).
  exit /b 1
)
if not defined VCToolsInstallDir (
  call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
)
for %%D in ("%OUT%\obj" "%OUT%\gen" "%OUT%\FPVDrone" "%OUT%\tests\FPVDroneDev" "%OUT%\tests\FPVDroneQuick") do if not exist "%%~D" mkdir "%%~D"
set CFLAGS=/nologo /std:c++20 /O2 /MT /EHsc /W3 /Zi /DUNICODE /D_UNICODE /I"%SDK%"
set LFLAGS=/DEBUG /INCREMENTAL:NO

echo === pack (encrypt Assets\ with a new key)
cl /nologo /std:c++20 /O2 /MT /EHsc /Fo"%OUT%\obj\\" "%ROOT%tools\pack\pack.cpp" /Fe"%OUT%\pack.exe" || exit /b 1
rem A random key can, rarely, put a 4-byte signature the check looks for into the ciphertext: try again.
set TRIES=0
:pack
set /a TRIES+=1
"%OUT%\pack.exe" "%ROOT%Assets" "%OUT%\gen\payload.bin" "%OUT%\gen\payload_key.h" && goto packed
if %TRIES% lss 5 goto pack
exit /b 1
:packed
echo === plugin (encrypted assets embedded)
rem rc looks for the .rc's build\gen\ files in the current directory first: run it from this plugin's folder
pushd "%ROOT%" || exit /b 1
rc /nologo /I "%ROOT%." /fo"%OUT%\obj\FPVDrone.res" "%ROOT%src\FPVDrone.rc" || (popd & exit /b 1)
popd
cl %CFLAGS% /I"%OUT%\gen" /LD /Fo"%OUT%\obj\\" /Fd"%OUT%\obj\\" "%ROOT%src\*.cpp" "%OUT%\obj\FPVDrone.res" /Fe"%OUT%\FPVDrone\FPVDrone.dll" /link %LFLAGS% || exit /b 1
echo === test harness and quick-fly dev build (never installed)
cl %CFLAGS% /LD /Fo"%OUT%\obj\\" /Fd"%OUT%\obj\\" "%ROOT%tests\FPVDroneDev\FPVDroneDev.cpp" /Fe"%OUT%\tests\FPVDroneDev\FPVDroneDev.dll" /link %LFLAGS% || exit /b 1
cl %CFLAGS% /LD /Fo"%OUT%\obj\\" /Fd"%OUT%\obj\\" "%ROOT%tests\FPVDroneQuick\FPVDroneQuick.cpp" /Fe"%OUT%\tests\FPVDroneQuick\FPVDroneQuick.dll" /link %LFLAGS% || exit /b 1
for /r "%OUT%" %%F in (*.exp *.lib *.ilk) do del /q "%%F"

if /i "%1"=="install" (
  tasklist /FI "IMAGENAME eq Geronimo-Win64-Shipping.exe" | find /I "Geronimo-Win64" >nul && (
    echo Geronimo is running - close it first.
    exit /b 1
  )
  echo === install to "%GAME%\GML\plugins\FPVDrone"
  if not exist "%GAME%\GML\plugins\FPVDrone" mkdir "%GAME%\GML\plugins\FPVDrone"
  copy /y "%OUT%\FPVDrone\FPVDrone.dll" "%GAME%\GML\plugins\FPVDrone\" >nul || exit /b 1
  copy /y "%ROOT%CREDITS.txt" "%GAME%\GML\plugins\FPVDrone\" >nul || exit /b 1
  rem symbols stay in build\ (they would map out the key handling)
  if exist "%GAME%\GML\plugins\FPVDrone\FPVDrone.pdb" del /q "%GAME%\GML\plugins\FPVDrone\FPVDrone.pdb"
)
echo === done
exit /b 0
