@echo off
setlocal

set "SCRIPT_DIR=%~dp0"

REM Locate conda activate script
set "CONDA_ACTIVATE="

if exist "%USERPROFILE%\miniforge3\Scripts\activate.bat" (
    set "CONDA_ACTIVATE=%USERPROFILE%\miniforge3\Scripts\activate.bat"
    goto :activate
)
if exist "%USERPROFILE%\miniconda3\Scripts\activate.bat" (
    set "CONDA_ACTIVATE=%USERPROFILE%\miniconda3\Scripts\activate.bat"
    goto :activate
)
if exist "%USERPROFILE%\anaconda3\Scripts\activate.bat" (
    set "CONDA_ACTIVATE=%USERPROFILE%\anaconda3\Scripts\activate.bat"
    goto :activate
)

echo WARNING: Could not find conda activate.bat — trying without activation
goto :run

:activate
call "%CONDA_ACTIVATE%" ros_env

:run
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /d "%SCRIPT_DIR%"
python movement_pkg\movement.py
