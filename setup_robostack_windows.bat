@echo off
setlocal enabledelayedexpansion

echo === RoboStack ROS2 Humble Setup for Windows 11 ===
echo.

REM Step 1: Locate conda executable
set "CONDA_EXE="

where conda >nul 2>&1
if %ERRORLEVEL% == 0 (
    for /f "delims=" %%i in ('where conda') do (
        set "CONDA_EXE=%%i"
        goto :found_conda
    )
)

if exist "%USERPROFILE%\miniforge3\Scripts\conda.exe" (
    set "CONDA_EXE=%USERPROFILE%\miniforge3\Scripts\conda.exe"
    goto :found_conda
)
if exist "%USERPROFILE%\miniconda3\Scripts\conda.exe" (
    set "CONDA_EXE=%USERPROFILE%\miniconda3\Scripts\conda.exe"
    goto :found_conda
)
if exist "%USERPROFILE%\anaconda3\Scripts\conda.exe" (
    set "CONDA_EXE=%USERPROFILE%\anaconda3\Scripts\conda.exe"
    goto :found_conda
)

echo ERROR: conda not found.
echo Please install Miniforge3 first:
echo   https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Windows-x86_64.exe
exit /b 1

:found_conda
echo conda found: %CONDA_EXE%

REM Step 2: Create ros_env if it doesn't already exist
"%CONDA_EXE%" env list | findstr /C:"ros_env" >nul 2>&1
if %ERRORLEVEL% == 0 (
    echo Environment 'ros_env' already exists.
) else (
    echo.
    echo Creating 'ros_env' (this may take a few minutes)...
    echo.
    "%CONDA_EXE%" create -n ros_env -y -c conda-forge -c robostack-humble python=3.11
    if %ERRORLEVEL% neq 0 (
        echo ERROR: conda create failed. See output above.
        exit /b 1
    )
)

REM Step 3: Install / verify all required packages (safe to re-run)
echo.
echo Installing / verifying required packages in ros_env...
echo   (This may take 10-20 minutes on first install)
echo.

"%CONDA_EXE%" install -n ros_env -y ^
    -c conda-forge ^
    -c robostack-humble ^
    ros-humble-desktop ^
    ros-humble-rmw-cyclonedds-cpp ^
    ros-humble-rmw-fastrtps-cpp ^
    ros-humble-xacro ^
    ros-humble-robot-state-publisher ^
    ros-humble-gazebo-ros-pkgs

if %ERRORLEVEL% neq 0 (
    echo.
    echo WARNING: Some packages may not be available for Windows.
    echo   ros-humble-gazebo-ros-pkgs is optional - Gazebo Classic has limited Windows support.
    echo   Retrying without Gazebo packages...
    echo.
    "%CONDA_EXE%" install -n ros_env -y ^
        -c conda-forge ^
        -c robostack-humble ^
        ros-humble-desktop ^
        ros-humble-rmw-cyclonedds-cpp ^
        ros-humble-rmw-fastrtps-cpp ^
        ros-humble-xacro ^
        ros-humble-robot-state-publisher
    if %ERRORLEVEL% neq 0 (
        echo ERROR: Package installation failed. See output above.
        exit /b 1
    )
)

:configure_channels
REM Step 4: Add robostack-humble channel to the environment
echo.
echo Configuring channels in ros_env...
"%CONDA_EXE%" run -n ros_env conda config --env --add channels robostack-humble 2>nul

echo.
echo === Setup complete ===
echo.
echo To verify, open a new Command Prompt and run:
echo   conda activate ros_env
echo   rviz2
echo.
echo You can now use the RViz and Gazebo buttons in the Robot Control app.
