#!/usr/bin/env python3
"""PyQt6 macOS app for Wheeltec robot parameter control and deployment."""

import sys
import os
import re
import time
import json
import glob
import shutil
import subprocess
import signal
import math
import urllib.request
import urllib.error
import base64
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QDoubleSpinBox, QComboBox, QPushButton, QTextEdit,
    QGroupBox, QGridLayout, QLineEdit, QMessageBox,
    QTabWidget, QPlainTextEdit, QStackedWidget, QListWidget,
    QSplitter, QScrollArea, QInputDialog, QTreeWidget, QTreeWidgetItem,
    QTreeWidgetItemIterator, QAbstractItemView, QTableWidget, QTableWidgetItem, QHeaderView,
    QCheckBox, QDialog, QDialogButtonBox, QFormLayout, QFrame,
    QButtonGroup, QMenu, QRadioButton,
)
from PyQt6.QtCore import (
    QThread, pyqtSignal, QRegularExpression, Qt, QSize, QRect,
    QTimer, QMimeData, QPointF, QRectF,
)
from PyQt6.QtGui import (
    QFont, QSyntaxHighlighter, QTextCharFormat, QColor, QPainter, QDrag,
    QPen, QBrush, QPolygonF, QTextCursor, QPainterPath, QPixmap, QIcon,
)

import socket
import paramiko

# --- Configuration ---
_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
MOVEMENT_PY = os.path.join(_PKG_DIR, "movement_pkg", "movement.py")
PROFILES_FILE = os.path.join(_PKG_DIR, ".robot_profiles.json")
DEFAULT_IP = "192.168.0.100"
REMOTE_MOVEMENT_PY = "~/wheeltec_ws/src/project1/movement_pkg/movement_pkg/movement.py"
_CANVAS_STATE_FILE = os.path.join(_PKG_DIR, ".node_canvas.json")
_GIT_CREDS_FILE    = os.path.join(_PKG_DIR, ".git_credentials.json")

# Default canvas items that cannot be deleted
_PROTECTED_PACKAGE = "movement_pkg"
_PROTECTED_NODES = {"movement"}
_PROTECTED_TOPICS = {"/cmd_vel", "/scan"}


# Source ROS2 base setup — auto-detects the distro from /opt/ros/
ROS_SOURCE_CMD = (
    'ROS_SETUP=$(ls -d /opt/ros/*/setup.bash 2>/dev/null | head -1) && '
    'source "$ROS_SETUP"'
)

REMOTE_BUILD_CMD = (
    ROS_SOURCE_CMD + " && "
    "cd ~/wheeltec_ws && "
    "colcon build --packages-select movement_pkg"
)

NODES = [
    ("robot_base", ROS_SOURCE_CMD + " && source ~/wheeltec_ros2/install/setup.bash && "
     "ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"),
    ("camera", ROS_SOURCE_CMD + " && source ~/wheeltec_ros2/install/setup.bash && "
     "ros2 launch wheeltec_camera wheeltec_camera.launch.py"),
    # lidar is already launched by turn_on_wheeltec_robot — launching it
    # separately causes duplicate serial port access and driver crashes
    ("movement", ROS_SOURCE_CMD + " && source ~/wheeltec_ros2/install/setup.bash && "
     "source ~/wheeltec_ws/install/setup.bash && "
     "ros2 launch movement_pkg movement_pkg.launch.py"),
]

# Files to show in Full View (relative to _PKG_DIR)
_FULL_VIEW_FILES = [
    "movement_pkg/movement.py",
    "movement_pkg/__init__.py",
    "launch/movement_pkg.launch.py",
    "launch/gazebo_sim.launch.py",
    "worlds/simulation.world",
    "setup.py",
    "setup_robostack.sh",
    "mini_mec_robot_gazebo.urdf",
]

# Default project files/folders that cannot be deleted from Full View
_PROTECTED_FV_FOLDERS = {"movement_pkg", "launch", "worlds", "roboapps"}
_PROTECTED_FV_FILES = set(_FULL_VIEW_FILES) | {
    "default_view.rviz", "mini_mec_robot.urdf", "package.xml",
    "testdrivemac.py", "setup.cfg",
}

# Code snippets for drag-and-drop in Simple View (8-space indent for __init__ body)
_SIMPLE_VIEW_SNIPPETS = {
    # Control
    "if_statement": (
        "        # --- if statement ---  \u2190 edit\n"
        "        if condition:  # \u2190 edit condition\n"
        "            pass  # \u2190 edit action\n"
    ),
    "while_statement": (
        "        # --- while loop ---  \u2190 edit\n"
        "        while condition:  # \u2190 edit condition\n"
        "            pass  # \u2190 edit action\n"
    ),
    "switch_statement": (
        "        # --- if/elif ---  \u2190 edit\n"
        "        if value == option1:  # \u2190 edit\n"
        "            pass\n"
        "        elif value == option2:  # \u2190 edit\n"
        "            pass\n"
        "        else:\n"
        "            pass\n"
    ),
    "for_loop": (
        "        # --- for loop ---  \u2190 edit\n"
        "        for i in range(10):  # \u2190 edit range\n"
        "            pass  # \u2190 edit action\n"
    ),
    # Movement
    "forward_speed": (
        "        self.move(self.forward_speed)  # drive forward\n"
    ),
    "backward_speed": (
        "        self.move(-self.backward_speed)  # reverse\n"
    ),
    "turn_speed": (
        "        self.set_speed(self.turn_speed)  # set turn speed\n"
    ),
    "turn_clockwise": (
        "        self.turn_cw(self.turn_cw_deg)  # turn CW\n"
    ),
    "turn_anti_clockwise": (
        "        self.turn_acw(self.turn_acw_deg)  # turn ACW\n"
    ),
    "stop_movement": (
        "        self.stop()  # stop robot\n"
    ),
    # Sensing
    "obstacle_distance": (
        "        if self.obstacle_in_front():  # check obstacle\n"
        "            pass  # \u2190 edit action\n"
    ),
    "colour_detection": (
        "        if self.detect_colour():  # detect colour\n"
        "            pass  # \u2190 edit action\n"
    ),
}


# --- Worker threads ---

class ConnectWorker(QThread):
    log = pyqtSignal(str)
    connected = pyqtSignal(object)  # emits the SSHClient
    failed = pyqtSignal(str)

    def __init__(self, host, username, password):
        super().__init__()
        self.host = host
        self.username = username
        self.password = password

    def run(self):
        try:
            self.log.emit(f"Connecting to {self.username}@{self.host}...")
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(self.host, username=self.username,
                           password=self.password, timeout=10)
            self.log.emit("Connected successfully.")
            self.connected.emit(client)
        except Exception as e:
            self.failed.emit(str(e))


class DeployWorker(QThread):
    log = pyqtSignal(str)
    finished_ok = pyqtSignal()

    def __init__(self, ssh_client, movement_py_path, params):
        super().__init__()
        self.ssh = ssh_client
        self.movement_py_path = movement_py_path
        self.params = params

    def run(self):
        try:
            self._update_movement_py()
            self._upload()
            self._build()
            self.log.emit("Deploy complete.")
            self.finished_ok.emit()
        except Exception as e:
            self.log.emit(f"ERROR: {e}")

    def _update_movement_py(self):
        self.log.emit("Updating movement.py parameters...")
        with open(self.movement_py_path, "r") as f:
            code = f.read()

        code = re.sub(
            r"(self\.forward_speed\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['forward_speed']}", code)
        code = re.sub(
            r"(self\.backward_speed\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['backward_speed']}", code)
        code = re.sub(
            r"(self\.turn_speed\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['turn_speed']}", code)
        code = re.sub(
            r"(self\.obstacle_distance\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['obstacle_distance']}", code)

        code = re.sub(
            r"(self\.turn_cw_deg\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['turn_cw_deg']}", code)
        code = re.sub(
            r"(self\.turn_acw_deg\s*=\s*)[\d.]+",
            rf"\g<1>{self.params['turn_acw_deg']}", code)
        code = re.sub(
            r'(self\.colour_detection\s*=\s*")[^"]*"',
            rf'\g<1>{self.params["colour_detection"]}"', code)

        with open(self.movement_py_path, "w") as f:
            f.write(code)
        self.log.emit("  movement.py updated locally.")

    def _upload(self):
        self.log.emit("Uploading to robot via SFTP...")
        sftp = self.ssh.open_sftp()
        # Expand ~ on remote
        stdin, stdout, stderr = self.ssh.exec_command("echo " + REMOTE_MOVEMENT_PY)
        remote_path = stdout.read().decode().strip()
        sftp.put(self.movement_py_path, remote_path)
        sftp.close()
        self.log.emit("  Upload complete.")

    def _build(self):
        self.log.emit("Building on robot (this may take a minute)...")
        _, stdout, stderr = self.ssh.exec_command(REMOTE_BUILD_CMD, timeout=180)
        exit_code = stdout.channel.recv_exit_status()
        if exit_code != 0:
            err = stderr.read().decode()
            raise RuntimeError(f"Build failed: {err}")
        self.log.emit("  Build successful.")


class SSHCmdWorker(QThread):
    log = pyqtSignal(str)

    def __init__(self, ssh_client, cmd, label=""):
        super().__init__()
        self.ssh = ssh_client
        self.cmd = cmd
        self.label = label

    def run(self):
        try:
            self.log.emit(f"Running: {self.label}...")
            _, stdout, stderr = self.ssh.exec_command(self.cmd, timeout=30)
            exit_code = stdout.channel.recv_exit_status()
            out = stdout.read().decode().strip()
            if out:
                self.log.emit(out)
            if exit_code != 0:
                err = stderr.read().decode().strip()
                if err:
                    self.log.emit(f"  stderr: {err}")
            self.log.emit(f"  Done: {self.label}")
        except Exception as e:
            self.log.emit(f"ERROR ({self.label}): {e}")


class StartAllWorker(QThread):
    log = pyqtSignal(str)
    status_update = pyqtSignal()

    def __init__(self, ssh_client):
        super().__init__()
        self.ssh = ssh_client

    def run(self):
        # Kill only non-robot-base ROS2 processes.
        # All nodes launched by turn_on_wheeltec_robot (wheeltec_robot_node,
        # lslidar_driver_node, imu_filter_madgwick, ekf_filter_node,
        # robot_state_publisher, joint_state_publisher) are intentionally left alive:
        # if the hardware driver is already healthy we reuse the whole group;
        # killing robot_state_publisher but not relaunching robot_base would leave
        # the base_link→lidar_link TF unpublished, making the model split apart in RViz.
        self.log.emit("Cleaning up stale ROS2 processes...")
        try:
            _, stdout, _ = self.ssh.exec_command(
                "pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; "
                "pkill -f 'movement_pkg' ; "
                "rm -f /tmp/movement_pause ; "
                "sleep 3 ; "
                "pkill -9 -f 'ros2 launch' ; pkill -9 -f 'ros2 run' ; "
                "pkill -9 -f 'movement_pkg' ; "
                "sleep 5 ; true", timeout=30)
            stdout.channel.recv_exit_status()
        except Exception:
            pass

        for name, cmd in NODES:
            if name == "robot_base":
                # If the serial hardware driver is already running, skip the full
                # robot_base relaunch entirely.  Launching a duplicate
                # wheeltec_robot_node crashes on the busy serial port, and the
                # launch file's on_exit handler then kills ekf_filter_node and
                # robot_state_publisher too — causing the model to split and freeze.
                try:
                    _, chk_out, _ = self.ssh.exec_command(
                        "pgrep -f wheeltec_robot_node   > /dev/null 2>&1 && echo hw:1  || echo hw:0 ;"
                        "pgrep -f ekf_filter_node       > /dev/null 2>&1 && echo ekf:1 || echo ekf:0 ;"
                        "pgrep -f robot_state_publisher > /dev/null 2>&1 && echo rsp:1 || echo rsp:0",
                        timeout=5)
                    out = chk_out.read().decode(errors="replace")
                except Exception:
                    out = ""
                hw_up  = "hw:1"  in out
                ekf_up = "ekf:1" in out
                rsp_up = "rsp:1" in out

                if hw_up:
                    dead = [n for n, up in [("EKF", ekf_up), ("robot_state_publisher", rsp_up)]
                            if not up]
                    if not dead:
                        self.log.emit(
                            "  robot_base: all nodes healthy, skipping relaunch.")
                    else:
                        self.log.emit(
                            f"  robot_base: {', '.join(dead)} dead — recovering individually...")
                        self._recover_robot_base_nodes()
                    time.sleep(3)
                    continue
                # hw not running → fall through to normal robot_base launch below

            launch_cmd = (
                f"nohup bash -c '{cmd}' "
                f"> /tmp/{name}.log 2>&1 &"
            )
            self.log.emit(f"Starting {name}...")
            try:
                _, stdout, _ = self.ssh.exec_command(launch_cmd, timeout=10)
                stdout.channel.recv_exit_status()
                self.log.emit(f"  {name} started (log: /tmp/{name}.log)")
            except Exception as e:
                self.log.emit(f"  ERROR starting {name}: {e}")
            time.sleep(3)
        self.log.emit("All nodes started.")
        self.status_update.emit()

    def _recover_robot_base_nodes(self):
        """Restart dead ekf_filter_node / robot_state_publisher without touching
        the serial hardware driver.

        A Python recovery script is uploaded via SFTP so that the full URDF
        content can be passed directly as a Python string argument — completely
        avoiding the shell-quoting problems that arise when embedding XML in a
        bash command.  The script is then executed with the ROS2 environment
        already sourced so that 'ros2 run' resolves correctly.
        """
        # --- recovery script written to /tmp/testdrive_recovery.py on the robot ---
        RECOVERY_SCRIPT = (
            "import subprocess, glob\n"
            "\n"
            "def find_first(*patterns):\n"
            "    for p in patterns:\n"
            "        r = glob.glob(p, recursive=True)\n"
            "        if r: return r[0]\n"
            "    return ''\n"
            "\n"
            "def is_running(name):\n"
            "    return subprocess.run(\n"
            "        ['pgrep', '-f', name], capture_output=True).returncode == 0\n"
            "\n"
            "def bg(cmd, log):\n"
            "    subprocess.Popen(\n"
            "        cmd, stdout=open(log, 'w'), stderr=open(log, 'a'),\n"
            "        start_new_session=True)\n"
            "\n"
            "# ── EKF filter node ────────────────────────────────────────────────\n"
            "# Search for the ekf YAML specifically, then fall back to any YAML\n"
            "# in the turn_on_wheeltec_robot package share directory.\n"
            "ekf_yaml = find_first(\n"
            "    '/home/wheeltec/wheeltec_ros2/install/turn_on_wheeltec_robot/share/**/ekf*.yaml',\n"
            "    '/home/wheeltec/wheeltec_ros2/install/turn_on_wheeltec_robot/share/**/*.yaml')\n"
            "if is_running('ekf_filter_node'):\n"
            "    print('ekf:already_running')\n"
            "elif ekf_yaml:\n"
            "    bg(['ros2', 'run', 'robot_localization', 'ekf_filter_node',\n"
            "        '--ros-args', '--params-file', ekf_yaml],\n"
            "       '/tmp/ekf_restart.log')\n"
            "    print('ekf:started:' + ekf_yaml)\n"
            "else:\n"
            "    print('ekf:no_params_found')\n"
            "\n"
            "# ── robot_state_publisher ──────────────────────────────────────────\n"
            "# Find the robot URDF in the wheeltec_robot_urdf package share.\n"
            "urdf = find_first(\n"
            "    '/home/wheeltec/wheeltec_ros2/install/wheeltec_robot_urdf/share/**/*.urdf')\n"
            "if is_running('robot_state_publisher'):\n"
            "    print('rsp:already_running')\n"
            "elif urdf:\n"
            "    urdf_content = open(urdf).read()\n"
            "    bg(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',\n"
            "        '--ros-args', '-p', 'robot_description:=' + urdf_content],\n"
            "       '/tmp/rsp_restart.log')\n"
            "    print('rsp:started:' + urdf)\n"
            "else:\n"
            "    print('rsp:no_urdf_found')\n"
            "\n"
            "print('recovery:done')\n"
        )

        # Upload the script via SFTP (binary write avoids any encoding edge-cases)
        try:
            sftp = self.ssh.open_sftp()
            with sftp.open('/tmp/testdrive_recovery.py', 'wb') as fh:
                fh.write(RECOVERY_SCRIPT.encode('utf-8'))
            sftp.close()
        except Exception as e:
            self.log.emit(f"  Recovery upload failed: {e}")
            return

        # Run the script with the full ROS2 + wheeltec workspace environment
        run_cmd = (
            ROS_SOURCE_CMD + " && source ~/wheeltec_ros2/install/setup.bash && "
            "python3 /tmp/testdrive_recovery.py"
        )
        try:
            _, out, _ = self.ssh.exec_command(run_cmd, timeout=30)
            for line in out.read().decode(errors="replace").splitlines():
                if line.strip():
                    self.log.emit(f"  Recovery: {line.strip()}")
        except Exception as e:
            self.log.emit(f"  Recovery execution failed: {e}")


class StopAllWorker(QThread):
    log = pyqtSignal(str)
    status_update = pyqtSignal()

    def __init__(self, ssh_client):
        super().__init__()
        self.ssh = ssh_client

    def run(self):
        self.log.emit("Sending stop velocity...")
        try:
            self.ssh.exec_command(
                ROS_SOURCE_CMD + ' && '
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0}, angular: {z: 0.0}}"',
                timeout=10)
        except Exception:
            pass

        # Kill all ROS2 launch processes and their child nodes.
        # Use SIGKILL (-9) to ensure the full nohup -> bash -> ros2 launch -> node
        # process tree is terminated (SIGTERM often doesn't propagate).
        self.log.emit("Killing all ROS2 processes...")
        try:
            _, stdout, _ = self.ssh.exec_command(
                "pkill -f 'ros2 launch' ; pkill -f 'ros2 run' ; "
                "pkill -f wheeltec_robot_node ; pkill -f lslidar_driver_node ; "
                "pkill -f imu_filter_madgwick ; pkill -f robot_state_publisher ; "
                "pkill -f ekf_filter_node ; pkill -f joint_state_publisher ; "
                "pkill -f 'movement_pkg' ; "
                "sleep 3 ; "
                "pkill -9 -f 'ros2 launch' ; pkill -9 -f 'ros2 run' ; "
                "pkill -9 -f wheeltec_robot_node ; pkill -9 -f lslidar_driver_node ; "
                "pkill -9 -f imu_filter_madgwick ; pkill -9 -f robot_state_publisher ; "
                "pkill -9 -f ekf_filter_node ; pkill -9 -f joint_state_publisher ; "
                "pkill -9 -f 'movement_pkg' ; "
                "screen -wipe 2>/dev/null; sleep 5; true", timeout=30)
            stdout.channel.recv_exit_status()
        except Exception:
            pass
        self.log.emit("All nodes stopped.")
        self.status_update.emit()



class SimOutputWorker(QThread):
    """Runs run_sim.sh in a background thread and streams its output."""
    output   = pyqtSignal(str)
    finished = pyqtSignal()

    def __init__(self, script_path):
        super().__init__()
        self.script_path = script_path
        self._process    = None

    def run(self):
        try:
            self._process = subprocess.Popen(
                ['bash', self.script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            for line in self._process.stdout:
                self.output.emit(line.rstrip())
            self._process.wait()
        except Exception as e:
            self.output.emit(f"ERROR: {e}")
        self.finished.emit()

    def stop(self):
        # Layer 1: pkill targets movement.py directly by name, bypassing any
        # process-group ambiguity introduced by conda activate in the script.
        try:
            subprocess.run(
                ['pkill', '-SIGINT', '-f', 'movement_pkg/movement.py'],
                capture_output=True,
            )
        except Exception:
            pass
        # Layer 2: SIGINT to the bash wrapper's process group as a fallback.
        if self._process and self._process.poll() is None:
            try:
                os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            except Exception:
                try:
                    os.kill(self._process.pid, signal.SIGINT)
                except Exception:
                    pass


class SimInitiationDialog(QDialog):
    """Popup that lets the user initiate / stop the movement node in Gazebo."""

    def __init__(self, script_path, parent=None):
        super().__init__(parent)
        self.script_path = script_path
        self._worker     = None
        self.setWindowTitle("Simulation Control")
        # 20% smaller than the previous half-screen size → ~40% of screen
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.4), int(screen.height() * 0.4))
        # Window (not Dialog) type gives a full title bar with minimize on macOS
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowStaysOnTopHint
            | Qt.WindowType.WindowMinimizeButtonHint
            | Qt.WindowType.WindowCloseButtonHint
        )
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        instr = QLabel(
            "Gazebo needs to receive sensor data from your code.<br><br>"
            "When you are ready to simulate, click the <b>Start</b> button.<br>"
            "Click <b>Stop</b> button to stop the simulation.<br><br>"
            "If this window is closed, click the Gazebo button again to bring it back."
        )
        instr.setWordWrap(True)
        instr.setTextFormat(Qt.TextFormat.RichText)
        instr.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(instr)
        layout.addSpacing(16)

        btn_row = QHBoxLayout()
        btn_row.addStretch()

        self.initiate_btn = QPushButton("Start")
        self.initiate_btn.setStyleSheet(
            "background-color: #34C759; color: white; "
            "padding: 8px; border-radius: 8px;"
        )
        self.initiate_btn.setMinimumWidth(100)
        self.initiate_btn.clicked.connect(self._initiate)
        btn_row.addWidget(self.initiate_btn)

        self.stop_sim_btn = QPushButton("Stop")
        self.stop_sim_btn.setStyleSheet(
            "QPushButton { background-color: #FF3B30; color: white; "
            "padding: 8px; border-radius: 8px; }"
            "QPushButton:disabled { background-color: #B0B0B0; "
            "color: #707070; border-radius: 8px; }"
        )
        self.stop_sim_btn.setMinimumWidth(100)
        self.stop_sim_btn.setEnabled(False)
        self.stop_sim_btn.clicked.connect(self._stop)
        btn_row.addWidget(self.stop_sim_btn)

        btn_row.addStretch()
        layout.addLayout(btn_row)
        layout.addSpacing(16)

        self.output_area = QPlainTextEdit()
        self.output_area.setReadOnly(True)
        _mono = QFont("Menlo")
        _mono.setFixedPitch(True)
        _mono.setPointSize(11)
        self.output_area.setFont(_mono)
        self.output_area.setStyleSheet("background:#1e1e1e; color:#d4d4d4;")
        layout.addWidget(self.output_area)

    def _initiate(self):
        self.output_area.clear()
        self.output_area.appendPlainText("Starting simulation node...\n")
        self.initiate_btn.setEnabled(False)
        self.stop_sim_btn.setEnabled(True)
        self._worker = SimOutputWorker(self.script_path)
        self._worker.output.connect(self._append_output)
        self._worker.finished.connect(self._on_finished)
        self._worker.start()

    def _stop(self):
        if not self._worker:
            return
        self.stop_sim_btn.setEnabled(False)
        self.output_area.appendPlainText("\n[Stopping simulation...]")
        # Disconnect output so no more lines are appended after this point
        try:
            self._worker.output.disconnect(self._append_output)
        except Exception:
            pass
        # Create pause flag — movement.py control_loop publishes zero-velocity
        # within 100ms; wait 300ms before killing so it has time to act.
        try:
            open('/tmp/movement_pause', 'w').close()
        except Exception:
            pass
        QTimer.singleShot(300, self._send_sigint)

    def _send_sigint(self):
        """Kill movement.py 300ms after the pause flag was created."""
        if self._worker:
            self._worker.stop()
        # Pause file is cleaned up by _on_finished once the thread exits.

    def _append_output(self, line):
        self.output_area.appendPlainText(line)
        sb = self.output_area.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _on_finished(self):
        try:
            os.remove('/tmp/movement_pause')
        except FileNotFoundError:
            pass
        self._worker = None
        self.initiate_btn.setEnabled(True)
        self.stop_sim_btn.setEnabled(False)
        self.output_area.appendPlainText("\n[Simulation stopped]")

    def closeEvent(self, event):
        if self._worker:
            try:
                self._worker.output.disconnect(self._append_output)
            except Exception:
                pass
            try:
                open('/tmp/movement_pause', 'w').close()
            except Exception:
                pass
            self._worker.stop()
            try:
                os.remove('/tmp/movement_pause')
            except FileNotFoundError:
                pass
        super().closeEvent(event)


class CondaInstallWorker(QThread):
    """Downloads Miniforge3, installs it, then runs setup_robostack.sh to
    create the ros_env conda environment with ROS 2 Humble."""
    output   = pyqtSignal(str)
    finished = pyqtSignal(bool)  # True = both steps succeeded

    def run(self):
        import platform
        import urllib.request
        machine = platform.machine()   # 'arm64' or 'x86_64'
        url = (
            "https://github.com/conda-forge/miniforge/releases/latest/download/"
            f"Miniforge3-MacOSX-{machine}.sh"
        )
        installer = "/tmp/Miniforge3_installer.sh"
        prefix    = os.path.expanduser("~/miniforge3")

        # ── Step 1: Download Miniforge3 ──────────────────────────────────
        try:
            self.output.emit(f"[1/3] Downloading Miniforge3 for {machine}...")
            urllib.request.urlretrieve(url, installer)
        except Exception as e:
            self.output.emit(f"Download failed: {e}")
            self.finished.emit(False)
            return

        # ── Step 2: Install Miniforge3 ───────────────────────────────────
        self.output.emit("[2/3] Installing Miniforge3 (this may take 1-2 minutes)...")
        try:
            result = subprocess.run(
                ['bash', installer, '-b', '-p', prefix],
                capture_output=True, text=True, timeout=300,
            )
            if result.returncode != 0:
                self.output.emit(f"Miniforge3 installation failed:\n{result.stderr[:500]}")
                self.finished.emit(False)
                return
            self.output.emit("Miniforge3 installed successfully.")
        except Exception as e:
            self.output.emit(f"Miniforge3 installation error: {e}")
            self.finished.emit(False)
            return

        # ── Step 3: Run setup_robostack.sh to create ros_env ────────────
        setup_script = os.path.join(_PKG_DIR, "setup_robostack.sh")
        if not os.path.isfile(setup_script):
            self.output.emit(f"ERROR: setup_robostack.sh not found at {setup_script}")
            self.finished.emit(False)
            return

        self.output.emit("[3/3] Installing ROS 2 Humble into ros_env (this may take 5-10 minutes)...")
        try:
            proc = subprocess.Popen(
                ['bash', setup_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                cwd=_PKG_DIR,
            )
            for line in proc.stdout:
                self.output.emit(line.rstrip())
            proc.wait()
            if proc.returncode == 0:
                self.output.emit("\nROS 2 Humble installed successfully.")
                self.finished.emit(True)
            else:
                self.output.emit(f"\nsetup_robostack.sh failed (exit {proc.returncode}).")
                self.finished.emit(False)
        except Exception as e:
            self.output.emit(f"ROS 2 installation error: {e}")
            self.finished.emit(False)


# --- Drag-and-drop function buttons for Simple View ---

class DraggableFunctionButton(QPushButton):
    """A light-blue rounded button that can be dragged into the code editor."""

    def __init__(self, label, code_snippet, parent=None):
        super().__init__(label, parent)
        self._code_snippet = code_snippet
        self._drag_start_pos = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.setStyleSheet(
            "QPushButton {"
            "  background-color: #E0EAF0; border: 1px solid #B0C4D0;"
            "  border-radius: 8px; padding: 6px 10px; text-align: left;"
            "  font-size: 12px; color: #1A1A1A;"
            "}"
            "QPushButton:hover { background-color: #D0DDE8; }"
        )

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (self._drag_start_pos is not None
                and event.buttons() & Qt.MouseButton.LeftButton):
            distance = (event.pos() - self._drag_start_pos).manhattanLength()
            if distance >= QApplication.startDragDistance():
                drag = QDrag(self)
                mime = QMimeData()
                mime.setText(self._code_snippet)
                drag.setMimeData(mime)
                drag.exec(Qt.DropAction.CopyAction)
                self._drag_start_pos = None
        super().mouseMoveEvent(event)


class FunctionsPanel(QWidget):
    """Left-side panel listing draggable function blocks grouped by category."""

    _CATEGORIES = [
        ("Control", [
            ("if_statement", "if_statement"),
            ("while_statement", "while_statement"),
            ("switch_statement", "switch_statement"),
            ("for_loop", "for_loop"),
        ]),
        ("Movement", [
            ("move_forward", "forward_speed"),
            ("move_backward", "backward_speed"),
            ("set_turn_speed", "turn_speed"),
            ("turn_clockwise", "turn_clockwise"),
            ("turn_anti_clockwise", "turn_anti_clockwise"),
            ("stop", "stop_movement"),
        ]),
        ("Sensing", [
            ("check_obstacle", "obstacle_distance"),
            ("detect_colour", "colour_detection"),
        ]),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title = QLabel("FUNCTIONS")
        title.setFont(QFont("Menlo", 13, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("margin-bottom: 4px;")
        layout.addWidget(title)

        for cat_name, functions in self._CATEGORIES:
            header = QLabel(cat_name)
            header.setFont(QFont("Menlo", 11, QFont.Weight.Bold))
            header.setStyleSheet("margin-top: 8px; color: #555555;")
            layout.addWidget(header)
            for btn_label, snippet_key in functions:
                btn = DraggableFunctionButton(
                    btn_label, _SIMPLE_VIEW_SNIPPETS[snippet_key]
                )
                layout.addWidget(btn)

        layout.addStretch()


# --- Code editor with line numbers ---

class _LineNumberArea(QWidget):
    """Gutter widget that displays line numbers for a LineNumberEditor."""

    def __init__(self, editor):
        super().__init__(editor)
        self._editor = editor

    def sizeHint(self):
        return QSize(self._editor._line_area_width(), 0)

    def paintEvent(self, event):
        self._editor.line_number_area_paint(event)


class LineNumberEditor(QPlainTextEdit):
    """QPlainTextEdit with a line-number gutter on the left."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._line_area = _LineNumberArea(self)
        self.blockCountChanged.connect(self._update_line_area_width)
        self.updateRequest.connect(self._update_line_area)
        self.cursorPositionChanged.connect(self._line_area.update)
        self._update_line_area_width()

    def _line_area_width(self):
        digits = max(1, len(str(self.blockCount())))
        return int(1.5 * (10 + self.fontMetrics().horizontalAdvance('9') * digits))

    def _update_line_area_width(self, _=0):
        self.setViewportMargins(self._line_area_width(), 0, 0, 0)

    def _update_line_area(self, rect, dy):
        if dy:
            self._line_area.scroll(0, dy)
        else:
            self._line_area.update(0, rect.y(), self._line_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_line_area_width()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._line_area.setGeometry(
            cr.left(), cr.top(), self._line_area_width(), cr.height()
        )

    def line_number_area_paint(self, event):
        painter = QPainter(self._line_area)
        painter.fillRect(event.rect(), QColor("#F0F0F0"))
        current_block_number = self.textCursor().blockNumber()
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = int(
            self.blockBoundingGeometry(block).translated(self.contentOffset()).top()
        )
        bottom = top + int(self.blockBoundingRect(block).height())
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                if block_number == current_block_number:
                    painter.fillRect(
                        0, top, self._line_area.width(),
                        self.fontMetrics().height(), QColor("#D0D0FF"),
                    )
                    painter.setPen(QColor("#0000CC"))
                else:
                    painter.setPen(QColor("#999999"))
                painter.drawText(
                    0, top,
                    self._line_area.width(),
                    self.fontMetrics().height(),
                    Qt.AlignmentFlag.AlignCenter,
                    str(block_number + 1),
                )
            block = block.next()
            top = bottom
            bottom = top + int(self.blockBoundingRect(block).height())
            block_number += 1
        painter.end()


class SimpleViewEditor(LineNumberEditor):
    """Editor for Simple View — only allows drops inside the control_loop section."""

    def _logic_start_line(self):
        """Return the line number of 'def control_loop', or None."""
        for i, line in enumerate(self.toPlainText().split('\n')):
            if 'def control_loop' in line:
                return i
        return None

    def dragEnterEvent(self, event):
        """Accept drag so the cursor tracks, but actual gating is in dropEvent."""
        if event.mimeData().hasText():
            event.acceptProposedAction()
        else:
            super().dragEnterEvent(event)

    def dragMoveEvent(self, event):
        """Show forbidden cursor when hovering over the parameter section."""
        logic_line = self._logic_start_line()
        if logic_line is not None:
            hover_line = self.cursorForPosition(event.position().toPoint()).blockNumber()
            if hover_line <= logic_line:
                event.ignore()
                return
        event.acceptProposedAction()

    def dropEvent(self, event):
        """Block drops above the control_loop section."""
        logic_line = self._logic_start_line()
        if logic_line is not None:
            drop_line = self.cursorForPosition(event.position().toPoint()).blockNumber()
            if drop_line <= logic_line:
                event.ignore()
                return
        super().dropEvent(event)


# --- Syntax highlighter for Simple View ---

class SimpleCodeHighlighter(QSyntaxHighlighter):
    """Highlights editable parameter values in the Simple View code editor."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self._keyword_fmt = QTextCharFormat()
        self._keyword_fmt.setForeground(QColor("#AA00AA"))
        self._keyword_fmt.setFontWeight(QFont.Weight.Bold)

        self._value_fmt = QTextCharFormat()
        self._value_fmt.setForeground(QColor("#0055DD"))
        self._value_fmt.setFontWeight(QFont.Weight.Bold)

        self._string_fmt = QTextCharFormat()
        self._string_fmt.setForeground(QColor("#008800"))
        self._string_fmt.setFontWeight(QFont.Weight.Bold)

        self._comment_fmt = QTextCharFormat()
        self._comment_fmt.setForeground(QColor("#888888"))

        self._self_fmt = QTextCharFormat()
        self._self_fmt.setForeground(QColor("#AA5500"))
        self._self_fmt.setFontWeight(QFont.Weight.Bold)

        self._edit_marker_fmt = QTextCharFormat()
        self._edit_marker_fmt.setForeground(QColor("#FF4400"))
        self._edit_marker_fmt.setFontWeight(QFont.Weight.Bold)

        self._drop_guide_fmt = QTextCharFormat()
        self._drop_guide_fmt.setForeground(QColor("#FF0000"))
        self._drop_guide_fmt.setFontWeight(QFont.Weight.Bold)

    def highlightBlock(self, text):
        stripped = text.lstrip()

        # Full-line comments
        if stripped.startswith('#'):
            # Drag-and-drop guide line — red + bold
            if 'Drag and drop' in text:
                self.setFormat(0, len(text), self._drop_guide_fmt)
            else:
                self.setFormat(0, len(text), self._comment_fmt)
            return

        # Python keywords
        for kw in ['import', 'from', 'class', 'def', 'super']:
            for m in re.finditer(rf'\b{kw}\b', text):
                self.setFormat(m.start(), len(m.group()), self._keyword_fmt)

        # self.param_name (before =)
        for m in re.finditer(r'self\.(\w+)\s*=', text):
            self.setFormat(m.start(), len(m.group()) - 1, self._self_fmt)

        # Numeric values after =
        for m in re.finditer(r'=\s*([\d.]+)', text):
            self.setFormat(m.start(1), len(m.group(1)), self._value_fmt)

        # Quoted strings
        for m in re.finditer(r'"([^"]*)"', text):
            self.setFormat(m.start(), len(m.group()), self._string_fmt)

        # Inline comments
        idx = text.find('#')
        if idx > 0:
            self.setFormat(idx, len(text) - idx, self._comment_fmt)

        # "← edit" marker in bright orange-red (overrides comment grey)
        edit_idx = text.find('\u2190 edit')
        if edit_idx >= 0:
            self.setFormat(edit_idx, 6, self._edit_marker_fmt)


# --- Syntax highlighter for Full View ---

class FullViewHighlighter(QSyntaxHighlighter):
    """Syntax highlighting for Full View: green comments, red warning comments."""

    _WARNING_PATTERNS = [
        "do not edit", "do not delete", "do not change", "do not remove",
        "could break", "take precaution",
        "customize main function below", "end main customization",
        "note: editing any of the generated code",
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._comment_fmt = QTextCharFormat()
        self._comment_fmt.setForeground(QColor("#2E7D32"))  # green

        self._warning_fmt = QTextCharFormat()
        self._warning_fmt.setForeground(QColor("#D32F2F"))  # red

    def highlightBlock(self, text):
        stripped = text.lstrip()
        if stripped.startswith('#'):
            idx = text.index('#')
            comment_text = text[idx:]
        else:
            idx = text.find('#')
            if idx < 0:
                return
            comment_text = text[idx:]

        lower = comment_text.lower()
        for pat in self._WARNING_PATTERNS:
            if pat in lower:
                self.setFormat(idx, len(text) - idx, self._warning_fmt)
                return

        self.setFormat(idx, len(text) - idx, self._comment_fmt)


class CodeEditorDialog(QDialog):
    """Popup code editor for editing node/topic source files."""

    def __init__(self, title, file_path, search_text="", parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setMinimumSize(700, 500)
        self._file_path = file_path
        self._saved = False
        self._show_in_code = False

        layout = QVBoxLayout(self)

        # Top row: Show In Code (left) | file path | Cancel + Save (right)
        top_row = QHBoxLayout()

        show_in_code_btn = QPushButton("Show In Code")
        show_in_code_btn.setStyleSheet(
            "QPushButton { padding: 6px 16px; border-radius: 8px; "
            "border: 1px solid #007AFF; color: #007AFF; }")
        show_in_code_btn.clicked.connect(self._on_show_in_code)
        top_row.addWidget(show_in_code_btn)

        path_label = QLabel(os.path.basename(file_path))
        path_label.setStyleSheet("color: #666; font-size: 11px;")
        top_row.addWidget(path_label)
        top_row.addStretch()

        cancel_btn = QPushButton("Cancel")
        cancel_btn.setStyleSheet(
            "QPushButton { padding: 6px 16px; border-radius: 8px; border: 1px solid #ccc; }")
        cancel_btn.clicked.connect(self.reject)
        top_row.addWidget(cancel_btn)

        save_btn = QPushButton("Save")
        save_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; "
            "padding: 6px 16px; border-radius: 8px; font-weight: bold; }")
        save_btn.clicked.connect(self._save)
        top_row.addWidget(save_btn)
        layout.addLayout(top_row)

        # Code editor
        self._editor = LineNumberEditor()
        self._editor.setFont(QFont("Menlo", 12))
        self._editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._highlighter = FullViewHighlighter(self._editor.document())
        layout.addWidget(self._editor)

        # Load file + jump to search_text
        try:
            with open(file_path, "r") as f:
                self._editor.setPlainText(f.read())
        except Exception as e:
            self._editor.setPlainText(f"# Error loading file: {e}")
        if search_text:
            cursor = self._editor.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            self._editor.setTextCursor(cursor)
            self._editor.find(search_text)

    def _save(self):
        try:
            with open(self._file_path, "w") as f:
                f.write(self._editor.toPlainText())
            self._saved = True
            self.accept()
        except Exception as e:
            QMessageBox.warning(self, "Save Error", str(e))

    def _on_show_in_code(self):
        self._show_in_code = True
        self.accept()

    @property
    def saved(self):         return self._saved
    @property
    def show_in_code(self):  return self._show_in_code
    @property
    def file_path(self):     return self._file_path
    @property
    def content(self):       return self._editor.toPlainText()


# --- Node Canvas data objects ---

class CanvasItem:
    """Data object for a node or topic on the canvas."""

    def __init__(self, item_id, item_type, name, x, y, package,
                 width=None, height=None, hidden=False):
        self.item_id = item_id
        self.item_type = item_type          # "node" or "topic"
        self.name = name
        self.x = x
        self.y = y
        self.package = package
        self.width = width or (64 if item_type == "node" else 140)
        self.height = height or (64 if item_type == "node" else 40)
        self.hidden = hidden                # True = removed from canvas only

    def rect(self):
        """Return QRectF centred on (x, y)."""
        return QRectF(self.x - self.width / 2, self.y - self.height / 2,
                      self.width, self.height)

    def connector_pos(self):
        """Return QPointF at top-right corner (for the + connector button)."""
        return QPointF(self.x + self.width / 2 - 8,
                       self.y - self.height / 2 - 8)

    def to_dict(self):
        d = {"item_id": self.item_id, "item_type": self.item_type,
             "name": self.name, "x": self.x, "y": self.y,
             "package": self.package,
             "width": self.width, "height": self.height}
        if self.hidden:
            d["hidden"] = True
        return d

    @staticmethod
    def from_dict(d):
        return CanvasItem(d["item_id"], d["item_type"], d["name"],
                          d["x"], d["y"], d["package"],
                          d.get("width"), d.get("height"),
                          d.get("hidden", False))


class CanvasConnection:
    """Directed arrow between two canvas items."""

    def __init__(self, conn_id, source_id, target_id, conn_type):
        self.conn_id = conn_id
        self.source_id = source_id
        self.target_id = target_id
        self.conn_type = conn_type          # "publishes" or "subscribes"

    def to_dict(self):
        return {"conn_id": self.conn_id, "source_id": self.source_id,
                "target_id": self.target_id, "conn_type": self.conn_type}

    @staticmethod
    def from_dict(d):
        return CanvasConnection(d["conn_id"], d["source_id"],
                                d["target_id"], d["conn_type"])


class CanvasState:
    """Data model managing all items, connections, and packages."""

    def __init__(self):
        self.packages = {}          # {pkg_name: {"label": str, "color": str}}
        self.items = {}             # {item_id: CanvasItem}
        self.connections = []       # [CanvasConnection]
        self._next_id = 4

    def _new_id(self, prefix):
        id_str = f"{prefix}_{self._next_id}"
        self._next_id += 1
        return id_str

    def add_item(self, item_type, name, x, y, package):
        prefix = "node" if item_type == "node" else "topic"
        item_id = self._new_id(prefix)
        item = CanvasItem(item_id, item_type, name, x, y, package)
        self.items[item_id] = item
        return item

    def add_connection(self, source_id, target_id, conn_type):
        conn_id = self._new_id("conn")
        conn = CanvasConnection(conn_id, source_id, target_id, conn_type)
        self.connections.append(conn)
        return conn

    def remove_item(self, item_id):
        self.items.pop(item_id, None)
        self.connections = [c for c in self.connections
                            if c.source_id != item_id and c.target_id != item_id]

    def remove_connection(self, conn_id):
        self.connections = [c for c in self.connections if c.conn_id != conn_id]

    def to_dict(self):
        return {
            "version": 1,
            "next_id": self._next_id,
            "packages": self.packages,
            "items": [it.to_dict() for it in self.items.values()],
            "connections": [c.to_dict() for c in self.connections],
        }

    @staticmethod
    def from_dict(d):
        state = CanvasState()
        state._next_id = d.get("next_id", 4)
        state.packages = d.get("packages", {})
        for it_d in d.get("items", []):
            item = CanvasItem.from_dict(it_d)
            state.items[item.item_id] = item
        for c_d in d.get("connections", []):
            state.connections.append(CanvasConnection.from_dict(c_d))
        return state

    @staticmethod
    def default_movement_pkg():
        """Create a default state for the movement package."""
        state = CanvasState()
        state.packages = {
            "movement_pkg": {"label": "movement", "color": "#34C759"},
        }
        node = CanvasItem("node_1", "node", "movement", 400, 250,
                          "movement_pkg")
        topic_pub = CanvasItem("topic_2", "topic", "/cmd_vel", 650, 250,
                               "movement_pkg")
        topic_sub = CanvasItem("topic_3", "topic", "/scan", 150, 250,
                               "movement_pkg")
        state.items = {
            "node_1": node, "topic_2": topic_pub, "topic_3": topic_sub,
        }
        state.connections = [
            CanvasConnection("conn_1", "node_1", "topic_2", "publishes"),
            CanvasConnection("conn_2", "topic_3", "node_1", "subscribes"),
        ]
        state._next_id = 4
        return state


class CanvasUndoStack:
    """Stores serialized snapshots of CanvasState for undo/redo."""

    def __init__(self, max_size=50):
        self._stack = []
        self._index = -1
        self._max = max_size

    def push(self, state_dict):
        # Discard redo history beyond current index
        self._stack = self._stack[:self._index + 1]
        self._stack.append(json.dumps(state_dict))
        if len(self._stack) > self._max:
            self._stack.pop(0)
        self._index = len(self._stack) - 1

    def undo(self, current_dict):
        if self._index <= 0:
            return None
        self._index -= 1
        return json.loads(self._stack[self._index])

    def redo(self, current_dict):
        if self._index >= len(self._stack) - 1:
            return None
        self._index += 1
        return json.loads(self._stack[self._index])


class TopicDiscoveryWorker(QThread):
    """Runs `ros2 topic list` over SSH and emits discovered topics."""
    log = pyqtSignal(str)
    topics_found = pyqtSignal(list)

    def __init__(self, ssh_client):
        super().__init__()
        self.ssh = ssh_client

    def run(self):
        try:
            self.log.emit("Discovering ROS2 topics...")
            cmd = ROS_SOURCE_CMD + " && ros2 topic list 2>&1"
            _, stdout, stderr = self.ssh.exec_command(cmd, timeout=15)
            exit_code = stdout.channel.recv_exit_status()
            output = stdout.read().decode().strip()
            if exit_code == 0 and output:
                topics = [t.strip() for t in output.splitlines() if t.strip().startswith("/")]
                self.log.emit(f"Found {len(topics)} topics.")
                self.topics_found.emit(topics)
            else:
                self.log.emit("No topics found or command failed.")
                self.topics_found.emit([])
        except Exception as e:
            self.log.emit(f"Topic discovery error: {e}")
            self.topics_found.emit([])


class CanvasFileTree(QTreeWidget):
    """QTreeWidget that supports dragging nodes/topics to the canvas."""

    def startDrag(self, supportedActions):
        item = self.currentItem()
        if item is None:
            return
        data = item.data(0, Qt.ItemDataRole.UserRole)
        if data is None or data[0] not in ("node", "topic"):
            return
        kind, name = data
        drag = QDrag(self)
        mime = QMimeData()
        mime.setText(f"{kind}:{name}")
        drag.setMimeData(mime)
        drag.exec(Qt.DropAction.MoveAction)


class NodeCanvasWidget(QWidget):
    """Custom-painted canvas for visualizing ROS2 nodes and topics."""

    navigate_to_code = pyqtSignal(str, str)  # (file_rel_path, search_text)
    delete_requested = pyqtSignal(str)       # item_id
    item_dropped = pyqtSignal()              # emitted after a tree-to-canvas drop
    connection_added = pyqtSignal()           # emitted after a new connection is created
    edit_item_requested = pyqtSignal(str)    # item_id

    _GRID_SPACING = 20
    _NODE_COLOR = QColor("#007AFF")
    _TOPIC_COLOR = QColor("#007AFF")
    _CONNECTOR_RADIUS = 8
    _ARROW_SIZE = 10

    def __init__(self, canvas_state, parent=None):
        super().__init__(parent)
        self.state = canvas_state
        self._dragging_item = None
        self._drag_offset = QPointF(0, 0)
        self._connecting_from = None     # item_id when drag-to-connect
        self._connect_mouse_pos = None
        self._hovered_item = None
        self._hovered_connector = None
        self.setMouseTracking(True)
        self.setMinimumSize(600, 400)
        self.setAcceptDrops(True)
        self._active_package = None      # filter to show items from this pkg
        self._delete_mode = False         # show red X buttons on items
        self._locked_item = None          # long-pressed item showing green +
        self._long_press_timer = QTimer(self)
        self._long_press_timer.setSingleShot(True)
        self._long_press_timer.setInterval(500)
        self._long_press_timer.timeout.connect(self._on_long_press)
        self._press_item_id = None       # item under press, pending long-press
        self._press_pos = None
        self._foreign_items = set()      # item_ids from other packages shown here

    def set_active_package(self, pkg_name):
        self._active_package = pkg_name
        self._foreign_items.clear()
        # Rebuild foreign items: find cross-package items linked by connections
        pkg_ids = {iid for iid, it in self.state.items.items()
                   if it.package == pkg_name and not it.hidden}
        for conn in self.state.connections:
            src_in = conn.source_id in pkg_ids
            tgt_in = conn.target_id in pkg_ids
            if src_in and not tgt_in and conn.target_id in self.state.items:
                self._foreign_items.add(conn.target_id)
            elif tgt_in and not src_in and conn.source_id in self.state.items:
                self._foreign_items.add(conn.source_id)
        self.update()

    # --- Drag-and-drop from file tree ---

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            text = event.mimeData().text()
            if ":" in text and text.split(":")[0] in ("node", "topic"):
                event.acceptProposedAction()

    def dragMoveEvent(self, event):
        event.acceptProposedAction()

    def dropEvent(self, event):
        text = event.mimeData().text()
        kind, name = text.split(":", 1)
        pos = event.position()
        for item in self.state.items.values():
            if item.item_type == kind and item.name == name:
                item.x = pos.x()
                item.y = pos.y()
                # Unhide if it was previously removed from canvas
                item.hidden = False
                if (self._active_package
                        and item.package != self._active_package):
                    self._foreign_items.add(item.item_id)
                break
        event.acceptProposedAction()
        self.item_dropped.emit()
        self.update()

    # --- Painting ---

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # White background
        p.fillRect(self.rect(), QColor("#FFFFFF"))

        # Grid dots
        dot_pen = QPen(QColor("#E0E0E0"))
        dot_pen.setWidth(1)
        p.setPen(dot_pen)
        for gx in range(0, self.width(), self._GRID_SPACING):
            for gy in range(0, self.height(), self._GRID_SPACING):
                p.drawPoint(gx, gy)

        # Instruction text at top centre
        p.setPen(QColor("#888888"))
        _inst_font = QFont("Menlo")
        _inst_font.setPixelSize(18)
        _inst_font.setBold(True)
        p.setFont(_inst_font)
        inst_rect = QRectF(0, 8 + self.height() * 0.03, self.width(), 30)
        p.drawText(inst_rect,
                   Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop,
                   "Long press any Node or Topic before you can connect "
                   "them by dragging a line from the green plus button.")

        visible = self._visible_items()

        # Draw connections (only when both endpoints are visible)
        for conn in self.state.connections:
            src = self.state.items.get(conn.source_id)
            tgt = self.state.items.get(conn.target_id)
            if src is None or tgt is None:
                continue
            if src.item_id not in visible or tgt.item_id not in visible:
                continue
            self._draw_connection(p, src, tgt, conn.conn_type)

        # Draw items
        for item_id in visible:
            item = self.state.items[item_id]
            self._draw_item(p, item)

        # Draw package folder icon + dotted line for foreign items
        for fid in self._foreign_items:
            item = self.state.items.get(fid)
            if item is None or item.hidden:
                continue
            pkg_info = self.state.packages.get(item.package, {})
            pkg_label = pkg_info.get("label", item.package)
            pkg_color = QColor(pkg_info.get("color", "#34C759"))
            # Folder icon centred above the item
            folder_x = item.x
            folder_y = item.y - item.height / 2 - 50
            # Dotted line from folder to top of item
            pen = QPen(QColor("#888888"))
            pen.setWidth(2)
            pen.setStyle(Qt.PenStyle.DotLine)
            p.setPen(pen)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(QPointF(folder_x, folder_y + 10),
                       QPointF(item.x, item.y - item.height / 2))
            # Folder shape
            fw, fh = 28, 22
            folder_rect = QRectF(folder_x - fw / 2, folder_y - fh / 2,
                                 fw, fh)
            p.setPen(QPen(pkg_color.darker(120), 1.5))
            p.setBrush(QBrush(pkg_color))
            p.drawRoundedRect(folder_rect, 3, 3)
            # Small tab on top-left of folder
            tab_rect = QRectF(folder_rect.x(), folder_rect.y() - 5,
                              fw * 0.4, 6)
            p.drawRoundedRect(tab_rect, 2, 2)
            # Package label below folder
            p.setPen(pkg_color)
            p.setFont(QFont("Menlo", 8, QFont.Weight.Bold))
            lbl_rect = QRectF(folder_x - 60, folder_y + fh / 2 + 1,
                              120, 16)
            p.drawText(lbl_rect,
                       Qt.AlignmentFlag.AlignHCenter
                       | Qt.AlignmentFlag.AlignTop,
                       pkg_label)

        # Draw green + connector on long-pressed (locked) item only
        if not self._delete_mode and self._locked_item and self._locked_item in visible:
            item = self.state.items.get(self._locked_item)
            if item:
                self._draw_connector_button(p, item)

        # Draw red X delete buttons on all visible items in delete mode
        if self._delete_mode:
            for item_id in visible:
                item = self.state.items[item_id]
                if (item.package == _PROTECTED_PACKAGE
                        and (item.name in _PROTECTED_NODES
                             or item.name in _PROTECTED_TOPICS)):
                    continue
                self._draw_delete_button(p, item)

        # In-progress drag-to-connect line
        if self._connecting_from and self._connect_mouse_pos:
            src_item = self.state.items.get(self._connecting_from)
            if src_item:
                pen = QPen(QColor("#007AFF"))
                pen.setWidth(2)
                pen.setStyle(Qt.PenStyle.DashLine)
                p.setPen(pen)
                src_center = QPointF(src_item.x, src_item.y)
                p.drawLine(src_center, self._connect_mouse_pos)

        p.end()

    def _visible_items(self):
        """Return set of item_ids visible for the active package filter."""
        if self._active_package is None:
            return {iid for iid, it in self.state.items.items()
                    if not it.hidden}
        visible = {iid for iid, it in self.state.items.items()
                   if it.package == self._active_package and not it.hidden}
        # Include cross-package items that were dragged onto the canvas
        visible |= {iid for iid in self._foreign_items
                     if iid in self.state.items
                     and not self.state.items[iid].hidden}
        return visible

    def _draw_item(self, p, item):
        r = item.rect()
        is_node = (item.item_type == "node")

        # Fill
        fill_color = QColor("#007AFF")
        p.setBrush(QBrush(fill_color))
        pen = QPen(fill_color.darker(120))
        pen.setWidth(2)
        p.setPen(pen)

        if is_node:
            # Blue circle for nodes
            radius = min(item.width, item.height) / 2
            p.drawEllipse(QPointF(item.x, item.y), radius, radius)
        else:
            # Blue rounded rectangle for topics
            p.drawRoundedRect(r, 10, 10)

        # Text label (white on blue)
        p.setPen(QPen(QColor("#FFFFFF")))
        font = QFont("Menlo", 10, QFont.Weight.Bold)
        p.setFont(font)
        p.drawText(r, Qt.AlignmentFlag.AlignCenter, item.name)

    def _draw_connector_button(self, p, item):
        """Draw small + circle at top-right of an item."""
        pos = item.connector_pos()
        r = self._CONNECTOR_RADIUS
        p.setBrush(QBrush(QColor("#34C759")))
        p.setPen(QPen(QColor("#2AA84B")))
        p.drawEllipse(pos, r, r)
        p.setPen(QPen(QColor("#FFFFFF")))
        font = QFont("Menlo", 10, QFont.Weight.Bold)
        p.setFont(font)
        p.drawText(QRectF(pos.x() - r, pos.y() - r, r * 2, r * 2),
                   Qt.AlignmentFlag.AlignCenter, "+")

    def _delete_btn_pos(self, item):
        """Return QPointF for the red X button at top-right corner."""
        return QPointF(item.x + item.width / 2 - 2,
                       item.y - item.height / 2 - 2)

    def _draw_delete_button(self, p, item):
        """Draw red circle with white X at top-right of an item."""
        pos = self._delete_btn_pos(item)
        r = self._CONNECTOR_RADIUS + 2
        p.setBrush(QBrush(QColor("#FF3B30")))
        p.setPen(QPen(QColor("#CC2A22"), 2))
        p.drawEllipse(pos, r, r)
        # Draw X
        p.setPen(QPen(QColor("#FFFFFF"), 2))
        offset = r * 0.45
        p.drawLine(QPointF(pos.x() - offset, pos.y() - offset),
                   QPointF(pos.x() + offset, pos.y() + offset))
        p.drawLine(QPointF(pos.x() + offset, pos.y() - offset),
                   QPointF(pos.x() - offset, pos.y() + offset))

    def _delete_btn_at(self, pos):
        """Return item_id if pos hits a delete button, else None."""
        if not self._delete_mode:
            return None
        visible = self._visible_items()
        for item_id in visible:
            item = self.state.items[item_id]
            # Skip protected default items
            if (item.package == _PROTECTED_PACKAGE
                    and (item.name in _PROTECTED_NODES
                         or item.name in _PROTECTED_TOPICS)):
                continue
            bp = self._delete_btn_pos(item)
            r = self._CONNECTOR_RADIUS + 2
            if (pos - bp).manhattanLength() < r * 2.5:
                return item_id
        return None

    def _edge_point(self, item, angle):
        """Return the point on the edge of an item shape along the given angle."""
        if item.item_type == "node":
            radius = min(item.width, item.height) / 2
            return QPointF(item.x + radius * math.cos(angle),
                           item.y + radius * math.sin(angle))
        else:
            # Rounded rect — approximate with ellipse intersection
            hw = item.width / 2
            hh = item.height / 2
            ca, sa = math.cos(angle), math.sin(angle)
            # Scale to find intersection with axis-aligned ellipse
            if abs(ca) < 1e-9:
                t = hh / abs(sa)
            elif abs(sa) < 1e-9:
                t = hw / abs(ca)
            else:
                t = min(hw / abs(ca), hh / abs(sa))
            return QPointF(item.x + t * ca, item.y + t * sa)

    def _draw_connection(self, p, src, tgt, conn_type, cross_pkg=False):
        """Draw arrow from src edge to tgt edge."""
        pen = QPen(QColor("#555555"))
        pen.setWidth(2)
        if cross_pkg:
            pen.setStyle(Qt.PenStyle.DotLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)

        # Angle from source center to target center
        dx = tgt.x - src.x
        dy = tgt.y - src.y
        dist = math.hypot(dx, dy)
        if dist < 1:
            return
        angle = math.atan2(dy, dx)

        # Start at edge of source, end at edge of target
        sp = self._edge_point(src, angle)
        tp = self._edge_point(tgt, angle + math.pi)  # inward from target side

        p.drawLine(sp, tp)

        # Arrowhead at target edge
        arrow_angle = math.atan2(tp.y() - sp.y(), tp.x() - sp.x())
        arrow_p1 = QPointF(
            tp.x() - self._ARROW_SIZE * math.cos(arrow_angle - math.pi / 6),
            tp.y() - self._ARROW_SIZE * math.sin(arrow_angle - math.pi / 6),
        )
        arrow_p2 = QPointF(
            tp.x() - self._ARROW_SIZE * math.cos(arrow_angle + math.pi / 6),
            tp.y() - self._ARROW_SIZE * math.sin(arrow_angle + math.pi / 6),
        )
        p.setPen(QPen(QColor("#555555")))
        p.setBrush(QBrush(QColor("#555555")))
        p.drawPolygon(QPolygonF([tp, arrow_p1, arrow_p2]))

    # --- Hit testing ---

    def _item_at(self, pos):
        """Return item_id at pos, or None."""
        visible = self._visible_items()
        for item_id in visible:
            item = self.state.items[item_id]
            if item.rect().contains(pos):
                return item_id
        return None

    def _connector_at(self, pos):
        """Return item_id if pos is on its connector button, else None."""
        if not self._locked_item:
            return None
        visible = self._visible_items()
        if self._locked_item in visible:
            item = self.state.items.get(self._locked_item)
            if item:
                cp = item.connector_pos()
                r = self._CONNECTOR_RADIUS
                if (pos - cp).manhattanLength() < r * 2.5:
                    return item.item_id
        return None

    # --- Long press detection ---

    def _on_long_press(self):
        """Timer fired — lock the item to show its green + connector."""
        if self._press_item_id and self._press_item_id in self.state.items:
            self._locked_item = self._press_item_id
            # Cancel any drag that may have started
            self._dragging_item = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            self.update()
        self._press_item_id = None

    # --- Mouse interaction ---

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        pos = QPointF(event.position())

        # Check delete button first
        del_id = self._delete_btn_at(pos)
        if del_id:
            self.delete_requested.emit(del_id)
            return

        # Check connector on locked item — start drag-to-connect
        conn_id = self._connector_at(pos)
        if conn_id and not self._delete_mode:
            self._connecting_from = conn_id
            self._connect_mouse_pos = pos
            self.setCursor(Qt.CursorShape.CrossCursor)
            return

        # Check if clicking on an item
        item_id = self._item_at(pos)
        if item_id:
            # If this item is locked, don't drag it — just ignore
            if item_id == self._locked_item:
                return
            # Start long-press timer + allow drag as fallback
            self._press_item_id = item_id
            self._press_pos = pos
            self._long_press_timer.start()
            item = self.state.items[item_id]
            self._dragging_item = item_id
            self._drag_offset = QPointF(item.x - pos.x(), item.y - pos.y())
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
        else:
            # Clicked on empty canvas background — clear locked item
            self._locked_item = None
            self._long_press_timer.stop()
            self._press_item_id = None
            self.update()

    def mouseMoveEvent(self, event):
        pos = QPointF(event.position())

        # Drag-to-connect
        if self._connecting_from:
            self._connect_mouse_pos = pos
            self.update()
            return

        # If user moved significantly, cancel long-press (they want to drag)
        if self._press_pos is not None:
            if (pos - self._press_pos).manhattanLength() > 10:
                self._long_press_timer.stop()
                self._press_item_id = None
                self._press_pos = None

        # Dragging item (only if not locked)
        if self._dragging_item and self._dragging_item != self._locked_item:
            item = self.state.items.get(self._dragging_item)
            if item:
                item.x = pos.x() + self._drag_offset.x()
                item.y = pos.y() + self._drag_offset.y()
                self.update()
            return

        # Hover detection
        prev = self._hovered_item
        self._hovered_item = self._item_at(pos)
        if self._hovered_item != prev:
            self.update()

        # Cursor
        if self._connector_at(pos):
            self.setCursor(Qt.CursorShape.CrossCursor)
        elif self._hovered_item:
            self.setCursor(Qt.CursorShape.OpenHandCursor)
        else:
            self.setCursor(Qt.CursorShape.ArrowCursor)

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        pos = QPointF(event.position())

        # Stop long-press timer
        self._long_press_timer.stop()
        self._press_item_id = None
        self._press_pos = None

        # Finish drag-to-connect
        if self._connecting_from:
            target_id = self._item_at(pos)
            if target_id and target_id != self._connecting_from:
                self._show_connection_dialog(self._connecting_from, target_id)
            self._connecting_from = None
            self._connect_mouse_pos = None
            self.setCursor(Qt.CursorShape.ArrowCursor)
            self.update()
            return

        # Finish dragging item
        if self._dragging_item:
            self._dragging_item = None
            self.setCursor(Qt.CursorShape.OpenHandCursor)
            self.update()

    def mouseDoubleClickEvent(self, event):
        pos = QPointF(event.position())
        item_id = self._item_at(pos)
        if not item_id:
            return
        self.edit_item_requested.emit(item_id)

    def _show_connection_dialog(self, src_id, tgt_id):
        """Show dialog to choose connection type between two items."""
        src = self.state.items.get(src_id)
        tgt = self.state.items.get(tgt_id)
        if not src or not tgt:
            return

        both_nodes = (src.item_type == "node" and tgt.item_type == "node")

        if both_nodes:
            # Node-to-node: prompt for topic name first
            topic_name, ok = QInputDialog.getText(
                self, "New Topic",
                "Enter topic name (e.g. /my_topic):")
            if not ok or not topic_name.strip():
                return
            topic_name = topic_name.strip()
            if not topic_name.startswith("/"):
                topic_name = "/" + topic_name
            # Create the topic between them
            mid_x = (src.x + tgt.x) / 2
            mid_y = (src.y + tgt.y) / 2 - 60
            new_topic = self.state.add_item("topic", topic_name, mid_x, mid_y,
                                            src.package)
            # src publishes to topic, topic feeds tgt
            self.state.add_connection(src_id, new_topic.item_id, "publishes")
            self.state.add_connection(new_topic.item_id, tgt_id, "subscribes")
        else:
            # Node-topic or topic-node connection
            items = ["Add Publisher (node → topic)",
                     "Add Subscriber (topic → node)",
                     "Add Both", "Cancel"]
            choice, ok = QInputDialog.getItem(
                self, "Connection Type",
                f"Connect {src.name} ↔ {tgt.name}:",
                items, 0, False)
            if not ok or choice == "Cancel":
                return

            # Figure out which is node and which is topic
            node_id = src_id if src.item_type == "node" else tgt_id
            topic_id = tgt_id if src.item_type == "node" else src_id

            if "Publisher" in choice or "Both" in choice:
                self.state.add_connection(node_id, topic_id, "publishes")
            if "Subscriber" in choice or "Both" in choice:
                self.state.add_connection(topic_id, node_id, "subscribes")

        self.connection_added.emit()
        self.update()


# ---------------------------------------------------------------------------
# Git integration helpers
# ---------------------------------------------------------------------------

def _make_github_icon(size=20, color="#333333"):
    """Return a QIcon containing the GitHub Invertocat mark rendered at *size*×*size*."""
    # GitHub mark SVG path (24×24 viewBox, MIT-licensed mark)
    SVG_D = (
        "M12 .297c-6.63 0-12 5.373-12 12 0 5.303 3.438 9.8 8.205 11.385"
        ".6.113.82-.258.82-.577 0-.285-.01-1.04-.015-2.04-3.338.724"
        "-4.042-1.61-4.042-1.61C4.422 18.07 3.633 17.7 3.633 17.7"
        "c-1.087-.744.084-.729.084-.729 1.205.084 1.838 1.236 1.838 1.236"
        " 1.07 1.835 2.809 1.305 3.495.998.108-.776.417-1.305.76-1.605"
        "-2.665-.3-5.466-1.332-5.466-5.93 0-1.31.465-2.38 1.235-3.22"
        "-.135-.303-.54-1.523.105-3.176 0 0 1.005-.322 3.3 1.23"
        ".96-.267 1.98-.399 3-.405 1.02.006 2.04.138 3 .405"
        " 2.28-1.552 3.285-1.23 3.285-1.23.645 1.653.24 2.873.12 3.176"
        ".765.84 1.23 1.91 1.23 3.22 0 4.61-2.805 5.625-5.475 5.92"
        ".42.36.81 1.096.81 2.22 0 1.606-.015 2.896-.015 3.286"
        " 0 .315.21.69.825.57C20.565 22.092 24 17.592 24 12.297"
        "c0-6.627-5.373-12-12-12"
    )

    # Parse SVG path into QPainterPath
    path = QPainterPath()
    tokens = re.findall(
        r'[MmCcLlZz]|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', SVG_D)
    i = 0
    cmd = None
    cx = cy = sx = sy = 0.0

    def nf():
        nonlocal i
        v = float(tokens[i]); i += 1; return v

    while i < len(tokens):
        if tokens[i] in 'MmCcLlZz':
            cmd = tokens[i]; i += 1; continue
        if cmd == 'M':
            x, y = nf(), nf(); path.moveTo(x, y); cx = x; cy = y; sx = x; sy = y; cmd = 'L'
        elif cmd == 'm':
            x = cx + nf(); y = cy + nf(); path.moveTo(x, y); cx = x; cy = y; sx = x; sy = y; cmd = 'l'
        elif cmd == 'L':
            x, y = nf(), nf(); path.lineTo(x, y); cx = x; cy = y
        elif cmd == 'l':
            x = cx + nf(); y = cy + nf(); path.lineTo(x, y); cx = x; cy = y
        elif cmd == 'C':
            x1, y1, x2, y2, x, y = nf(), nf(), nf(), nf(), nf(), nf()
            path.cubicTo(x1, y1, x2, y2, x, y); cx = x; cy = y
        elif cmd == 'c':
            x1 = cx + nf(); y1 = cy + nf()
            x2 = cx + nf(); y2 = cy + nf()
            x  = cx + nf(); y  = cy + nf()
            path.cubicTo(x1, y1, x2, y2, x, y); cx = x; cy = y
        elif cmd in 'Zz':
            path.closeSubpath(); cx = sx; cy = sy; cmd = None
        else:
            i += 1

    # Render path into a QPixmap
    px = QPixmap(size, size)
    px.fill(Qt.GlobalColor.transparent)
    painter = QPainter(px)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    scale = size / 24.0
    painter.scale(scale, scale)
    painter.fillPath(path, QBrush(QColor(color)))
    painter.end()
    return QIcon(px)


class GitHubButton(QPushButton):
    """Circular button (same style as the ? help button) with a GitHub mark icon."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(36, 36)
        self.setIcon(_make_github_icon(20, "#333333"))
        self.setIconSize(QSize(20, 20))
        self.setToolTip("Git / GitHub")
        self.setStyleSheet(
            "QPushButton { background-color: white; border-radius: 18px; "
            "border: 1px solid #CCCCCC; }"
            "QPushButton:hover { background-color: #F0F0F0; }"
        )


# ---------------------------------------------------------------------------
# Git dialogs
# ---------------------------------------------------------------------------

class GitInitDialog(QDialog):
    """Dialog: Initialize local git repo + create GitHub repository via API."""

    def __init__(self, creds, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Initialize & Create GitHub Repo")
        self.setMinimumWidth(460)
        self._result_creds = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(14)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        form.setSpacing(10)

        self._user  = QLineEdit(creds.get("username", ""))
        self._user.setPlaceholderText("your-github-username")

        # PAT row: field + help link
        pat_row = QHBoxLayout()
        self._pat = QLineEdit(creds.get("token", ""))
        self._pat.setEchoMode(QLineEdit.EchoMode.Password)
        self._pat.setPlaceholderText("ghp_xxxxxxxxxxxxxxxxxxxx")
        pat_help = QPushButton("?")
        pat_help.setFixedSize(22, 22)
        pat_help.setStyleSheet(
            "QPushButton { background: white; border-radius: 11px; "
            "border: 1px solid #ccc; font-weight: bold; font-size: 11px; color: #555; }"
            "QPushButton:hover { background: #f0f0f0; }"
        )
        pat_help.setToolTip("How to create a GitHub Personal Access Token")
        pat_help.clicked.connect(lambda: subprocess.Popen(
            ["open", "https://github.com/settings/tokens/new"
             "?description=TestDrive&scopes=repo"]))
        pat_row.addWidget(self._pat)
        pat_row.addWidget(pat_help)

        self._repo = QLineEdit(creds.get("repo_name", ""))
        self._repo.setPlaceholderText("my-robot-project")

        self._desc = QLineEdit(creds.get("description", ""))
        self._desc.setPlaceholderText("Optional description")

        form.addRow("GitHub Username:", self._user)
        form.addRow("Personal Access Token:", pat_row)
        form.addRow("Repository Name:", self._repo)
        form.addRow("Description:", self._desc)
        layout.addLayout(form)

        # Public / Private toggle
        vis_row = QHBoxLayout()
        vis_label = QLabel("Visibility:")
        vis_label.setFixedWidth(140)
        vis_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self._pub_btn  = QPushButton("Public")
        self._priv_btn = QPushButton("Private")
        for btn in (self._pub_btn, self._priv_btn):
            btn.setCheckable(True)
            btn.setFixedWidth(90)
        self._pub_btn.setChecked(True)
        self._pub_btn.clicked.connect(lambda: self._set_vis(False))
        self._priv_btn.clicked.connect(lambda: self._set_vis(True))
        self._private = False
        self._update_vis_style()
        vis_row.addWidget(vis_label)
        vis_row.addWidget(self._pub_btn)
        vis_row.addWidget(self._priv_btn)
        vis_row.addStretch()
        layout.addLayout(vis_row)

        # Options
        self._readme_cb = QCheckBox("Create README.md")
        self._readme_cb.setChecked(True)
        self._save_cb   = QCheckBox("Save credentials for this session")
        self._save_cb.setChecked(creds.get("save", True))
        layout.addWidget(self._readme_cb)
        layout.addWidget(self._save_cb)

        # Buttons
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        cancel_btn = QPushButton("Cancel")
        cancel_btn.setFixedWidth(90)
        cancel_btn.setStyleSheet(
            "QPushButton { padding: 8px; border-radius: 8px; border: 1px solid #ccc; }")
        cancel_btn.clicked.connect(self.reject)
        create_btn = QPushButton("Create")
        create_btn.setFixedWidth(90)
        create_btn.setStyleSheet(
            "QPushButton { background-color: #2DA44E; color: white; "
            "padding: 8px; border-radius: 8px; font-weight: bold; }"
            "QPushButton:hover { background-color: #218A41; }"
        )
        create_btn.clicked.connect(self._accept)
        btn_row.addWidget(cancel_btn)
        btn_row.addWidget(create_btn)
        layout.addLayout(btn_row)

    def _set_vis(self, private: bool):
        self._private = private
        self._pub_btn.setChecked(not private)
        self._priv_btn.setChecked(private)
        self._update_vis_style()

    def _update_vis_style(self):
        active   = "background-color: #0969DA; color: white; padding: 6px; border-radius: 6px; font-weight: bold;"
        inactive = "background-color: #f6f8fa; color: #333; padding: 6px; border-radius: 6px; border: 1px solid #ccc;"
        self._pub_btn.setStyleSheet(active if not self._private else inactive)
        self._priv_btn.setStyleSheet(active if self._private else inactive)

    def _accept(self):
        if not self._user.text().strip():
            QMessageBox.warning(self, "Missing Field", "GitHub Username is required."); return
        if not self._pat.text().strip():
            QMessageBox.warning(self, "Missing Field", "Personal Access Token is required."); return
        if not self._repo.text().strip():
            QMessageBox.warning(self, "Missing Field", "Repository Name is required."); return
        self._result_creds = {
            "username":    self._user.text().strip(),
            "token":       self._pat.text().strip(),
            "repo_name":   self._repo.text().strip(),
            "description": self._desc.text().strip(),
            "private":     self._private,
            "readme":      self._readme_cb.isChecked(),
            "save":        self._save_cb.isChecked(),
        }
        self.accept()

    def result_creds(self):
        return self._result_creds


class GitPushDialog(QDialog):
    """Dialog: Commit + push to GitHub."""

    def __init__(self, creds, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Commit & Push to GitHub")
        self.setMinimumWidth(460)
        self._result = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(14)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        form.setSpacing(10)

        default_msg = f"TestDrive update {time.strftime('%Y-%m-%d %H:%M')}"
        self._msg  = QLineEdit(default_msg)

        # Build default repo URL from saved creds
        saved_user = creds.get("username", "")
        saved_repo = creds.get("repo_name", "")
        default_repo_url = (
            f"https://github.com/{saved_user}/{saved_repo}"
            if saved_user and saved_repo else ""
        )
        self._repo_url = QLineEdit(default_repo_url)
        self._repo_url.setPlaceholderText("https://github.com/username/repo")

        # PAT row
        pat_row = QHBoxLayout()
        self._pat = QLineEdit(creds.get("token", ""))
        self._pat.setEchoMode(QLineEdit.EchoMode.Password)
        self._pat.setPlaceholderText("ghp_xxxxxxxxxxxxxxxxxxxx")
        pat_help = QPushButton("?")
        pat_help.setFixedSize(22, 22)
        pat_help.setStyleSheet(
            "QPushButton { background: white; border-radius: 11px; "
            "border: 1px solid #ccc; font-weight: bold; font-size: 11px; color: #555; }"
            "QPushButton:hover { background: #f0f0f0; }"
        )
        pat_help.setToolTip("How to create a GitHub Personal Access Token")
        pat_help.clicked.connect(lambda: subprocess.Popen(
            ["open", "https://github.com/settings/tokens/new"
             "?description=TestDrive&scopes=repo"]))
        pat_row.addWidget(self._pat)
        pat_row.addWidget(pat_help)

        self._branch = QComboBox()
        self._branch.addItems(["main", "windows", "roboapps"])
        saved_branch = creds.get("branch", "main")
        idx = self._branch.findText(saved_branch)
        self._branch.setCurrentIndex(idx if idx >= 0 else 0)

        form.addRow("Commit Message:", self._msg)
        form.addRow("GitHub Repository:", self._repo_url)
        form.addRow("Branch:", self._branch)
        form.addRow("Personal Access Token:", pat_row)
        layout.addLayout(form)

        self._save_cb = QCheckBox("Save credentials for this session")
        self._save_cb.setChecked(creds.get("save", True))
        layout.addWidget(self._save_cb)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        cancel_btn = QPushButton("Cancel")
        cancel_btn.setFixedWidth(90)
        cancel_btn.setStyleSheet(
            "QPushButton { padding: 8px; border-radius: 8px; border: 1px solid #ccc; }")
        cancel_btn.clicked.connect(self.reject)
        push_btn = QPushButton("Push")
        push_btn.setFixedWidth(90)
        push_btn.setStyleSheet(
            "QPushButton { background-color: #0969DA; color: white; "
            "padding: 8px; border-radius: 8px; font-weight: bold; }"
            "QPushButton:hover { background-color: #0757BA; }"
        )
        push_btn.clicked.connect(self._accept)
        btn_row.addWidget(cancel_btn)
        btn_row.addWidget(push_btn)
        layout.addLayout(btn_row)

    def _accept(self):
        if not self._repo_url.text().strip():
            QMessageBox.warning(self, "Missing Field", "GitHub Repository URL is required."); return
        if not self._pat.text().strip():
            QMessageBox.warning(self, "Missing Field", "Personal Access Token is required."); return
        self._result = {
            "message":  self._msg.text().strip() or f"TestDrive update {time.strftime('%Y-%m-%d %H:%M')}",
            "repo_url": self._repo_url.text().strip(),
            "branch":   self._branch.currentText(),
            "token":    self._pat.text().strip(),
            "save":     self._save_cb.isChecked(),
        }
        self.accept()

    def result_data(self):
        return self._result


class GitPullDialog(QDialog):
    """Dialog: Select branch and pull from GitHub."""

    def __init__(self, creds, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Pull from GitHub")
        self.setMinimumWidth(320)
        self._result = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(14)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        form.setSpacing(10)

        self._branch = QComboBox()
        self._branch.addItems(["main", "windows", "roboapps"])
        saved_branch = creds.get("branch", "main")
        idx = self._branch.findText(saved_branch)
        self._branch.setCurrentIndex(idx if idx >= 0 else 0)

        form.addRow("Branch:", self._branch)
        layout.addLayout(form)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        cancel_btn = QPushButton("Cancel")
        cancel_btn.setFixedWidth(90)
        cancel_btn.setStyleSheet(
            "QPushButton { padding: 8px; border-radius: 8px; border: 1px solid #ccc; }")
        cancel_btn.clicked.connect(self.reject)
        pull_btn = QPushButton("Pull")
        pull_btn.setFixedWidth(90)
        pull_btn.setStyleSheet(
            "QPushButton { background-color: #0969DA; color: white; "
            "padding: 8px; border-radius: 8px; font-weight: bold; }"
            "QPushButton:hover { background-color: #0757BA; }"
        )
        pull_btn.clicked.connect(self._accept)
        btn_row.addWidget(cancel_btn)
        btn_row.addWidget(pull_btn)
        layout.addLayout(btn_row)

    def _accept(self):
        self._result = {"branch": self._branch.currentText()}
        self.accept()

    def result_data(self):
        return self._result


# --- Main window ---

class RobotControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TestDrive")
        self.setMinimumWidth(600)
        self._workers = []
        self.ssh_client = None
        self._rviz_process = None
        self._gazebo_process = None
        self._robosim_process = None
        self._sim_dialog = None
        self._syncing = False
        self._full_view_current_file = None
        self._fv_edit_mode = False
        self._blocking_item_changed = False

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # --- Header row with Git / ? / RViz / Gazebo buttons ---
        header_row = QHBoxLayout()

        self.git_btn = GitHubButton()
        self.git_btn.clicked.connect(self._show_git_menu)
        header_row.addWidget(self.git_btn)

        header_row.addSpacing(6)

        self.support_btn = QPushButton("?")
        self.support_btn.setFixedSize(36, 36)
        self.support_btn.setStyleSheet(
            "QPushButton { background-color: white; color: #333333; "
            "border-radius: 18px; border: 1px solid #CCCCCC; "
            "font-size: 16px; font-weight: bold; }"
            "QPushButton:hover { background-color: #F0F0F0; }"
        )
        self.support_btn.setToolTip("Contact Support")
        self.support_btn.clicked.connect(self._show_support_dialog)
        header_row.addWidget(self.support_btn)

        header_row.addStretch()
        self.rviz_btn = QPushButton("RViz")
        self.rviz_btn.setFixedWidth(90)
        self.rviz_btn.setStyleSheet(
            "background-color: #5856D6; color: white; padding: 8px; "
            "border-radius: 8px; font-weight: bold;"
        )
        self.rviz_btn.setToolTip("Launch RViz2 to visualize robot topics")
        self.rviz_btn.clicked.connect(self._launch_rviz)
        header_row.addWidget(self.rviz_btn)

        self.gazebo_btn = QPushButton("Gazebo")
        self.gazebo_btn.setFixedWidth(90)
        self.gazebo_btn.setStyleSheet(
            "background-color: #FF9500; color: white; padding: 8px; "
            "border-radius: 8px; font-weight: bold;"
        )
        self.gazebo_btn.setToolTip("Launch Gazebo simulation with robot model")
        self.gazebo_btn.clicked.connect(self._launch_gazebo)
        header_row.addWidget(self.gazebo_btn)

        main_layout.addLayout(header_row)

        # --- Tabs ---
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)

        self._build_robot_control_tab()
        self._build_node_canvas_tab()
        self._build_code_editor_tab()
        self._build_roboapps_tab()

        self._refresh_profile_combo()
        self._load_params()

    # ------------------------------------------------------------------ #
    #  Tab 1: Robot Control                                                #
    # ------------------------------------------------------------------ #

    def _build_robot_control_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # --- Connection group ---
        conn_group = QGroupBox("Robot Connection")
        conn_layout = QGridLayout()

        conn_layout.addWidget(QLabel("IP Address:"), 0, 0)
        self.ip_input = QLineEdit(DEFAULT_IP)
        conn_layout.addWidget(self.ip_input, 0, 1)

        self.profile_combo = QComboBox()
        self.profile_combo.setMinimumWidth(240)
        self.profile_combo.currentIndexChanged.connect(self._on_profile_selected)
        conn_layout.addWidget(self.profile_combo, 0, 2)

        conn_layout.addWidget(QLabel("Username:"), 1, 0)
        self.user_input = QLineEdit("wheeltec")
        conn_layout.addWidget(self.user_input, 1, 1)

        conn_layout.addWidget(QLabel("Password:"), 2, 0)
        self.pass_input = QLineEdit("dongguan")
        self.pass_input.setEchoMode(QLineEdit.EchoMode.Password)
        conn_layout.addWidget(self.pass_input, 2, 1)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setStyleSheet("background-color: #007AFF; color: white; padding: 8px; border-radius: 8px;")
        self.connect_btn.setMinimumWidth(100)
        self.connect_btn.clicked.connect(self.do_connect)
        conn_layout.addWidget(self.connect_btn, 3, 0)

        self.conn_status = QLabel("Disconnected")
        self.conn_status.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.conn_status, 3, 1)

        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # --- Parameters group ---
        param_group = QGroupBox("Parameters")
        param_layout = QGridLayout()

        # Left column: first four parameters
        param_layout.addWidget(QLabel("Forward Speed (m/s):"), 0, 0)
        self.forward_speed = QDoubleSpinBox()
        self.forward_speed.setRange(0.01, 2.0)
        self.forward_speed.setSingleStep(0.05)
        self.forward_speed.setDecimals(2)
        self.forward_speed.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.forward_speed, 0, 1)

        param_layout.addWidget(QLabel("Backward Speed (m/s):"), 1, 0)
        self.backward_speed = QDoubleSpinBox()
        self.backward_speed.setRange(0.01, 2.0)
        self.backward_speed.setSingleStep(0.05)
        self.backward_speed.setDecimals(2)
        self.backward_speed.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.backward_speed, 1, 1)

        param_layout.addWidget(QLabel("Turn Speed (rad/s):"), 2, 0)
        self.turn_speed = QDoubleSpinBox()
        self.turn_speed.setRange(0.1, 3.0)
        self.turn_speed.setSingleStep(0.1)
        self.turn_speed.setDecimals(2)
        self.turn_speed.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.turn_speed, 2, 1)

        param_layout.addWidget(QLabel("Obstacle Distance (m):"), 3, 0)
        self.obstacle_distance = QDoubleSpinBox()
        self.obstacle_distance.setRange(0.10, 2.0)
        self.obstacle_distance.setSingleStep(0.05)
        self.obstacle_distance.setDecimals(2)
        self.obstacle_distance.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.obstacle_distance, 3, 1)

        # Right column: last three parameters aligned with first three rows
        param_layout.addWidget(QLabel("Turn Clockwise (deg):"), 0, 2)
        self.turn_cw = QDoubleSpinBox()
        self.turn_cw.setRange(0, 360)
        self.turn_cw.setSingleStep(5)
        self.turn_cw.setDecimals(1)
        self.turn_cw.setValue(90.0)
        self.turn_cw.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.turn_cw, 0, 3)

        param_layout.addWidget(QLabel("Turn Anti-Clockwise (deg):"), 1, 2)
        self.turn_acw = QDoubleSpinBox()
        self.turn_acw.setRange(0, 360)
        self.turn_acw.setSingleStep(5)
        self.turn_acw.setDecimals(1)
        self.turn_acw.setValue(90.0)
        self.turn_acw.valueChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.turn_acw, 1, 3)

        param_layout.addWidget(QLabel("Colour Detection:"), 2, 2)
        self.colour_detection = QComboBox()
        self.colour_detection.addItems(["Red", "Blue", "Yellow", "Green"])
        self.colour_detection.currentTextChanged.connect(self._sync_simple_view_from_spinboxes)
        param_layout.addWidget(self.colour_detection, 2, 3)

        _btn_row = QHBoxLayout()
        _btn_row.setSpacing(8)
        _btn_row.setContentsMargins(0, 0, 0, 0)

        self.save_btn = QPushButton("Save")
        self.save_btn.setStyleSheet(
            "background-color: #34C759; color: white; padding: 8px; border-radius: 8px;"
        )
        self.save_btn.setMinimumWidth(100)
        self.save_btn.clicked.connect(self.save)
        _btn_row.addWidget(self.save_btn)

        self.deploy_btn = QPushButton("Deploy")
        self.deploy_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; padding: 8px; border-radius: 8px; }"
            "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }"
        )
        self.deploy_btn.setMinimumWidth(100)
        self.deploy_btn.clicked.connect(self.deploy)
        self.deploy_btn.setEnabled(False)
        _btn_row.addWidget(self.deploy_btn)

        _btn_row.addStretch()
        _btn_container = QWidget()
        _btn_container.setLayout(_btn_row)
        param_layout.addWidget(_btn_container, 4, 0, 1, 4)

        param_group.setLayout(param_layout)
        layout.addWidget(param_group)

        # --- Robot control group ---
        control_group = QGroupBox("Robot Control")
        control_layout = QVBoxLayout()

        btn_row = QHBoxLayout()
        btn_row.addStretch()

        for btn_text, btn_style, btn_slot, btn_attr in [
            ("Start All",
             "QPushButton { background-color: #34C759; color: white; padding: 8px; border-radius: 8px; }"
             "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }",
             self.start_all, "start_all_btn"),
            ("Stop All",
             "QPushButton { background-color: #FF3B30; color: white; padding: 8px; border-radius: 8px; }"
             "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }",
             self.stop_all, "stop_all_btn"),
            ("Launch Files",
             "QPushButton { background-color: #007AFF; color: white; padding: 8px; border-radius: 8px; }"
             "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }",
             self._show_launch_file_dialog, "launch_file_btn"),
            ("Pause Robot",
             "QPushButton { background-color: #FF9500; color: white; padding: 8px; border-radius: 8px; }"
             "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }",
             self.pause_movement, "pause_btn"),
        ]:
            btn = QPushButton(btn_text)
            btn.setFixedWidth(130)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(btn_slot)
            btn.setEnabled(False)
            setattr(self, btn_attr, btn)
            btn_row.addWidget(btn)

        self.launch_file_btn.setEnabled(True)  # always active
        btn_row.addStretch()
        control_layout.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        self.check_nodes_btn = QPushButton("Check ROS2 Nodes")
        self.check_nodes_btn.clicked.connect(self.check_ros2_nodes)
        self.check_nodes_btn.setEnabled(False)
        btn_row2.addWidget(self.check_nodes_btn)

        self.check_topics_btn = QPushButton("Check ROS2 Topics")
        self.check_topics_btn.clicked.connect(self.check_ros2_topics)
        self.check_topics_btn.setEnabled(False)
        btn_row2.addWidget(self.check_topics_btn)

        self.check_logs_btn = QPushButton("Check Logs")
        self.check_logs_btn.clicked.connect(self.check_launch_logs)
        self.check_logs_btn.setEnabled(True)   # always active
        btn_row2.addWidget(self.check_logs_btn)

        control_layout.addLayout(btn_row2)

        self.node_labels = {}
        self.node_start_btns = {}
        self.node_stop_btns = {}

        for name, cmd in NODES:
            row = QHBoxLayout()
            label = QLabel(f"  {name}")
            label.setMinimumWidth(100)
            status = QLabel("unknown")
            status.setMinimumWidth(80)
            self.node_labels[name] = status

            start_btn = QPushButton("Start")
            start_btn.setEnabled(False)
            start_btn.clicked.connect(lambda checked, n=name, c=cmd: self.start_node(n, c))
            self.node_start_btns[name] = start_btn

            stop_btn = QPushButton("Stop")
            stop_btn.setEnabled(False)
            stop_btn.clicked.connect(lambda checked, n=name: self.stop_node(n))
            self.node_stop_btns[name] = stop_btn

            row.addWidget(label)
            row.addWidget(status)
            row.addWidget(start_btn)
            row.addWidget(stop_btn)
            control_layout.addLayout(row)

        self._control_layout = control_layout
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # --- Log area ---
        log_group = QGroupBox("Log Output")
        log_layout = QVBoxLayout()
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setFont(QFont("Menlo", 11))
        self.log_area.setMinimumHeight(225)
        log_layout.addWidget(self.log_area)
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)

        self.tabs.addTab(tab, "Robot Control")

    # ------------------------------------------------------------------ #
    #  Tab 2: Node Canvas                                                  #
    # ------------------------------------------------------------------ #

    def _build_node_canvas_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # --- Top bar ---
        top_bar = QHBoxLayout()

        # Undo / Redo (high-contrast circular style)
        _circle_btn_style = (
            "QPushButton { font-size: 20px; font-weight: bold;"
            "  min-width: 36px; max-width: 36px;"
            "  min-height: 36px; max-height: 36px; border-radius: 18px;"
            "  background-color: #007AFF; color: white;"
            "  border: 2px solid #005ECB; }"
            "QPushButton:hover { background-color: #005ECB; }"
        )
        self.canvas_undo_btn = QPushButton("\u21BA")
        self.canvas_undo_btn.setToolTip("Undo")
        self.canvas_undo_btn.setStyleSheet(_circle_btn_style)
        self.canvas_undo_btn.clicked.connect(self._canvas_undo)
        top_bar.addWidget(self.canvas_undo_btn)

        self.canvas_redo_btn = QPushButton("\u21BB")
        self.canvas_redo_btn.setToolTip("Redo")
        self.canvas_redo_btn.setStyleSheet(_circle_btn_style)
        self.canvas_redo_btn.clicked.connect(self._canvas_redo)
        top_bar.addWidget(self.canvas_redo_btn)

        top_bar.addStretch()

        # Package name label (center)
        self.canvas_pkg_label = QLabel("movement")
        self.canvas_pkg_label.setFont(QFont("Menlo", 26, QFont.Weight.Bold))
        self.canvas_pkg_label.setStyleSheet("color: #34C759;")
        self.canvas_pkg_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_bar.addWidget(self.canvas_pkg_label)

        top_bar.addStretch()

        # + button (high-contrast circular style)
        self.canvas_add_btn = QPushButton("+")
        self.canvas_add_btn.setToolTip("Add node, topic, or package")
        self.canvas_add_btn.setStyleSheet(_circle_btn_style)
        self.canvas_add_btn.clicked.connect(self._canvas_add_menu)
        top_bar.addWidget(self.canvas_add_btn)

        # - button (high-contrast circular style)
        _delete_btn_style = (
            "QPushButton { font-size: 20px; font-weight: bold;"
            "  min-width: 36px; max-width: 36px;"
            "  min-height: 36px; max-height: 36px; border-radius: 18px;"
            "  background-color: #FF3B30; color: white;"
            "  border: 2px solid #CC2A22; }"
            "QPushButton:hover { background-color: #CC2A22; }"
            "QPushButton:checked { background-color: #CC2A22;"
            "  border: 2px solid #FFFFFF; }"
        )
        self.canvas_delete_btn = QPushButton("\u2212")
        self.canvas_delete_btn.setToolTip("Delete mode — click red X to remove items")
        self.canvas_delete_btn.setCheckable(True)
        self.canvas_delete_btn.setStyleSheet(_delete_btn_style)
        self.canvas_delete_btn.clicked.connect(self._canvas_toggle_delete_mode)
        top_bar.addWidget(self.canvas_delete_btn)

        # Save button (always enabled)
        self.canvas_save_btn = QPushButton("Save")
        self.canvas_save_btn.setStyleSheet(
            "QPushButton { background-color: #34C759; color: white; padding: 6px 14px; border-radius: 8px; }"
        )
        self.canvas_save_btn.setMinimumWidth(100)
        self.canvas_save_btn.clicked.connect(self.save)
        top_bar.addWidget(self.canvas_save_btn)

        # Deploy button (requires SSH connection)
        self.canvas_deploy_btn = QPushButton("Deploy")
        self.canvas_deploy_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; padding: 6px 14px; border-radius: 8px; }"
            "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }"
        )
        self.canvas_deploy_btn.setMinimumWidth(100)
        self.canvas_deploy_btn.setEnabled(False)
        self.canvas_deploy_btn.clicked.connect(self.deploy)
        top_bar.addWidget(self.canvas_deploy_btn)

        layout.addLayout(top_bar)

        # --- Body: splitter with file tree + canvas ---
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left pane: FILES tree + add button
        left_pane = QWidget()
        left_layout = QVBoxLayout(left_pane)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        # Header label for the FILES tree
        canvas_tree_lbl = QLabel("FILES")
        canvas_tree_lbl.setFont(QFont("Menlo", 11, QFont.Weight.Bold))
        canvas_tree_lbl.setContentsMargins(4, 2, 4, 2)
        left_layout.addWidget(canvas_tree_lbl)

        self.canvas_file_tree = CanvasFileTree()
        self.canvas_file_tree.setHeaderHidden(True)
        self.canvas_file_tree.setFont(QFont("Menlo", 11))
        self.canvas_file_tree.setMinimumWidth(150)
        self.canvas_file_tree.setDragEnabled(True)
        self.canvas_file_tree.setDragDropMode(
            QAbstractItemView.DragDropMode.DragOnly)
        self.canvas_file_tree.itemClicked.connect(self._canvas_file_tree_clicked)
        self.canvas_file_tree.itemDoubleClicked.connect(self._canvas_tree_double_clicked)
        self.canvas_file_tree.itemChanged.connect(self._canvas_tree_item_changed)
        left_layout.addWidget(self.canvas_file_tree)

        pane_btn_row = QHBoxLayout()
        pane_btn_row.addStretch()
        self.canvas_pane_add_btn = QPushButton("+")
        self.canvas_pane_add_btn.setToolTip("Add node, topic, or package")
        self.canvas_pane_add_btn.setStyleSheet(_circle_btn_style)
        self.canvas_pane_add_btn.clicked.connect(self._canvas_add_menu)
        pane_btn_row.addWidget(self.canvas_pane_add_btn)

        self.canvas_pane_delete_btn = QPushButton("\u2212")
        self.canvas_pane_delete_btn.setToolTip("Delete mode")
        self.canvas_pane_delete_btn.setCheckable(True)
        self.canvas_pane_delete_btn.setStyleSheet(_delete_btn_style)
        self.canvas_pane_delete_btn.clicked.connect(self._canvas_toggle_delete_mode)
        pane_btn_row.addWidget(self.canvas_pane_delete_btn)
        pane_btn_row.addStretch()
        left_layout.addLayout(pane_btn_row)

        splitter.addWidget(left_pane)

        # Right pane: NodeCanvasWidget
        self._canvas_state = self._load_canvas_state()
        self._canvas_undo_stack = CanvasUndoStack()
        self._canvas_undo_stack.push(self._canvas_state.to_dict())
        self.canvas_widget = NodeCanvasWidget(self._canvas_state)
        self.canvas_widget.navigate_to_code.connect(self._canvas_navigate_to_code)
        self.canvas_widget.edit_item_requested.connect(self._canvas_open_item_editor)
        self.canvas_widget.delete_requested.connect(self._canvas_delete_item)
        self.canvas_widget.item_dropped.connect(self._canvas_on_item_dropped)
        self.canvas_widget.connection_added.connect(self._canvas_on_connection_added)
        splitter.addWidget(self.canvas_widget)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([200, 600])
        splitter.setHandleWidth(6)
        splitter.setStyleSheet(
            "QSplitter::handle { background-color: #C0C0C0; }"
            "QSplitter::handle:hover { background-color: #A0A0A0; }"
        )

        layout.addWidget(splitter)

        self.tabs.addTab(tab, "Node Canvas")

        # Populate the file tree
        self._populate_canvas_file_tree()

        # Set active package
        if self._canvas_state.packages:
            first_pkg = list(self._canvas_state.packages.keys())[0]
            self.canvas_widget.set_active_package(first_pkg)

        # Canvas autosave timer (every 5 seconds)
        self._canvas_autosave_timer = QTimer(self)
        self._canvas_autosave_timer.timeout.connect(self._save_canvas_state)
        self._canvas_autosave_timer.start(5000)

    # --- Canvas helper methods ---

    def _load_canvas_state(self):
        """Load canvas state from .node_canvas.json or create default."""
        if os.path.isfile(_CANVAS_STATE_FILE):
            try:
                with open(_CANVAS_STATE_FILE, "r") as f:
                    data = json.load(f)
                return CanvasState.from_dict(data)
            except Exception:
                pass
        return CanvasState.default_movement_pkg()

    def _save_canvas_state(self):
        """Persist canvas state to .node_canvas.json."""
        try:
            with open(_CANVAS_STATE_FILE, "w") as f:
                json.dump(self._canvas_state.to_dict(), f, indent=2)
        except Exception:
            pass

    def _canvas_on_item_dropped(self):
        """Handle an item dropped from the tree onto the canvas."""
        self._canvas_push_undo()
        self._populate_canvas_file_tree()

    def _canvas_on_connection_added(self):
        """Handle a new connection (may have created an inline topic)."""
        self._canvas_push_undo()
        self._populate_canvas_file_tree()

    def _canvas_push_undo(self):
        """Push current canvas state to undo stack and save to disk."""
        self._canvas_undo_stack.push(self._canvas_state.to_dict())
        self._save_canvas_state()

    def _canvas_undo(self):
        prev = self._canvas_undo_stack.undo(self._canvas_state.to_dict())
        if prev:
            self._canvas_state = CanvasState.from_dict(prev)
            self.canvas_widget.state = self._canvas_state
            self.canvas_widget.update()
            self._populate_canvas_file_tree()

    def _canvas_redo(self):
        nxt = self._canvas_undo_stack.redo(self._canvas_state.to_dict())
        if nxt:
            self._canvas_state = CanvasState.from_dict(nxt)
            self.canvas_widget.state = self._canvas_state
            self.canvas_widget.update()
            self._populate_canvas_file_tree()

    def _generate_node_template(self, node_name):
        """Return standard ROS2 node boilerplate for a new node."""
        class_name = node_name[0].upper() + node_name[1:]
        return (
            f"import rclpy\n"
            f"from rclpy.node import Node\n"
            f"\n"
            f"\n"
            f"class {class_name}(Node):\n"
            f"    def __init__(self):\n"
            f"        super().__init__('{node_name}')\n"
            f"\n"
            f"        # === Publishers ===\n"
            f"        # self.publisher = self.create_publisher(String, '/topic', 10)\n"
            f"\n"
            f"        # === Subscribers ===\n"
            f"        # self.subscription = self.create_subscription(\n"
            f"        #     String, '/topic', self.listener_callback, 10)\n"
            f"\n"
            f"        # === Timers ===\n"
            f"        self.timer = self.create_timer(0.1, self.timer_callback)\n"
            f"\n"
            f"    def timer_callback(self):\n"
            f"        pass\n"
            f"\n"
            f"    # def listener_callback(self, msg):\n"
            f"    #     self.get_logger().info(f'Received: {{msg.data}}')\n"
            f"\n"
            f"\n"
            f"def main(args=None):\n"
            f"    rclpy.init(args=args)\n"
            f"    node = {class_name}()\n"
            f"    rclpy.spin(node)\n"
            f"    node.destroy_node()\n"
            f"    rclpy.shutdown()\n"
            f"\n"
            f"\n"
            f"if __name__ == '__main__':\n"
            f"    main()\n"
        )

    def _generate_topic_template(self, topic_name):
        """Return standard ROS2 publisher/subscriber boilerplate for a topic."""
        return (
            f"# Topic: {topic_name}\n"
            f"#\n"
            f"# Publisher usage:\n"
            f"#   from std_msgs.msg import String\n"
            f"#   self.publisher = self.create_publisher(String, '{topic_name}', 10)\n"
            f"#   msg = String()\n"
            f"#   msg.data = 'hello'\n"
            f"#   self.publisher.publish(msg)\n"
            f"#\n"
            f"# Subscriber usage:\n"
            f"#   from std_msgs.msg import String\n"
            f"#   self.subscription = self.create_subscription(\n"
            f"#       String, '{topic_name}', self.listener_callback, 10)\n"
            f"#\n"
            f"#   def listener_callback(self, msg):\n"
            f"#       self.get_logger().info(f'Received: {{msg.data}}')\n"
        )

    def _ensure_node_file(self, package, node_name):
        """Create the node .py file with template code if it doesn't exist."""
        pkg_dir = os.path.join(_PKG_DIR, package)
        os.makedirs(pkg_dir, exist_ok=True)
        file_path = os.path.join(pkg_dir, f"{node_name}.py")
        if not os.path.isfile(file_path):
            with open(file_path, "w") as f:
                f.write(self._generate_node_template(node_name))
        return f"{package}/{node_name}.py"

    def _canvas_add_menu(self):
        """+ button dialog: New Package / New Node / New Topic / Cancel."""
        items = ["New Node", "New Topic", "New Package", "Topics from Robot", "Cancel"]
        choice, ok = QInputDialog.getItem(
            self, "Add to Canvas", "What would you like to add?",
            items, 0, False)
        if not ok or choice == "Cancel":
            return

        if choice == "New Package":
            name, ok = QInputDialog.getText(
                self, "New Package", "Package name:")
            if ok and name.strip():
                name = name.strip()
                label, ok2 = QInputDialog.getText(
                    self, "Package Label",
                    "Display label:", text=name)
                if ok2 and label.strip():
                    self._canvas_state.packages[name] = {
                        "label": label.strip(), "color": "#34C759"}
                    self._canvas_push_undo()
                    self._populate_canvas_file_tree()
                    self._load_file_tree()
                    self.canvas_widget.update()

        elif choice == "New Node":
            name, ok = QInputDialog.getText(
                self, "New Node", "Node name:")
            if ok and name.strip():
                pkg = self.canvas_widget._active_package
                if not pkg and self._canvas_state.packages:
                    pkg = list(self._canvas_state.packages.keys())[0]
                if not pkg:
                    pkg = "default_pkg"
                # Offset so successive nodes don't overlap
                existing = sum(1 for it in self._canvas_state.items.values()
                               if it.package == pkg and it.item_type == "node")
                x = 400 + (existing % 5) * 100
                y = 200 + (existing // 5) * 100
                self._canvas_state.add_item("node", name.strip(), x, y, pkg)
                self._ensure_node_file(pkg, name.strip())
                self._canvas_push_undo()
                self._populate_canvas_file_tree()
                self._load_file_tree()
                self.canvas_widget.update()

        elif choice == "New Topic":
            name, ok = QInputDialog.getText(
                self, "New Topic", "Topic name (e.g. /my_topic):")
            if ok and name.strip():
                topic_name = name.strip()
                if not topic_name.startswith("/"):
                    topic_name = "/" + topic_name
                pkg = self.canvas_widget._active_package
                if not pkg and self._canvas_state.packages:
                    pkg = list(self._canvas_state.packages.keys())[0]
                if not pkg:
                    pkg = "default_pkg"
                # Offset so successive topics don't overlap
                existing = sum(1 for it in self._canvas_state.items.values()
                               if it.package == pkg and it.item_type == "topic")
                x = 400 + (existing % 5) * 120
                y = 300 + (existing // 5) * 80
                self._canvas_state.add_item("topic", topic_name, x, y, pkg)
                self._canvas_push_undo()
                self._populate_canvas_file_tree()
                self.canvas_widget.update()

        elif choice == "Topics from Robot":
            self._canvas_add_robot_topics()

    def _canvas_add_robot_topics(self):
        """Discover active robot topics via SSH and add selected ones to canvas."""
        if not self._require_ssh_connection():
            return

        # Show a loading dialog while discovering topics
        loading_dlg = QDialog(self)
        loading_dlg.setWindowTitle("Discovering Topics")
        loading_layout = QVBoxLayout(loading_dlg)
        loading_label = QLabel("Discovering active ROS2 topics...")
        loading_layout.addWidget(loading_label)
        loading_dlg.setMinimumWidth(300)

        discovered_topics = []

        def _on_topics_found(topics):
            discovered_topics.extend(topics)
            loading_dlg.accept()

        worker = TopicDiscoveryWorker(self.ssh_client)
        worker.topics_found.connect(_on_topics_found)
        worker.log.connect(self._log)
        worker.start()
        loading_dlg.exec()
        worker.wait()

        if not discovered_topics:
            QMessageBox.information(self, "No Topics",
                                   "No active ROS2 topics were found.")
            return

        # Show topic selection dialog
        dlg = QDialog(self)
        dlg.setWindowTitle("Active Robot Topics")
        dlg.setMinimumWidth(450)
        dlg.setMinimumHeight(350)
        layout = QVBoxLayout(dlg)

        # Top row with Add Topics button
        top_row = QHBoxLayout()
        top_row.addStretch()
        add_btn = QPushButton("Add Topics")
        add_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; "
            "padding: 8px 20px; border-radius: 8px; font-weight: bold; }"
        )
        top_row.addWidget(add_btn)
        layout.addLayout(top_row)

        # Scrollable list of topics with checkboxes
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)

        checkboxes = []
        for topic in sorted(discovered_topics):
            cb = QCheckBox(topic)
            cb.setLayoutDirection(Qt.LayoutDirection.RightToLeft)
            scroll_layout.addWidget(cb)
            checkboxes.append(cb)
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        def _on_add():
            selected = [cb.text() for cb in checkboxes if cb.isChecked()]
            if not selected:
                return
            # Ensure "robot" package exists
            if "robot" not in self._canvas_state.packages:
                self._canvas_state.packages["robot"] = {
                    "label": "robot", "color": "#34C759"
                }
            # Add each selected topic
            existing = sum(1 for it in self._canvas_state.items.values()
                           if it.package == "robot" and it.item_type == "topic")
            for i, topic_name in enumerate(selected):
                idx = existing + i
                x = 400 + (idx % 5) * 120
                y = 300 + (idx // 5) * 80
                self._canvas_state.add_item("topic", topic_name, x, y, "robot")
            self._canvas_push_undo()
            self._populate_canvas_file_tree()
            self.canvas_widget.update()
            dlg.accept()

        add_btn.clicked.connect(_on_add)
        dlg.exec()

    def _canvas_toggle_delete_mode(self):
        """Toggle delete mode — show/hide red X on canvas items AND side pane."""
        on = not self.canvas_widget._delete_mode
        self.canvas_widget._delete_mode = on
        self.canvas_delete_btn.setChecked(on)
        self.canvas_pane_delete_btn.setChecked(on)
        self._populate_canvas_file_tree()
        self.canvas_widget.update()

    def _canvas_tree_item_changed(self, item, column):
        """No-op — canvas tree is not inline-editable."""
        return

    def _canvas_tree_double_clicked(self, item, column):
        """Double-click to rename a node, topic, or package in the canvas tree."""
        data = item.data(0, Qt.ItemDataRole.UserRole)
        if not data:
            return
        kind, old_name = data
        if kind == "package":
            type_label = "folder"
        elif kind == "node":
            type_label = "node"
        elif kind == "topic":
            type_label = "topic"
        else:
            return
        reply = QMessageBox.question(
            self, "Rename",
            f"Would you like to change the name of this {type_label}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        new_name, ok = QInputDialog.getText(
            self, f"Rename {type_label}", "New name:", text=old_name)
        if not ok or not new_name.strip() or new_name.strip() == old_name:
            return
        new_name = new_name.strip()
        if kind == "package":
            if old_name in self._canvas_state.packages:
                pkg_info = self._canvas_state.packages.pop(old_name)
                pkg_info["label"] = new_name
                self._canvas_state.packages[new_name] = pkg_info
                for it in self._canvas_state.items.values():
                    if it.package == old_name:
                        it.package = new_name
                if self.canvas_widget._active_package == old_name:
                    self.canvas_widget.set_active_package(new_name)
                # Also rename folder on disk if it exists
                old_dir = os.path.join(_PKG_DIR, old_name)
                new_dir = os.path.join(_PKG_DIR, new_name)
                if os.path.isdir(old_dir) and not os.path.exists(new_dir):
                    try:
                        os.rename(old_dir, new_dir)
                    except Exception:
                        pass
        elif kind in ("node", "topic"):
            for it in self._canvas_state.items.values():
                if it.item_type == kind and it.name == old_name:
                    it.name = new_name
                    break
        self._canvas_push_undo()
        self._populate_canvas_file_tree()
        self.canvas_widget.update()
        # Refresh Full View tree so the rename is reflected there too
        self._load_file_tree()

    # --- Full View delete mode ---

    def _fv_toggle_delete_mode(self):
        self._fv_edit_mode = not self._fv_edit_mode
        self.fv_delete_btn.setChecked(self._fv_edit_mode)
        self._load_file_tree()

    def _fv_add_menu(self):
        """+ button dialog: Add Package / Add File / Cancel."""
        items = ["Add Package", "Add File", "Cancel"]
        choice, ok = QInputDialog.getItem(
            self, "Add to Full View", "What would you like to add?",
            items, 0, False)
        if not ok or choice == "Cancel":
            return
        if choice == "Add Package":
            name, ok = QInputDialog.getText(
                self, "New Package", "Package folder name:")
            if ok and name.strip():
                pkg_dir = os.path.join(_PKG_DIR, name.strip())
                os.makedirs(pkg_dir, exist_ok=True)
                self._load_file_tree()
        elif choice == "Add File":
            # Collect available package folders (on-disk + canvas packages)
            disk_folders = set(
                d for d in os.listdir(_PKG_DIR)
                if os.path.isdir(os.path.join(_PKG_DIR, d))
                and not d.startswith(".") and not d.startswith("__")
                and not any(d.endswith(h) or d == h
                            for h in self._FV_HIDDEN_DIRS)
            )
            canvas_pkgs = set(self._canvas_state.packages.keys())
            pkg_folders = sorted(disk_folders | canvas_pkgs)
            if not pkg_folders:
                QMessageBox.information(
                    self, "No Package",
                    "Create a package folder first.")
                return
            folder, ok = QInputDialog.getItem(
                self, "Select Package",
                "Add the file to which package folder?",
                pkg_folders, 0, False)
            if not ok:
                return
            name, ok2 = QInputDialog.getText(
                self, "New File", "File name (e.g. my_script.py):")
            if ok2 and name.strip():
                # Ensure the folder exists on disk
                folder_path = os.path.join(_PKG_DIR, folder)
                os.makedirs(folder_path, exist_ok=True)
                fpath = os.path.join(folder_path, name.strip())
                if not os.path.exists(fpath):
                    with open(fpath, "w") as f:
                        f.write("")
                self._load_file_tree()

    def _fv_tree_item_changed(self, item, column):
        """No-op — Full View tree is not inline-editable."""
        return

    def _fv_tree_double_clicked(self, item, column):
        """Double-click to rename a file or folder in the Full View tree."""
        rel_path = item.data(0, Qt.ItemDataRole.UserRole)
        is_folder = rel_path is None
        if is_folder:
            type_label = "folder"
            # Extract display name (strip icon prefix)
            display = item.text(0).strip()
            for prefix in ("\U0001F4C1 ", "\U0001F4C1"):
                if display.startswith(prefix):
                    display = display[len(prefix):]
                    break
            old_name = display
        else:
            type_label = "file"
            old_name = os.path.basename(rel_path)
        reply = QMessageBox.question(
            self, "Rename",
            f"Would you like to change the name of this {type_label}?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        new_name, ok = QInputDialog.getText(
            self, f"Rename {type_label}", "New name:", text=old_name)
        if not ok or not new_name.strip() or new_name.strip() == old_name:
            return
        new_name = new_name.strip()
        if is_folder:
            old_path = os.path.join(_PKG_DIR, old_name)
            new_path = os.path.join(_PKG_DIR, new_name)
            # Rename on disk only if the directory actually exists
            if os.path.isdir(old_path):
                try:
                    os.rename(old_path, new_path)
                except Exception as e:
                    QMessageBox.warning(self, "Rename failed", str(e))
                    return
            # Update current file reference if it was inside the renamed folder
            if (self._full_view_current_file
                    and self._full_view_current_file.startswith(old_name + "/")):
                self._full_view_current_file = (
                    new_name + self._full_view_current_file[len(old_name):])
            # If the renamed folder is also a canvas package, update canvas state
            if old_name in self._canvas_state.packages:
                pkg_info = self._canvas_state.packages.pop(old_name)
                pkg_info["label"] = new_name
                self._canvas_state.packages[new_name] = pkg_info
                for it in self._canvas_state.items.values():
                    if it.package == old_name:
                        it.package = new_name
                if self.canvas_widget._active_package == old_name:
                    self.canvas_widget.set_active_package(new_name)
                self._canvas_push_undo()
                self._populate_canvas_file_tree()
                self.canvas_widget.update()
        else:
            old_path = os.path.join(_PKG_DIR, rel_path)
            new_rel = os.path.join(os.path.dirname(rel_path), new_name)
            new_path = os.path.join(_PKG_DIR, new_rel)
            try:
                os.rename(old_path, new_path)
            except Exception as e:
                QMessageBox.warning(self, "Rename failed", str(e))
                return
            if self._full_view_current_file == rel_path:
                self._full_view_current_file = new_rel
        self._load_file_tree()

    def _canvas_delete_item(self, item_id):
        """Prompt user then delete a node/topic from canvas and disk."""
        item = self._canvas_state.items.get(item_id)
        if not item:
            return
        # Block deletion of protected items
        if (item.package == _PROTECTED_PACKAGE
                and (item.name in _PROTECTED_NODES
                     or item.name in _PROTECTED_TOPICS)):
            return
        type_label = "Node" if item.item_type == "node" else "Topic"
        reply = QMessageBox.question(
            self, f"Delete {type_label}",
            f"Delete {type_label} \"{item.name}\"?\n"
            "This will also delete its file from disk.\n"
            "You can not undo this action.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return

        # Delete file from disk
        rel = f"{item.package}/{item.name}.py"
        full = os.path.join(_PKG_DIR, rel)
        if os.path.isfile(full):
            os.remove(full)
        if self._full_view_current_file == rel:
            self._full_view_current_file = None
            self.full_editor.clear()

        # Remove from canvas state (also removes connections)
        self._canvas_state.remove_item(item_id)
        self._canvas_push_undo()
        self._populate_canvas_file_tree()
        self._load_file_tree()
        self.canvas_widget.update()

    def _canvas_comment_out_code(self, item):
        """Comment out code associated with a hidden canvas item."""
        mp = os.path.join(_PKG_DIR, "movement_pkg", "movement.py")
        if not os.path.isfile(mp):
            return
        try:
            with open(mp, "r") as f:
                lines = f.readlines()
        except Exception:
            return

        changed = False
        if item.item_type == "topic":
            topic_str = f"'{item.name}'"
            topic_str2 = f'"{item.name}"'
            new_lines = []
            for l in lines:
                if (topic_str in l or topic_str2 in l) and not l.lstrip().startswith("#"):
                    indent = len(l) - len(l.lstrip())
                    new_lines.append(l[:indent] + "# " + l[indent:])
                    changed = True
                else:
                    new_lines.append(l)
        elif item.item_type == "node":
            new_lines = lines  # nodes are structural — don't strip code
        else:
            return

        if changed:
            with open(mp, "w") as f:
                f.writelines(new_lines)
            # Refresh Full View editor if movement.py is open
            if (self._full_view_current_file
                    and self._full_view_current_file.endswith("movement.py")):
                self.full_editor.setPlainText("".join(new_lines))

    def _canvas_navigate_to_code(self, file_path, search_text):
        """Switch to Code Editor Full View, select file, jump to search_text."""
        # Switch to Code Editor tab (index 2 after inserting Node Canvas)
        self.tabs.setCurrentIndex(2)
        # Switch to Full View
        self._show_full_view()

        # Select the file in the file tree
        self._select_file_tree_item(file_path)

        # Search for the text and highlight it
        if search_text:
            QTimer.singleShot(100, lambda: self._canvas_highlight_text(search_text))

    def _canvas_highlight_text(self, search_text):
        """Find and highlight text in the Full View editor."""
        cursor = self.full_editor.textCursor()
        # Move to start
        cursor.movePosition(cursor.MoveOperation.Start)
        self.full_editor.setTextCursor(cursor)

        # Find the text
        found = self.full_editor.find(search_text)
        if found:
            # Highlight with light blue using extra selections
            cursor = self.full_editor.textCursor()
            sel = QTextEdit.ExtraSelection()
            sel.cursor = cursor
            fmt = QTextCharFormat()
            fmt.setBackground(QColor("#D4EDDA"))
            sel.format = fmt
            self.full_editor.setExtraSelections([sel])

    def _canvas_open_item_editor(self, item_id):
        """Open a CodeEditorDialog for the given canvas item."""
        item = self._canvas_state.items.get(item_id)
        if not item:
            return

        if item.item_type == "node":
            rel_path = f"{item.package}/{item.name}.py"
            search_text = f"class {item.name[0].upper() + item.name[1:]}"
            title = "Node Editor"
        else:
            rel_path = self._find_file_containing_topic(item.name)
            search_text = item.name
            title = "Topic Editor"

        # Robot topics have no local source
        if not rel_path and item.package == "robot":
            QMessageBox.information(self, title,
                "Sorry, can not show the source code of this topic from the robot.")
            return

        # For non-robot items with no file yet, create a template
        if not rel_path and item.item_type == "topic":
            # Create a topic file in the item's package directory
            rel_path = f"{item.package}/{item.name.strip('/')}.py"
            full_path = os.path.join(_PKG_DIR, rel_path)
            os.makedirs(os.path.dirname(full_path), exist_ok=True)
            with open(full_path, "w") as f:
                f.write(self._generate_topic_template(item.name))
            self._load_file_tree()

        full_path = os.path.join(_PKG_DIR, rel_path)
        if not os.path.isfile(full_path):
            # Node file missing — create from template
            self._ensure_node_file(item.package, item.name)
            self._load_file_tree()

        dlg = CodeEditorDialog(title, full_path, search_text, parent=self)
        dlg.exec()

        if dlg.saved:
            # Sync Full View if same file is open
            if self._full_view_current_file == rel_path:
                self.full_editor.setPlainText(dlg.content)
            # Reload params if movement.py was edited
            if rel_path.endswith("movement.py"):
                self._load_params()
            # Refresh Full View file tree in case a new file was created
            self._load_file_tree()

        if dlg.show_in_code:
            self._canvas_navigate_to_code(rel_path, search_text)

    def _find_file_containing_topic(self, topic_name):
        """Search .py files under _PKG_DIR for one referencing topic_name."""
        for rel_path in _FULL_VIEW_FILES:
            if not rel_path.endswith(".py"):
                continue
            full_path = os.path.join(_PKG_DIR, rel_path)
            if not os.path.isfile(full_path):
                continue
            try:
                with open(full_path, "r") as f:
                    if topic_name in f.read():
                        return rel_path
            except Exception:
                continue
        # Fallback: all .py files
        for py_file in glob.glob(os.path.join(_PKG_DIR, "**", "*.py"), recursive=True):
            rel = os.path.relpath(py_file, _PKG_DIR)
            try:
                with open(py_file, "r") as f:
                    if topic_name in f.read():
                        return rel
            except Exception:
                continue
        return None

    def _canvas_file_tree_clicked(self, item, column):
        """Handle clicks on FILES tree items."""
        data = item.data(0, Qt.ItemDataRole.UserRole)
        if not data:
            return
        kind, value = data

        # Delete-mode: red cross column
        if column == 1 and self.canvas_widget._delete_mode:
            reply = QMessageBox.warning(
                self, "Delete Item",
                "Are you sure you want to delete this item? "
                "You can not undo this action.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                return
            if kind == "package":
                # Delete files from disk
                pkg_dir = os.path.join(_PKG_DIR, value)
                if os.path.isdir(pkg_dir):
                    shutil.rmtree(pkg_dir)
                # Remove from canvas state
                self._canvas_state.packages.pop(value, None)
                to_remove = [iid for iid, it in self._canvas_state.items.items()
                             if it.package == value]
                for iid in to_remove:
                    self._canvas_state.remove_item(iid)
            elif kind in ("node", "topic"):
                for iid, it in list(self._canvas_state.items.items()):
                    if it.item_type == kind and it.name == value:
                        # Delete file from disk
                        rel = f"{it.package}/{it.name}.py"
                        full = os.path.join(_PKG_DIR, rel)
                        if os.path.isfile(full):
                            os.remove(full)
                        if self._full_view_current_file == rel:
                            self._full_view_current_file = None
                            self.full_editor.clear()
                        self._canvas_state.remove_item(iid)
                        break
            self._canvas_push_undo()
            self._populate_canvas_file_tree()
            self._load_file_tree()
            self.canvas_widget.update()
            return

        if kind == "package":
            self.canvas_widget.set_active_package(value)
            pkg_info = self._canvas_state.packages.get(value, {})
            label = pkg_info.get("label", value)
            color = pkg_info.get("color", "#34C759")
            self.canvas_pkg_label.setText(label)
            self.canvas_pkg_label.setStyleSheet(f"color: {color};")
        elif kind == "node" or kind == "topic":
            # Scroll / highlight the item on canvas
            canvas_item = self.state_item_by_name(kind, value)
            if canvas_item:
                # Switch to the item's package
                pkg_name = canvas_item.package
                self.canvas_widget.set_active_package(pkg_name)
                pkg_info = self._canvas_state.packages.get(pkg_name, {})
                label = pkg_info.get("label", pkg_name)
                color = pkg_info.get("color", "#34C759")
                self.canvas_pkg_label.setText(label)
                self.canvas_pkg_label.setStyleSheet(f"color: {color};")
                self.canvas_widget._hovered_item = canvas_item.item_id
                self.canvas_widget.update()

    def state_item_by_name(self, item_type, name):
        """Find a CanvasItem by type and name."""
        for it in self._canvas_state.items.values():
            if it.item_type == item_type and it.name == name:
                return it
        return None

    def _canvas_discover_topics(self, callback):
        """SSH topic discovery; shows 'connect first' dialog if no SSH."""
        if self.ssh_client is None:
            reply = QMessageBox.question(
                self, "Not Connected",
                "Please connect to your robot first.\n\n"
                "Switch to Robot Control tab to connect?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel)
            if reply == QMessageBox.StandardButton.Yes:
                self.tabs.setCurrentIndex(0)
            return

        worker = TopicDiscoveryWorker(self.ssh_client)
        worker.log.connect(self._log)
        worker.topics_found.connect(callback)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker))
        worker.start()

    def _populate_canvas_file_tree(self):
        """Rebuild FILES tree from canvas state."""
        self._blocking_item_changed = True
        self.canvas_file_tree.clear()

        delete_on = self.canvas_widget._delete_mode
        if delete_on:
            self.canvas_file_tree.setColumnCount(2)
            self.canvas_file_tree.header().setStretchLastSection(False)
            self.canvas_file_tree.header().setSectionResizeMode(
                0, self.canvas_file_tree.header().ResizeMode.Stretch)
            self.canvas_file_tree.header().setSectionResizeMode(
                1, self.canvas_file_tree.header().ResizeMode.Fixed)
            self.canvas_file_tree.header().resizeSection(1, 30)
        else:
            self.canvas_file_tree.setColumnCount(1)

        for pkg_name, pkg_info in self._canvas_state.packages.items():
            label = pkg_info.get("label", pkg_name)
            color = pkg_info.get("color", "#34C759")

            pkg_item = QTreeWidgetItem(self.canvas_file_tree)
            pkg_item.setText(0, f"\U0001F4C1 {label}")
            pkg_item.setData(0, Qt.ItemDataRole.UserRole, ("package", pkg_name))
            pkg_item.setForeground(0, QColor(color))
            pkg_item.setFont(0, QFont("Menlo", 11, QFont.Weight.Bold))
            is_protected_pkg = (pkg_name == _PROTECTED_PACKAGE)
            if delete_on and not is_protected_pkg:
                pkg_item.setText(1, "\u2716")
                pkg_item.setForeground(1, QColor("#FF3B30"))

            # Nodes section
            nodes_header = QTreeWidgetItem(pkg_item)
            nodes_header.setText(0, "Nodes")
            nodes_header.setFont(0, QFont("Menlo", 10, QFont.Weight.Bold))
            nodes_header.setForeground(0, QColor("#888888"))

            for it in self._canvas_state.items.values():
                if it.package == pkg_name and it.item_type == "node":
                    node_item = QTreeWidgetItem(nodes_header)
                    node_item.setText(0, f"\u25CF {it.name}")
                    node_item.setData(0, Qt.ItemDataRole.UserRole,
                                      ("node", it.name))
                    if it.hidden:
                        node_item.setForeground(0, QColor("#B0B0B0"))
                    else:
                        node_item.setForeground(0, QColor("#007AFF"))
                    if delete_on and not (is_protected_pkg
                                          and it.name in _PROTECTED_NODES):
                        node_item.setText(1, "\u2716")
                        node_item.setForeground(1, QColor("#FF3B30"))

            # Topics section
            topics_header = QTreeWidgetItem(pkg_item)
            topics_header.setText(0, "Topics")
            topics_header.setFont(0, QFont("Menlo", 10, QFont.Weight.Bold))
            topics_header.setForeground(0, QColor("#888888"))

            for it in self._canvas_state.items.values():
                if it.package == pkg_name and it.item_type == "topic":
                    topic_item = QTreeWidgetItem(topics_header)
                    topic_item.setText(0, f"\u25A0 {it.name}")
                    topic_item.setData(0, Qt.ItemDataRole.UserRole,
                                       ("topic", it.name))
                    if it.hidden:
                        topic_item.setForeground(0, QColor("#B0B0B0"))
                    else:
                        topic_item.setForeground(0, QColor("#007AFF"))
                    if delete_on and not (is_protected_pkg
                                          and it.name in _PROTECTED_TOPICS):
                        topic_item.setText(1, "\u2716")
                        topic_item.setForeground(1, QColor("#FF3B30"))

            pkg_item.setExpanded(True)
            nodes_header.setExpanded(True)
            topics_header.setExpanded(True)

        self._blocking_item_changed = False

    # ------------------------------------------------------------------ #
    #  Tab 3: Code Editor                                                  #
    # ------------------------------------------------------------------ #

    def _build_code_editor_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # Top bar
        top_bar = QHBoxLayout()

        # Undo / Redo buttons (high-contrast circular style)
        _circle_btn_style = (
            "QPushButton { font-size: 20px; font-weight: bold;"
            "  min-width: 36px; max-width: 36px;"
            "  min-height: 36px; max-height: 36px; border-radius: 18px;"
            "  background-color: #007AFF; color: white;"
            "  border: 2px solid #005ECB; }"
            "QPushButton:hover { background-color: #005ECB; }"
        )
        self.undo_btn = QPushButton("\u21BA")          # ↺ anti-clockwise
        self.undo_btn.setToolTip("Undo")
        self.undo_btn.setStyleSheet(_circle_btn_style)
        self.undo_btn.clicked.connect(self._undo)
        top_bar.addWidget(self.undo_btn)

        self.redo_btn = QPushButton("\u21BB")          # ↻ clockwise
        self.redo_btn.setToolTip("Redo")
        self.redo_btn.setStyleSheet(_circle_btn_style)
        self.redo_btn.clicked.connect(self._redo)
        top_bar.addWidget(self.redo_btn)

        _toggle_btn_style = (
            "QPushButton { border: none; padding: 6px 16px; border-radius: 8px;"
            "  color: white; background: #8E8E93; font-size: 13px; }"
            "QPushButton:checked { background: #007AFF; }"
        )

        self.simple_view_btn = QPushButton("Simple View")
        self.simple_view_btn.setCheckable(True)
        self.simple_view_btn.setChecked(True)
        self.simple_view_btn.setStyleSheet(_toggle_btn_style)
        self.simple_view_btn.clicked.connect(self._show_simple_view)
        top_bar.addWidget(self.simple_view_btn)

        self.full_view_btn = QPushButton("Expert View")
        self.full_view_btn.setCheckable(True)
        self.full_view_btn.setStyleSheet(_toggle_btn_style)
        self.full_view_btn.clicked.connect(self._show_full_view)
        top_bar.addWidget(self.full_view_btn)

        self._view_group = QButtonGroup(tab)
        self._view_group.setExclusive(True)
        self._view_group.addButton(self.simple_view_btn)
        self._view_group.addButton(self.full_view_btn)

        top_bar.addStretch()

        # Font size buttons (high-contrast circular style)
        self.font_smaller_btn = QPushButton("A")
        self.font_smaller_btn.setToolTip("Decrease font size")
        self.font_smaller_btn.setStyleSheet(
            _circle_btn_style + " QPushButton { font-size: 14px; }"
        )
        self.font_smaller_btn.clicked.connect(self._decrease_font_size)
        top_bar.addWidget(self.font_smaller_btn)

        self.font_larger_btn = QPushButton("A")
        self.font_larger_btn.setToolTip("Increase font size")
        self.font_larger_btn.setStyleSheet(
            _circle_btn_style + " QPushButton { font-size: 20px; }"
        )
        self.font_larger_btn.clicked.connect(self._increase_font_size)
        top_bar.addWidget(self.font_larger_btn)

        # Full View: search button
        self.fv_search_btn = QPushButton("\U0001F50D")
        self.fv_search_btn.setToolTip("Search in code")
        self.fv_search_btn.setStyleSheet(_circle_btn_style)
        self.fv_search_btn.clicked.connect(self._fv_toggle_search)
        top_bar.addWidget(self.fv_search_btn)
        self.fv_search_btn.hide()

        # Full View: add (+) button
        self.fv_add_btn = QPushButton("+")
        self.fv_add_btn.setToolTip("Add package or file")
        self.fv_add_btn.setStyleSheet(_circle_btn_style)
        self.fv_add_btn.clicked.connect(self._fv_add_menu)
        top_bar.addWidget(self.fv_add_btn)
        self.fv_add_btn.hide()

        # Full View: delete (−) button
        _fv_delete_btn_style = (
            "QPushButton { font-size: 20px; font-weight: bold;"
            "  min-width: 36px; max-width: 36px;"
            "  min-height: 36px; max-height: 36px; border-radius: 18px;"
            "  background-color: #FF3B30; color: white;"
            "  border: 2px solid #CC2A22; }"
            "QPushButton:hover { background-color: #CC2A22; }"
            "QPushButton:checked { background-color: #CC2A22;"
            "  border: 2px solid #FFFFFF; }"
        )
        self.fv_delete_btn = QPushButton("\u2212")
        self.fv_delete_btn.setToolTip("Delete mode — click red X to remove items")
        self.fv_delete_btn.setCheckable(True)
        self.fv_delete_btn.setStyleSheet(_fv_delete_btn_style)
        self.fv_delete_btn.clicked.connect(self._fv_toggle_delete_mode)
        top_bar.addWidget(self.fv_delete_btn)
        self.fv_delete_btn.hide()

        self.editor_save_btn = QPushButton("Save")
        self.editor_save_btn.setStyleSheet(
            "QPushButton { background-color: #34C759; color: white; padding: 6px 14px; border-radius: 8px; }"
        )
        self.editor_save_btn.setMinimumWidth(100)
        self.editor_save_btn.clicked.connect(self._save_from_editor)
        top_bar.addWidget(self.editor_save_btn)

        self.editor_deploy_btn = QPushButton("Deploy")
        self.editor_deploy_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; padding: 6px 14px; border-radius: 8px; }"
            "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }"
        )
        self.editor_deploy_btn.setMinimumWidth(100)
        self.editor_deploy_btn.setEnabled(False)
        self.editor_deploy_btn.clicked.connect(self._deploy_from_editor)
        top_bar.addWidget(self.editor_deploy_btn)

        layout.addLayout(top_bar)

        # Stacked widget (Simple View = 0, Full View = 1)
        self.editor_stack = QStackedWidget()

        # --- Simple View (splitter: functions panel | code editor) ---
        simple_view_splitter = QSplitter(Qt.Orientation.Horizontal)

        functions_panel = FunctionsPanel()
        func_scroll = QScrollArea()
        func_scroll.setWidgetResizable(True)
        func_scroll.setWidget(functions_panel)
        func_scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        func_scroll.setMinimumWidth(180)
        simple_view_splitter.addWidget(func_scroll)

        self.simple_editor = SimpleViewEditor()
        self.simple_editor.setFont(QFont("Menlo", 13))
        self.simple_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._simple_highlighter = SimpleCodeHighlighter(
            self.simple_editor.document()
        )
        self.simple_editor.textChanged.connect(self._on_simple_code_changed)
        simple_view_splitter.addWidget(self.simple_editor)

        simple_view_splitter.setStretchFactor(0, 0)   # panel: fixed
        simple_view_splitter.setStretchFactor(1, 1)   # editor: stretches
        simple_view_splitter.setSizes([200, 600])

        self.editor_stack.addWidget(simple_view_splitter)

        # --- Full View ---
        full_view_widget = QWidget()
        full_view_outer = QVBoxLayout(full_view_widget)
        full_view_outer.setContentsMargins(0, 0, 0, 0)
        full_view_outer.setSpacing(0)

        # Search bar (hidden by default)
        self._fv_search_bar = QWidget()
        _sb_layout = QHBoxLayout(self._fv_search_bar)
        _sb_layout.setContentsMargins(4, 4, 4, 4)
        self._fv_search_input = QLineEdit()
        self._fv_search_input.setPlaceholderText("Search...")
        self._fv_search_input.textChanged.connect(self._fv_perform_search)
        _sb_layout.addWidget(self._fv_search_input)
        _sb_close = QPushButton("\u2715")
        _sb_close.setFixedSize(28, 28)
        _sb_close.setStyleSheet(
            "QPushButton { border: none; font-size: 16px; }"
            "QPushButton:hover { color: red; }"
        )
        _sb_close.clicked.connect(self._fv_toggle_search)
        _sb_layout.addWidget(_sb_close)
        self._fv_search_bar.hide()
        full_view_outer.addWidget(self._fv_search_bar)

        # Horizontal content: file tree + editor
        _fv_content = QWidget()
        full_layout = QHBoxLayout(_fv_content)
        full_layout.setContentsMargins(0, 0, 0, 0)
        full_view_outer.addWidget(_fv_content)

        # Left container: header row + file tree (fixed 220px)
        fv_left_container = QWidget()
        fv_left_container.setFixedWidth(220)
        fv_left_layout = QVBoxLayout(fv_left_container)
        fv_left_layout.setContentsMargins(0, 0, 0, 0)
        fv_left_layout.setSpacing(2)

        fv_tree_lbl = QLabel("FILES")
        fv_tree_lbl.setFont(QFont("Menlo", 11, QFont.Weight.Bold))
        fv_tree_lbl.setContentsMargins(4, 2, 4, 2)
        fv_left_layout.addWidget(fv_tree_lbl)

        self.file_tree = QTreeWidget()
        self.file_tree.setHeaderHidden(True)
        self.file_tree.setFont(QFont("Menlo", 11))
        self.file_tree.itemClicked.connect(self._on_file_tree_clicked)
        self.file_tree.itemDoubleClicked.connect(self._fv_tree_double_clicked)
        self.file_tree.itemChanged.connect(self._fv_tree_item_changed)
        fv_left_layout.addWidget(self.file_tree)

        full_layout.addWidget(fv_left_container)

        self.full_editor = LineNumberEditor()
        self.full_editor.setFont(QFont("Menlo", 12))
        self.full_editor.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._full_view_highlighter = FullViewHighlighter(
            self.full_editor.document()
        )
        full_layout.addWidget(self.full_editor)

        self.editor_stack.addWidget(full_view_widget)

        layout.addWidget(self.editor_stack)

        self.tabs.addTab(tab, "Code Editor")

        # Populate views
        self._load_file_tree()

        # Auto-save timer (every 5 seconds)
        self._autosave_timer = QTimer(self)
        self._autosave_timer.timeout.connect(self._autosave)
        self._autosave_timer.start(5000)

    # ------------------------------------------------------------------ #
    #  Tab 4: RoboApps                                                     #
    # ------------------------------------------------------------------ #

    def _build_roboapps_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # --- Top bar ---
        top_bar = QHBoxLayout()
        top_bar.addStretch()

        roboapps_label = QLabel("RoboApps")
        roboapps_label.setFont(QFont("Menlo", 13, QFont.Weight.Bold))
        roboapps_label.setStyleSheet("color: #34C759;")
        roboapps_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        top_bar.addWidget(roboapps_label)

        top_bar.addStretch()
        layout.addLayout(top_bar)

        # --- Main area (white background) ---
        main_area = QWidget()
        main_area.setStyleSheet("background-color: white;")
        main_area_layout = QVBoxLayout(main_area)
        main_area_layout.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        main_area_layout.setContentsMargins(24, 24, 24, 24)

        # Icons row — horizontal, left-aligned, icons added here at runtime
        icons_container = QWidget()
        self._roboapps_icons_layout = QHBoxLayout(icons_container)
        self._roboapps_icons_layout.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self._roboapps_icons_layout.setSpacing(16)
        self._roboapps_icons_layout.setContentsMargins(0, 0, 0, 0)

        # RoboSim app icon (iPhone-style)
        robosim_btn = QPushButton("RoboSim")
        robosim_btn.setFixedSize(120, 120)
        robosim_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: #007AFF;"
            "  color: white;"
            "  font-size: 16px;"
            "  font-weight: bold;"
            "  border-radius: 27px;"
            "  border: none;"
            "}"
            "QPushButton:hover {"
            "  background-color: #005ECB;"
            "}"
            "QPushButton:pressed {"
            "  background-color: #004AAD;"
            "}"
        )
        robosim_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        robosim_btn.setToolTip("Launch RoboSim5")
        robosim_btn.clicked.connect(self._launch_robosim)
        self._roboapps_icons_layout.addWidget(robosim_btn)
        self._roboapps_icons_layout.addStretch()

        main_area_layout.addWidget(icons_container)
        main_area_layout.addStretch()
        layout.addWidget(main_area)

        self.tabs.addTab(tab, "RoboApps")

        # Track whether the Conda icon has been added
        self._conda_icon_added = False
        # Check asynchronously so it doesn't delay UI startup
        QTimer.singleShot(300, self._check_and_add_conda_icon)

    # ------------------------------------------------------------------ #
    #  Conda detection, icon, and installation helpers                    #
    # ------------------------------------------------------------------ #

    def _is_conda_installed(self):
        """Return True if a conda executable is found on this machine."""
        if shutil.which("conda"):
            return True
        candidates = [
            os.path.expanduser("~/miniforge3/condabin/conda"),
            os.path.expanduser("~/miniforge3/bin/conda"),
            os.path.expanduser("~/miniconda3/condabin/conda"),
            os.path.expanduser("~/miniconda3/bin/conda"),
            os.path.expanduser("~/anaconda3/condabin/conda"),
            os.path.expanduser("~/anaconda3/bin/conda"),
            "/opt/homebrew/Caskroom/miniforge/base/condabin/conda",
        ]
        return any(os.path.isfile(c) for c in candidates)

    def _check_and_add_conda_icon(self):
        """Called once after startup — adds the Conda icon if conda is installed."""
        if self._is_conda_installed():
            self._add_conda_to_roboapps()

    def _add_conda_to_roboapps(self):
        """Add a Conda icon to the RoboApps tab (idempotent)."""
        if self._conda_icon_added:
            return
        self._conda_icon_added = True
        conda_btn = QPushButton("Conda")
        conda_btn.setFixedSize(120, 120)
        conda_btn.setStyleSheet(
            "QPushButton {"
            "  background-color: #34C759;"
            "  color: white;"
            "  font-size: 16px;"
            "  font-weight: bold;"
            "  border-radius: 27px;"
            "  border: none;"
            "}"
            "QPushButton:hover {"
            "  background-color: #248A3D;"
            "}"
            "QPushButton:pressed {"
            "  background-color: #1A6B2F;"
            "}"
        )
        conda_btn.setCursor(Qt.CursorShape.PointingHandCursor)
        conda_btn.setToolTip("Conda environment manager for ROS2")
        conda_btn.clicked.connect(self._on_conda_icon_clicked)
        # Insert before the trailing stretch
        count = self._roboapps_icons_layout.count()
        self._roboapps_icons_layout.insertWidget(count - 1, conda_btn)

    def _on_conda_icon_clicked(self):
        """Show brief Conda info when the RoboApps Conda icon is clicked."""
        conda_prefix, _ = self._find_conda_env()
        if conda_prefix:
            msg = f"Conda (ros_env) is ready.\n\nEnvironment path:\n{conda_prefix}"
        else:
            msg = (
                "Conda is installed but the ros_env environment was not found.\n\n"
                "Run  bash setup_robostack.sh  in the project folder to set it up."
            )
        QMessageBox.information(self, "Conda", msg)

    def _show_conda_not_installed_dialog(self):
        """Show the 'Conda not installed' dialog and trigger installation if chosen."""
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Conda Installation")
        dlg.setText("ROS 2 Humble in a Conda environment is required to run Rviz or Gazebo on your Mac.")
        install_btn = dlg.addButton("Install Conda", QMessageBox.ButtonRole.AcceptRole)
        dlg.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        dlg.exec()
        if dlg.clickedButton() == install_btn:
            self._run_conda_install()

    def _run_conda_install(self):
        """Download and install Miniforge3 in a background thread with a progress dialog."""
        progress = QDialog(self)
        progress.setWindowTitle("Installing Conda")
        progress.setMinimumWidth(520)
        prog_layout = QVBoxLayout(progress)

        prog_label = QLabel("Installing Conda and ROS 2 Humble — please wait...")
        prog_layout.addWidget(prog_label)

        prog_output = QPlainTextEdit()
        prog_output.setReadOnly(True)
        prog_output.setMinimumHeight(160)
        _mono = QFont("Menlo")
        _mono.setFixedPitch(True)
        _mono.setPointSize(10)
        prog_output.setFont(_mono)
        prog_output.setStyleSheet("background:#1e1e1e; color:#d4d4d4;")
        prog_layout.addWidget(prog_output)

        close_btn = QPushButton("Close")
        close_btn.setEnabled(False)
        close_btn.clicked.connect(progress.accept)
        prog_layout.addWidget(close_btn)

        worker = CondaInstallWorker()

        def _on_output(line):
            prog_output.appendPlainText(line)

        def _on_finished(success):
            close_btn.setEnabled(True)
            if success:
                self._add_conda_to_roboapps()
                QTimer.singleShot(200, self._show_conda_success_dialog)
            else:
                prog_label.setText("Installation failed — see output above.")

        worker.output.connect(_on_output)
        worker.finished.connect(_on_finished)
        worker.start()
        progress.exec()

    def _show_conda_success_dialog(self):
        """Show the 'installed successfully' dialog after Conda installation."""
        dlg = QMessageBox(self)
        dlg.setWindowTitle("Conda Installation")
        dlg.setText(
            "Conda has been installed successfully.\n"
            "You can launch Rviz or Gazebo in the Robot Control tab now."
        )
        go_btn = dlg.addButton("Go to Robot Control", QMessageBox.ButtonRole.AcceptRole)
        dlg.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        dlg.exec()
        if dlg.clickedButton() == go_btn:
            self.tabs.setCurrentIndex(0)

    def _launch_robosim(self):
        """Launch RoboSim5 as a subprocess: python RobotSim5.py"""
        if self._robosim_process is not None and self._robosim_process.poll() is None:
            self._log(f"RoboSim is already running (PID {self._robosim_process.pid}).")
            return

        robosim_dir = os.path.join(_PKG_DIR, "roboapps", "RobotSim5")
        script = "RobotSim5.py"

        if not os.path.isfile(os.path.join(robosim_dir, script)):
            self._log("ERROR: RobotSim5.py not found at: " + robosim_dir)
            return

        self._log("Launching RoboSim5...")
        try:
            self._robosim_process = subprocess.Popen(
                [sys.executable, script],
                cwd=robosim_dir,
                start_new_session=True,
            )
            self._log(f"  RoboSim5 started (PID {self._robosim_process.pid}).")
        except Exception as e:
            self._log(f"ERROR launching RoboSim5: {e}")

    # --- Support dialog ---

    def _show_support_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Contact Support")
        dialog.setMinimumWidth(400)
        form_layout = QFormLayout(dialog)

        subject_input = QLineEdit()
        first_name_input = QLineEdit()
        last_name_input = QLineEdit()
        email_input = QLineEdit()
        description_input = QTextEdit()
        description_input.setMinimumHeight(100)

        form_layout.addRow("Subject:", subject_input)
        form_layout.addRow("Your First Name:", first_name_input)
        form_layout.addRow("Your Last Name:", last_name_input)
        form_layout.addRow("Your Email:", email_input)
        form_layout.addRow("Issue Description:", description_input)

        send_btn = QPushButton("Send")
        send_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; "
            "padding: 8px 24px; border-radius: 8px; font-weight: bold; }"
        )
        form_layout.addRow("", send_btn)

        def _on_send():
            subj = subject_input.text().strip()
            fname = first_name_input.text().strip()
            lname = last_name_input.text().strip()
            email = email_input.text().strip()
            desc = description_input.toPlainText().strip()

            if not all([subj, fname, lname, email, desc]):
                QMessageBox.warning(dialog, "Missing Fields",
                                    "Please fill in all fields before sending.")
                return

            import urllib.parse
            body = f"From: {fname} {lname} <{email}>\n\n{desc}"
            mailto_url = (
                f"mailto:hi@mirobot.ai"
                f"?subject={urllib.parse.quote(subj)}"
                f"&body={urllib.parse.quote(body)}"
            )
            try:
                subprocess.Popen(["open", mailto_url])
                QMessageBox.information(dialog, "Email Client Opened",
                                        "Your default email client has been opened "
                                        "with the support request pre-filled.")
                dialog.accept()
            except Exception as e:
                QMessageBox.critical(dialog, "Error",
                                     f"Could not open email client: {e}")

        send_btn.clicked.connect(_on_send)
        dialog.exec()

    # --- SSH connection guard ---

    def _require_ssh_connection(self):
        """Show warning if SSH is not connected. Returns True if connected."""
        if self.ssh_client is not None:
            return True
        dlg = QDialog(self)
        dlg.setWindowTitle("Not Connected")
        layout = QVBoxLayout(dlg)
        msg = QLabel(
            "Please click on the Connect button in Robot Control tab\n"
            "to establish a connection to the robot first."
        )
        msg.setWordWrap(True)
        layout.addWidget(msg)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.setStyleSheet(
            "QPushButton { padding: 6px 20px; border: 2px solid #007AFF; "
            "border-radius: 8px; color: #007AFF; }"
        )
        cancel_btn.clicked.connect(dlg.reject)
        btn_row = QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(cancel_btn)
        layout.addLayout(btn_row)
        dlg.exec()
        return False

    # --- Launch File dialog ---

    def _show_launch_file_dialog(self):
        if not self._require_ssh_connection():
            return

        dialog = QDialog(self)
        dialog.setWindowTitle("Available Launch Files")
        dialog.setMinimumWidth(500)
        dialog.setMinimumHeight(400)

        layout = QVBoxLayout(dialog)

        # Top row with Add button
        top_row = QHBoxLayout()
        top_row.addStretch()
        add_btn = QPushButton("Add")
        add_btn.setStyleSheet(
            "QPushButton { background-color: #007AFF; color: white; "
            "padding: 6px 20px; border-radius: 8px; font-weight: bold; }"
        )
        top_row.addWidget(add_btn)
        layout.addLayout(top_row)

        # Discover launch files via SSH
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        file_layout = QVBoxLayout(container)

        checkboxes = []
        try:
            stdin, stdout, stderr = self.ssh_client.exec_command(
                'find ~/wheeltec_ros2/install -name "*.launch.py" 2>/dev/null'
            )
            files = stdout.read().decode().strip().split('\n')
            files = [f for f in files if f.strip()]

            if not files:
                file_layout.addWidget(QLabel("No launch files found."))
            else:
                for filepath in sorted(files):
                    row = QHBoxLayout()
                    label = QLabel(filepath)
                    label.setStyleSheet("font-size: 12px;")
                    cb = QCheckBox()
                    cb.setProperty("filepath", filepath)
                    row.addWidget(cb)
                    row.addWidget(label)
                    row.addStretch()
                    file_layout.addLayout(row)
                    checkboxes.append(cb)
        except Exception as e:
            file_layout.addWidget(QLabel(f"Error discovering files: {e}"))

        file_layout.addStretch()
        scroll.setWidget(container)
        layout.addWidget(scroll)

        def _on_add():
            selected = [cb for cb in checkboxes if cb.isChecked()]
            if not selected:
                QMessageBox.warning(dialog, "No Selection",
                                    "Please select at least one launch file.")
                return
            for cb in selected:
                filepath = cb.property("filepath")
                # Extract package name and launch filename
                # Path pattern: .../install/<package>/share/<package>/launch/<file>.launch.py
                parts = filepath.split('/')
                launch_name = parts[-1]  # e.g. "turn_on_wheeltec_robot.launch.py"
                pkg_name = None
                for i, p in enumerate(parts):
                    if p == 'install' and i + 1 < len(parts):
                        pkg_name = parts[i + 1]
                        break
                if pkg_name is None:
                    pkg_name = "unknown_pkg"
                display_name = launch_name.replace('.launch.py', '')
                cmd = f"source ~/wheeltec_ros2/install/setup.bash && ros2 launch {pkg_name} {launch_name}"
                self._add_launch_node(display_name, cmd)
            dialog.accept()

        add_btn.clicked.connect(_on_add)
        dialog.exec()

    def _add_launch_node(self, name, cmd):
        row = QHBoxLayout()
        label = QLabel(f"  {name}")
        label.setMinimumWidth(100)
        status = QLabel("stopped")
        status.setMinimumWidth(80)
        self.node_labels[name] = status

        start_btn = QPushButton("Start")
        start_btn.setEnabled(True)
        start_btn.clicked.connect(lambda checked, n=name, c=cmd: self.start_node(n, c))
        self.node_start_btns[name] = start_btn

        stop_btn = QPushButton("Stop")
        stop_btn.setEnabled(True)
        stop_btn.clicked.connect(lambda checked, n=name: self.stop_node(n))
        self.node_stop_btns[name] = stop_btn

        remove_btn = QPushButton("Remove")
        remove_btn.setStyleSheet(
            "QPushButton { background-color: #FF3B30; color: white; "
            "padding: 4px 8px; border-radius: 8px; }"
            "QPushButton:disabled { background-color: #B0B0B0; color: #707070; "
            "border-radius: 8px; }")
        remove_btn.clicked.connect(
            lambda checked, n=name, r=row: self._remove_launch_node(n, r))

        row.addWidget(label)
        row.addWidget(status)
        row.addWidget(start_btn)
        row.addWidget(stop_btn)
        row.addWidget(remove_btn)

        # Insert before the last item in control_layout (to keep above log area)
        count = self._control_layout.count()
        self._control_layout.insertLayout(count, row)

    def _remove_launch_node(self, name, row_layout):
        """Stop and remove a user-added launch node from the Robot Control tab."""
        reply = QMessageBox.question(
            self, "Remove Launch File",
            f'Remove "{name}" from the launch list?',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        # Stop the node if running
        if name in self.node_labels and self.node_labels[name].text() == "running":
            self.stop_node(name)
        # Remove from tracking dicts
        self.node_labels.pop(name, None)
        self.node_start_btns.pop(name, None)
        self.node_stop_btns.pop(name, None)
        # Remove all widgets from the row layout
        while row_layout.count():
            item = row_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()
        # Remove the row layout from the parent
        self._control_layout.removeItem(row_layout)

    # --- Undo / Redo / Autosave ---

    def _undo(self):
        if self.editor_stack.currentIndex() == 0:
            self.simple_editor.undo()
        else:
            self.full_editor.undo()

    def _redo(self):
        if self.editor_stack.currentIndex() == 0:
            self.simple_editor.redo()
        else:
            self.full_editor.redo()

    def _autosave(self):
        """Auto-save: persist editor content to disk every 5 seconds."""
        if self.editor_stack.currentIndex() == 0:
            # Simple View — write params + logic to movement.py
            self._write_params_to_movement_py()
            self._write_simple_logic_to_movement_py()
        else:
            self._save_full_view_file()

    def _change_font_size(self, delta):
        """Adjust the font size of the active editor by *delta* points."""
        if self.editor_stack.currentIndex() == 0:
            editor = self.simple_editor
        else:
            editor = self.full_editor
        font = editor.font()
        new_size = max(8, font.pointSize() + delta)
        font.setPointSize(new_size)
        editor.setFont(font)

    def _increase_font_size(self):
        self._change_font_size(1)

    def _decrease_font_size(self):
        self._change_font_size(-1)

    # --- Simple View helpers ---

    def _generate_simple_code(self):
        """Generate simplified ROS2-style code from current parameter values."""
        fwd = self.forward_speed.value()
        bwd = self.backward_speed.value()
        turn = self.turn_speed.value()
        obs = self.obstacle_distance.value()
        cw = self.turn_cw.value()
        acw = self.turn_acw.value()
        colour = self.colour_detection.currentText()

        return (
            f'import rclpy\n'
            f'from rclpy.node import Node\n'
            f'\n'
            f'class Movement(Node):\n'
            f'    def __init__(self):\n'
            f'        super().__init__(\'movement\')\n'
            f'        # === Editable Parameters ===\n'
            f'        self.forward_speed = {fwd:.2f}       # m/s  \u2190 edit\n'
            f'        self.backward_speed = {bwd:.2f}      # m/s  \u2190 edit\n'
            f'        self.turn_speed = {turn:.2f}          # rad/s  \u2190 edit\n'
            f'        self.obstacle_distance = {obs:.2f}   # metres  \u2190 edit\n'
            f'        self.turn_cw_deg = {cw:.1f}         # degrees CW  \u2190 edit\n'
            f'        self.turn_acw_deg = {acw:.1f}        # degrees ACW  \u2190 edit\n'
            f'        self.colour_detection = "{colour}"   # Red|Blue|Yellow|Green  \u2190 edit\n'
            f'\n'
            f'    # vvv Drag and drop functions below vvv\n'
            f'\n'
            f'    def control_loop(self):\n'
            f'        # === Movement Logic ===\n'
            f'        if self.obstacle_in_front():\n'
            f'            self.stop()                       # stop movement\n'
            f'            self.turn_cw(self.turn_cw_deg)    # turn clockwise  \u2190 edit\n'
            f'        else:\n'
            f'            self.move(self.forward_speed)     # drive forward  \u2190 edit\n'
        )

    def _on_simple_code_changed(self):
        """Parse Simple View text and update spinboxes in Robot Control tab."""
        if self._syncing:
            return
        self._syncing = True
        try:
            text = self.simple_editor.toPlainText()
            m = re.search(r'self\.forward_speed\s*=\s*([\d.]+)', text)
            if m:
                self.forward_speed.setValue(float(m.group(1)))
            m = re.search(r'self\.backward_speed\s*=\s*([\d.]+)', text)
            if m:
                self.backward_speed.setValue(float(m.group(1)))
            m = re.search(r'self\.turn_speed\s*=\s*([\d.]+)', text)
            if m:
                self.turn_speed.setValue(float(m.group(1)))
            m = re.search(r'self\.obstacle_distance\s*=\s*([\d.]+)', text)
            if m:
                self.obstacle_distance.setValue(float(m.group(1)))
            m = re.search(r'self\.turn_cw_deg\s*=\s*([\d.]+)', text)
            if m:
                self.turn_cw.setValue(float(m.group(1)))
            m = re.search(r'self\.turn_acw_deg\s*=\s*([\d.]+)', text)
            if m:
                self.turn_acw.setValue(float(m.group(1)))
            m = re.search(r'self\.colour_detection\s*=\s*"([^"]+)"', text)
            if m:
                colour = m.group(1)
                if colour in ["Red", "Blue", "Yellow", "Green"]:
                    self.colour_detection.setCurrentText(colour)
        finally:
            self._syncing = False

    def _sync_simple_view_from_spinboxes(self):
        """Update Simple View parameter values in-place (preserves user logic)."""
        if self._syncing:
            return
        self._syncing = True
        try:
            code = self.simple_editor.toPlainText()
            # First launch — editor is empty, generate fresh code
            if not code.strip():
                self.simple_editor.setPlainText(self._generate_simple_code())
                return

            # In-place regex replacement of parameter values only.
            # Use count=1 to only replace the first occurrence (in __init__),
            # leaving any duplicates in the logic section untouched.
            replacements = [
                (r'(self\.forward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.forward_speed.value():.2f}'),
                (r'(self\.backward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.backward_speed.value():.2f}'),
                (r'(self\.turn_speed\s*=\s*)[\d.]+', rf'\g<1>{self.turn_speed.value():.2f}'),
                (r'(self\.obstacle_distance\s*=\s*)[\d.]+', rf'\g<1>{self.obstacle_distance.value():.2f}'),
                (r'(self\.turn_cw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_cw.value():.1f}'),
                (r'(self\.turn_acw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_acw.value():.1f}'),
                (r'(self\.colour_detection\s*=\s*")[^"]*"',
                 rf'\g<1>{self.colour_detection.currentText()}"'),
            ]
            new_code = code
            for pattern, repl in replacements:
                new_code = re.sub(pattern, repl, new_code, count=1)

            if new_code != code:
                # Save and restore cursor position
                cursor = self.simple_editor.textCursor()
                pos = cursor.position()
                self.simple_editor.setPlainText(new_code)
                cursor = self.simple_editor.textCursor()
                cursor.setPosition(min(pos, len(new_code)))
                self.simple_editor.setTextCursor(cursor)
        finally:
            self._syncing = False

    def _sync_full_view_from_spinboxes(self):
        """Apply current spinbox parameter values to the Full View editor."""
        code = self.full_editor.toPlainText()
        if not code.strip():
            return
        replacements = [
            (r'(self\.forward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.forward_speed.value():.2f}'),
            (r'(self\.backward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.backward_speed.value():.2f}'),
            (r'(self\.turn_speed\s*=\s*)[\d.]+', rf'\g<1>{self.turn_speed.value():.2f}'),
            (r'(self\.obstacle_distance\s*=\s*)[\d.]+', rf'\g<1>{self.obstacle_distance.value():.2f}'),
            (r'(self\.turn_cw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_cw.value():.1f}'),
            (r'(self\.turn_acw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_acw.value():.1f}'),
            (r'(self\.colour_detection\s*=\s*")[^"]*"',
             rf'\g<1>{self.colour_detection.currentText()}"'),
        ]
        new_code = code
        for pattern, repl in replacements:
            new_code = re.sub(pattern, repl, new_code, count=1)
        if new_code != code:
            cursor = self.full_editor.textCursor()
            pos = cursor.position()
            self.full_editor.setPlainText(new_code)
            cursor = self.full_editor.textCursor()
            cursor.setPosition(min(pos, len(new_code)))
            self.full_editor.setTextCursor(cursor)

    # --- Simple View ↔ movement.py sync engine ---

    def _extract_simple_view_logic(self):
        """Extract the control_loop body text from the Simple View editor."""
        text = self.simple_editor.toPlainText()
        lines = text.split('\n')
        logic_start = None
        for i, line in enumerate(lines):
            if '# === Movement Logic ===' in line:
                logic_start = i + 1
                break
        if logic_start is None:
            return None
        logic_lines = lines[logic_start:]
        # Strip trailing empty lines
        while logic_lines and not logic_lines[-1].strip():
            logic_lines.pop()
        if not logic_lines:
            return None
        return '\n'.join(logic_lines)

    def _write_simple_logic_to_movement_py(self):
        """Replace the control_loop user section in movement.py with Simple View logic."""
        logic = self._extract_simple_view_logic()
        if logic is None:
            return
        if not os.path.isfile(MOVEMENT_PY):
            return
        with open(MOVEMENT_PY, 'r') as f:
            code = f.read()

        marker_start = '        # user control_loop logic below\n'
        marker_end = '        # end user control_loop logic'

        start_idx = code.find(marker_start)
        end_idx = code.find(marker_end)
        if start_idx == -1 or end_idx == -1:
            return

        new_code = code[:start_idx + len(marker_start)] + logic + '\n' + code[end_idx:]

        if new_code != code:
            with open(MOVEMENT_PY, 'w') as f:
                f.write(new_code)

    def _load_simple_view_from_movement_py(self):
        """Read movement.py markers and rebuild Simple View with current params + saved logic."""
        if not os.path.isfile(MOVEMENT_PY):
            return
        with open(MOVEMENT_PY, 'r') as f:
            code = f.read()

        # Extract logic between markers
        m = re.search(
            r'        # user control_loop logic below\n(.*?)        # end user control_loop logic',
            code, re.DOTALL,
        )
        if not m:
            # No markers — fall back to generated default
            self._syncing = True
            try:
                self.simple_editor.setPlainText(self._generate_simple_code())
            finally:
                self._syncing = False
            return

        saved_logic = m.group(1).rstrip('\n')

        # Build Simple View with current spinbox params and saved logic
        base_code = self._generate_simple_code()
        lines = base_code.split('\n')
        logic_start = None
        for i, line in enumerate(lines):
            if '# === Movement Logic ===' in line:
                logic_start = i + 1
                break
        if logic_start is not None:
            new_lines = lines[:logic_start]
            new_lines.append(saved_logic)
            base_code = '\n'.join(new_lines) + '\n'

        self._syncing = True
        try:
            self.simple_editor.setPlainText(base_code)
        finally:
            self._syncing = False

    def _write_params_to_movement_py(self):
        """Write current spinbox parameter values into movement.py on disk."""
        if not os.path.isfile(MOVEMENT_PY):
            return
        with open(MOVEMENT_PY, 'r') as f:
            code = f.read()

        replacements = [
            (r'(self\.forward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.forward_speed.value():.2f}'),
            (r'(self\.backward_speed\s*=\s*)[\d.]+', rf'\g<1>{self.backward_speed.value():.2f}'),
            (r'(self\.turn_speed\s*=\s*)[\d.]+', rf'\g<1>{self.turn_speed.value():.2f}'),
            (r'(self\.obstacle_distance\s*=\s*)[\d.]+', rf'\g<1>{self.obstacle_distance.value():.2f}'),
            (r'(self\.turn_cw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_cw.value():.1f}'),
            (r'(self\.turn_acw_deg\s*=\s*)[\d.]+', rf'\g<1>{self.turn_acw.value():.1f}'),
            (r'(self\.colour_detection\s*=\s*")[^"]*"',
             rf'\g<1>{self.colour_detection.currentText()}"'),
        ]

        for pattern, repl in replacements:
            code = re.sub(pattern, repl, code, count=1)

        with open(MOVEMENT_PY, 'w') as f:
            f.write(code)

    def _sync_simple_view_to_full_view(self):
        """Persist Simple View params + logic to movement.py and refresh Full View."""
        self._write_params_to_movement_py()
        self._write_simple_logic_to_movement_py()
        # Reload Full View editor if movement.py is the currently open file
        if self._full_view_current_file == "movement_pkg/movement.py":
            with open(MOVEMENT_PY, 'r') as f:
                self.full_editor.setPlainText(f.read())

    def _show_simple_view(self):
        # If switching from Full View, save the file first
        if self.editor_stack.currentIndex() == 1:
            self._save_full_view_file()
        self.editor_stack.setCurrentIndex(0)
        self.simple_view_btn.setChecked(True)
        self.full_view_btn.setChecked(False)
        self.fv_add_btn.hide()
        self.fv_delete_btn.hide()
        self.fv_search_btn.hide()
        self._fv_search_bar.hide()
        self._fv_search_input.clear()
        # Load persisted logic from movement.py
        self._load_simple_view_from_movement_py()

    def _show_full_view(self):
        # If switching from Simple View, sync changes to movement.py and reload
        if self.editor_stack.currentIndex() == 0:
            self._sync_simple_view_to_full_view()
        self.editor_stack.setCurrentIndex(1)
        self.full_view_btn.setChecked(True)
        self.simple_view_btn.setChecked(False)
        self.fv_add_btn.show()
        self.fv_delete_btn.show()
        self.fv_search_btn.show()
        # Sync spinbox parameter values into Full View if movement.py is open
        if self._full_view_current_file == "movement_pkg/movement.py":
            self._sync_full_view_from_spinboxes()

    # --- Full View search ---

    def _fv_toggle_search(self):
        """Toggle search bar visibility in Full View."""
        visible = self._fv_search_bar.isVisible()
        self._fv_search_bar.setVisible(not visible)
        if visible:
            self._fv_search_input.clear()
            self.full_editor.setExtraSelections([])
        else:
            self._fv_search_input.setFocus()

    def _fv_perform_search(self):
        """Highlight all occurrences of the search term in the Full View editor."""
        term = self._fv_search_input.text()
        if not term:
            self.full_editor.setExtraSelections([])
            return

        selections = []
        doc = self.full_editor.document()
        highlight_fmt = QTextCharFormat()
        highlight_fmt.setBackground(QColor("#FFE082"))

        cursor = QTextCursor(doc)
        while True:
            cursor = doc.find(term, cursor)
            if cursor.isNull():
                break
            sel = QTextEdit.ExtraSelection()
            sel.cursor = cursor
            sel.format = highlight_fmt
            selections.append(sel)

        self.full_editor.setExtraSelections(selections)

    # --- Full View helpers ---

    # Directories to hide from the Full View file tree
    _FV_HIDDEN_DIRS = {"__pycache__", ".git", "resource", "msg", "srv",
                       ".egg-info"}

    def _load_file_tree(self):
        self._blocking_item_changed = True
        self.file_tree.clear()

        delete_on = self._fv_edit_mode
        if delete_on:
            self.file_tree.setColumnCount(2)
            self.file_tree.header().setStretchLastSection(False)
            self.file_tree.header().setSectionResizeMode(
                0, self.file_tree.header().ResizeMode.Stretch)
            self.file_tree.header().setSectionResizeMode(
                1, self.file_tree.header().ResizeMode.Fixed)
            self.file_tree.header().resizeSection(1, 30)
        else:
            self.file_tree.setColumnCount(1)

        folders = {}   # dir_name -> QTreeWidgetItem
        seen_files = set()
        first_file_item = None

        _protected_files = _PROTECTED_FV_FILES
        _protected_folders = _PROTECTED_FV_FOLDERS

        def _add_delete_col(tree_item, protected=False):
            if delete_on and not protected:
                tree_item.setText(1, "\u2716")
                tree_item.setForeground(1, QColor("#FF3B30"))

        def _ensure_folder(dir_name):
            """Return (or create) the QTreeWidgetItem for *dir_name*."""
            if dir_name in folders:
                return folders[dir_name]
            folder_item = QTreeWidgetItem(self.file_tree)
            folder_item.setText(0, f"\U0001F4C1 {dir_name}")
            folder_item.setFont(0, QFont("Menlo", 11, QFont.Weight.Bold))
            folder_item.setForeground(0, QColor("#34C759"))
            folder_item.setExpanded(True)
            _add_delete_col(folder_item, dir_name in _protected_folders)
            folders[dir_name] = folder_item
            return folder_item

        def _add_file(parent, file_name, rel_path):
            nonlocal first_file_item
            if rel_path in seen_files:
                return
            seen_files.add(rel_path)
            fi = QTreeWidgetItem(parent)
            fi.setText(0, file_name)
            fi.setData(0, Qt.ItemDataRole.UserRole, rel_path)
            fi.setForeground(0, QColor("#007AFF"))
            _add_delete_col(fi, rel_path in _protected_files)
            if first_file_item is None:
                first_file_item = fi

        # 1) Static files from _FULL_VIEW_FILES
        for rel_path in _FULL_VIEW_FILES:
            full_path = os.path.join(_PKG_DIR, rel_path)
            if not os.path.isfile(full_path):
                continue
            dir_name = os.path.dirname(rel_path)
            file_name = os.path.basename(rel_path)
            parent = _ensure_folder(dir_name) if dir_name else self.file_tree.invisibleRootItem()
            _add_file(parent, file_name, rel_path)

        # 2) Scan every subdirectory on disk (skip hidden / dunder)
        for entry in sorted(os.listdir(_PKG_DIR)):
            entry_path = os.path.join(_PKG_DIR, entry)
            if not os.path.isdir(entry_path):
                continue
            if entry.startswith(".") or entry.startswith("__"):
                continue
            if any(entry.endswith(h) or entry == h
                   for h in self._FV_HIDDEN_DIRS):
                continue
            parent = _ensure_folder(entry)
            for fname in sorted(os.listdir(entry_path)):
                fpath = os.path.join(entry_path, fname)
                if os.path.isfile(fpath):
                    _add_file(parent, fname, os.path.join(entry, fname))

        # 3) Canvas-only packages that have no directory yet
        for pkg_name in (self._canvas_state.packages or {}):
            if pkg_name not in folders:
                _ensure_folder(pkg_name)

        # 4) Root-level files not in _FULL_VIEW_FILES
        for entry in sorted(os.listdir(_PKG_DIR)):
            entry_path = os.path.join(_PKG_DIR, entry)
            if os.path.isfile(entry_path) and not entry.startswith("."):
                _add_file(self.file_tree.invisibleRootItem(), entry, entry)

        if first_file_item:
            self.file_tree.setCurrentItem(first_file_item)
            self._on_file_tree_clicked(first_file_item, 0)

        self._blocking_item_changed = False

    def _on_file_tree_clicked(self, item, column):
        # Edit-mode delete via red minus column
        if column == 1 and self._fv_edit_mode:
            rel_path = item.data(0, Qt.ItemDataRole.UserRole)
            is_folder = rel_path is None
            target = item.text(0).strip() if is_folder else rel_path
            if not target:
                return
            reply = QMessageBox.warning(
                self, "Delete Item",
                "Are you sure you want to delete this item? "
                "You can not undo this action.",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                return
            if is_folder:
                # Extract folder name (remove icon prefix)
                folder_display = item.text(0).strip()
                for prefix in ("\U0001F4C1 ", "\U0001F4C1"):
                    if folder_display.startswith(prefix):
                        folder_display = folder_display[len(prefix):]
                        break
                folder_path = os.path.join(_PKG_DIR, folder_display)
                if os.path.isdir(folder_path):
                    shutil.rmtree(folder_path)
                # Remove matching package and items from canvas
                self._canvas_state.packages.pop(folder_display, None)
                to_remove = [iid for iid, it
                             in self._canvas_state.items.items()
                             if it.package == folder_display]
                for iid in to_remove:
                    self._canvas_state.remove_item(iid)
            else:
                full_path = os.path.join(_PKG_DIR, rel_path)
                if os.path.isfile(full_path):
                    os.remove(full_path)
                if self._full_view_current_file == rel_path:
                    self._full_view_current_file = None
                    self.full_editor.clear()
                # Remove matching node/topic from canvas
                file_name = os.path.splitext(os.path.basename(rel_path))[0]
                pkg_name = os.path.dirname(rel_path)
                for iid, it in list(self._canvas_state.items.items()):
                    if it.package == pkg_name and it.name == file_name:
                        self._canvas_state.remove_item(iid)
                        break
            self._canvas_push_undo()
            self._populate_canvas_file_tree()
            self._load_file_tree()
            self.canvas_widget.update()
            return

        rel_path = item.data(0, Qt.ItemDataRole.UserRole)
        if not rel_path:
            return  # Clicked a folder — ignore
        # Save previously edited file
        self._save_full_view_file()
        # Load new file
        full_path = os.path.join(_PKG_DIR, rel_path)
        self._full_view_current_file = rel_path
        try:
            with open(full_path, "r") as f:
                self.full_editor.setPlainText(f.read())
        except Exception as e:
            self.full_editor.setPlainText(f"# Error loading file: {e}")

    def _select_file_tree_item(self, rel_path):
        """Select a file in the Full View tree by its relative path."""
        it = QTreeWidgetItemIterator(self.file_tree)
        while it.value():
            node = it.value()
            if node.data(0, Qt.ItemDataRole.UserRole) == rel_path:
                self.file_tree.setCurrentItem(node)
                self._on_file_tree_clicked(node, 0)
                return
            it += 1

    def _save_full_view_file(self):
        """Save the currently open Full View file to disk."""
        if not self._full_view_current_file:
            return
        full_path = os.path.join(_PKG_DIR, self._full_view_current_file)
        try:
            with open(full_path, "w") as f:
                f.write(self.full_editor.toPlainText())
        except Exception:
            pass

    def _deploy_from_editor(self):
        """Save & Deploy triggered from Code Editor tab."""
        if self.editor_stack.currentIndex() == 0:
            # Simple View — write params + logic to movement.py before deploy
            self._write_params_to_movement_py()
            self._write_simple_logic_to_movement_py()
        else:
            # Full View — save current file to disk first
            self._save_full_view_file()
            # If movement.py was edited, reload params from it
            if (self._full_view_current_file
                    and self._full_view_current_file.endswith("movement.py")):
                self._load_params()
        self.deploy()

    # ------------------------------------------------------------------ #
    #  Shared logic                                                        #
    # ------------------------------------------------------------------ #

    def _log(self, msg):
        self.log_area.append(f"> {msg}")
        self.log_area.verticalScrollBar().setValue(
            self.log_area.verticalScrollBar().maximum())

    def _set_controls_enabled(self, enabled):
        self.deploy_btn.setEnabled(enabled)
        self.editor_deploy_btn.setEnabled(enabled)
        self.canvas_deploy_btn.setEnabled(enabled)
        # save buttons (save_btn, editor_save_btn, canvas_save_btn) are always enabled
        self.start_all_btn.setEnabled(enabled)
        self.stop_all_btn.setEnabled(enabled)
        self.pause_btn.setEnabled(enabled)
        self.check_nodes_btn.setEnabled(enabled)
        self.check_topics_btn.setEnabled(enabled)
        # check_logs_btn and launch_file_btn are intentionally excluded — always active
        for btn in self.node_start_btns.values():
            btn.setEnabled(enabled)
        for btn in self.node_stop_btns.values():
            btn.setEnabled(enabled)

    def do_connect(self):
        # If already connected, treat as disconnect
        if self.connect_btn.text() == "Disconnect":
            self.do_disconnect()
            return

        ip = self.ip_input.text().strip()
        self.connect_btn.setEnabled(False)
        self.conn_status.setText("Connecting...")
        self.conn_status.setStyleSheet("color: orange; font-weight: bold;")

        # Local RViz mode — no SSH needed
        if ip == "127.0.0.1":
            self._on_connected(None)
            return

        w = ConnectWorker(
            ip,
            self.user_input.text().strip(),
            self.pass_input.text(),
        )
        w.log.connect(self._log)
        w.connected.connect(self._on_connected)
        w.failed.connect(self._on_connect_failed)
        self._workers.append(w)
        w.finished.connect(lambda: self._workers.remove(w))
        w.start()

    def _on_connected(self, client):
        self.ssh_client = client
        is_local = (self.ip_input.text().strip() == "127.0.0.1")
        self.conn_status.setText("Connected (Local)" if is_local else "Connected")
        self.conn_status.setStyleSheet("color: green; font-weight: bold;")
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Disconnect")
        self.connect_btn.setStyleSheet(
            "background-color: #FF3B30; color: white; padding: 8px; border-radius: 8px;"
        )
        if not is_local:
            self._set_controls_enabled(True)
            self._save_current_profile()

    def _on_connect_failed(self, error):
        self._log(f"Connection failed: {error}")
        self.conn_status.setText("Disconnected")
        self.conn_status.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet(
            "background-color: #007AFF; color: white; padding: 8px; border-radius: 8px;"
        )

    def do_disconnect(self):
        if self.ssh_client:
            try:
                self.ssh_client.close()
            except Exception:
                pass
            self.ssh_client = None
        self.conn_status.setText("Disconnected")
        self.conn_status.setStyleSheet("color: red; font-weight: bold;")
        self.connect_btn.setText("Connect")
        self.connect_btn.setStyleSheet(
            "background-color: #007AFF; color: white; padding: 8px; border-radius: 8px;"
        )
        self._set_controls_enabled(False)

    # --- Saved connection profiles ---

    def _load_profiles(self):
        if os.path.isfile(PROFILES_FILE):
            with open(PROFILES_FILE, "r") as f:
                return json.load(f)
        return []

    def _save_profiles(self, profiles):
        with open(PROFILES_FILE, "w") as f:
            json.dump(profiles, f, indent=2)

    def _set_local_rviz_mode(self, is_local):
        """Dim username/password when local RViz (127.0.0.1) is selected."""
        self.user_input.setEnabled(not is_local)
        self.pass_input.setEnabled(not is_local)
        grey = "background-color: #E8E8E8; color: #A0A0A0;"
        self.user_input.setStyleSheet(grey if is_local else "")
        self.pass_input.setStyleSheet(grey if is_local else "")

    def _refresh_profile_combo(self):
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        self.profile_combo.addItem("Saved connections...")

        profiles = self._load_profiles()
        # User profiles are all non-default IPs, displayed newest-first
        user_profiles = [p for p in profiles if p["ip"] != DEFAULT_IP]
        for p in reversed(user_profiles):
            display_name = p.get("name", "") or p.get("username", p["ip"])
            self.profile_combo.addItem(f"{p['ip']}  ({display_name})")

        # Pinned local RViz entry
        self.profile_combo.addItem("127.0.0.1  (RViz)")
        # Pinned default robot IP
        self.profile_combo.addItem(f"{DEFAULT_IP}  (robot hotspot)")
        self.profile_combo.addItem("Edit IP...")

        n = len(user_profiles)
        # Auto-select newest user profile, or robot hotspot if none
        if user_profiles:
            newest = user_profiles[-1]
            self.ip_input.setText(newest["ip"])
            self.user_input.setText(newest["username"])
            self.pass_input.setText(newest["password"])
            self.profile_combo.setCurrentIndex(1)
        else:
            self.ip_input.setText(DEFAULT_IP)
            default_profile = next((p for p in profiles if p["ip"] == DEFAULT_IP), None)
            if default_profile:
                self.user_input.setText(default_profile["username"])
                self.pass_input.setText(default_profile["password"])
            self.profile_combo.setCurrentIndex(n + 2)  # robot hotspot

        self.profile_combo.blockSignals(False)

    def _on_profile_selected(self, index):
        if index <= 0:
            return
        profiles = self._load_profiles()
        user_profiles = [p for p in profiles if p["ip"] != DEFAULT_IP]
        n = len(user_profiles)

        if 1 <= index <= n:
            # User profile (displayed newest-first, so reverse the index)
            p = user_profiles[n - index]
            self.ip_input.setText(p["ip"])
            self.user_input.setText(p["username"])
            self.pass_input.setText(p["password"])
            self._set_local_rviz_mode(False)
        elif index == n + 1:
            # Pinned local RViz entry
            self.ip_input.setText("127.0.0.1")
            self._set_local_rviz_mode(True)
        elif index == n + 2:
            # Pinned default robot hotspot IP
            default_profile = next((p for p in profiles if p["ip"] == DEFAULT_IP), None)
            if default_profile:
                self.ip_input.setText(default_profile["ip"])
                self.user_input.setText(default_profile["username"])
                self.pass_input.setText(default_profile["password"])
            else:
                self.ip_input.setText(DEFAULT_IP)
            self._set_local_rviz_mode(False)
        elif index == n + 3:
            # "Edit IP..." — reset selection then open dialog
            self.profile_combo.blockSignals(True)
            self.profile_combo.setCurrentIndex(1 if n > 0 else n + 2)
            self.profile_combo.blockSignals(False)
            self._show_edit_ips_dialog()

    def _save_current_profile(self):
        ip = self.ip_input.text().strip()
        username = self.user_input.text().strip()
        password = self.pass_input.text()
        if not ip:
            return
        profiles = self._load_profiles()
        # Remove existing entry for this IP (if any) so we can append it at the end
        # (end of list = most recently used, auto-selected on next launch)
        existing = next((p for p in profiles if p["ip"] == ip), None)
        profiles = [p for p in profiles if p["ip"] != ip]
        name = existing.get("name", "") if existing else ""
        profiles.append({"ip": ip, "username": username, "password": password, "name": name})
        self._save_profiles(profiles)
        self._refresh_profile_combo()

    def _show_edit_ips_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Saved IPs")
        dialog.setMinimumWidth(460)
        layout = QVBoxLayout(dialog)
        layout.setSpacing(8)

        profiles = self._load_profiles()
        user_profiles = [p for p in profiles if p["ip"] != DEFAULT_IP]

        # Table: IP Address | Name | (delete)
        table = QTableWidget()
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["IP Address", "Name", ""])
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
        table.setColumnWidth(2, 40)
        table.verticalHeader().setVisible(False)
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setEditTriggers(
            QAbstractItemView.EditTrigger.DoubleClicked |
            QAbstractItemView.EditTrigger.SelectedClicked
        )

        PINNED = [
            {"ip": DEFAULT_IP, "name": "robot hotspot"},
            {"ip": "127.0.0.1",  "name": "RViz"},
        ]
        n_pinned = len(PINNED)
        table.setRowCount(n_pinned + len(user_profiles))

        # Pinned rows — non-editable, grayed out
        for row, entry in enumerate(PINNED):
            for col, text in enumerate([entry["ip"], entry["name"]]):
                item = QTableWidgetItem(text)
                item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
                item.setForeground(QBrush(QColor("#A0A0A0")))
                table.setItem(row, col, item)
            lock = QLabel("\U0001f512")
            lock.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lock.setStyleSheet("color: #A0A0A0;")
            table.setCellWidget(row, 2, lock)

        # User profile rows — editable, with delete button
        original_ips = {}
        for i, p in enumerate(user_profiles):
            row = n_pinned + i
            original_ips[row] = p["ip"]
            name = p.get("name", "")
            table.setItem(row, 0, QTableWidgetItem(p["ip"]))
            table.setItem(row, 1, QTableWidgetItem(name))

            del_btn = QPushButton("\u2715")
            del_btn.setFixedSize(28, 28)
            del_btn.setStyleSheet(
                "QPushButton { background-color: #FF3B30; color: white; "
                "border-radius: 14px; font-weight: bold; font-size: 12px; }"
                "QPushButton:hover { background-color: #FF6B6B; }"
            )

            def make_delete_handler(profile_ip):
                def handler():
                    reply = QMessageBox.question(
                        dialog,
                        "Delete IP",
                        "Are you sure you want to delete this IP?\nThis action is not revertible.",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    )
                    if reply == QMessageBox.StandardButton.Yes:
                        updated = [pr for pr in self._load_profiles() if pr["ip"] != profile_ip]
                        self._save_profiles(updated)
                        self._refresh_profile_combo()
                        dialog.accept()
                        self._show_edit_ips_dialog()
                return handler

            del_btn.clicked.connect(make_delete_handler(p["ip"]))
            wrapper = QWidget()
            wl = QHBoxLayout(wrapper)
            wl.setContentsMargins(4, 2, 4, 2)
            wl.addWidget(del_btn)
            table.setCellWidget(row, 2, wrapper)

        layout.addWidget(table)

        # Save edits on cell change
        def on_cell_changed(row, col):
            if row < n_pinned or row not in original_ips:
                return
            ip_item   = table.item(row, 0)
            name_item = table.item(row, 1)
            new_ip   = ip_item.text().strip()   if ip_item   else ""
            new_name = name_item.text().strip() if name_item else ""
            if not new_ip:
                return
            old_ip = original_ips[row]
            all_profiles = self._load_profiles()
            for pr in all_profiles:
                if pr["ip"] == old_ip:
                    pr["ip"]   = new_ip
                    pr["name"] = new_name
                    original_ips[row] = new_ip
                    break
            self._save_profiles(all_profiles)
            self._refresh_profile_combo()

        table.cellChanged.connect(on_cell_changed)

        close_btn = QPushButton("Close")
        close_btn.setStyleSheet("padding: 6px; margin-top: 4px;")
        close_btn.clicked.connect(dialog.accept)
        layout.addWidget(close_btn)

        dialog.exec()

    def _load_params(self):
        if not os.path.isfile(MOVEMENT_PY):
            self._log(f"WARNING: {MOVEMENT_PY} not found. Using defaults.")
            self.forward_speed.setValue(0.2)
            self.backward_speed.setValue(0.2)
            self.turn_speed.setValue(0.5)
            self.obstacle_distance.setValue(0.30)
            self.turn_cw.setValue(90.0)
            self.turn_acw.setValue(90.0)
            return

        with open(MOVEMENT_PY, "r") as f:
            code = f.read()

        self._syncing = True
        try:
            m = re.search(r"self\.forward_speed\s*=\s*([\d.]+)", code)
            self.forward_speed.setValue(float(m.group(1)) if m else 0.2)
            m = re.search(r"self\.backward_speed\s*=\s*([\d.]+)", code)
            self.backward_speed.setValue(float(m.group(1)) if m else 0.2)
            m = re.search(r"self\.turn_speed\s*=\s*([\d.]+)", code)
            self.turn_speed.setValue(float(m.group(1)) if m else 0.5)
            m = re.search(r"self\.obstacle_distance\s*=\s*([\d.]+)", code)
            self.obstacle_distance.setValue(float(m.group(1)) if m else 0.30)
            m = re.search(r"self\.turn_cw_deg\s*=\s*([\d.]+)", code)
            self.turn_cw.setValue(float(m.group(1)) if m else 90.0)
            m = re.search(r"self\.turn_acw_deg\s*=\s*([\d.]+)", code)
            self.turn_acw.setValue(float(m.group(1)) if m else 90.0)
            m = re.search(r'self\.colour_detection\s*=\s*"([^"]*)"', code)
            if m and m.group(1) in ["Red", "Blue", "Yellow", "Green"]:
                self.colour_detection.setCurrentText(m.group(1))
        finally:
            self._syncing = False

        self._load_simple_view_from_movement_py()
        self._log("Loaded parameters from movement.py")

    def _flash_save_buttons(self):
        """Briefly show 'Saved ✓' on all Save buttons, then revert."""
        for btn in (self.save_btn, self.canvas_save_btn, self.editor_save_btn):
            btn.setText("Saved ✓")
            btn.setStyleSheet(
                "background-color: #248A3D; color: white; padding: 8px; border-radius: 8px;"
            )
        QTimer.singleShot(2000, self._revert_save_buttons)

    def _revert_save_buttons(self):
        for btn in (self.save_btn, self.canvas_save_btn, self.editor_save_btn):
            btn.setText("Save")
            btn.setStyleSheet(
                "background-color: #34C759; color: white; padding: 8px; border-radius: 8px;"
            )

    def _flash_deploy_buttons(self):
        """Briefly show 'Deployed ✓' on all Deploy buttons, then revert."""
        for btn in (self.deploy_btn, self.editor_deploy_btn, self.canvas_deploy_btn):
            btn.setText("Deployed ✓")
            btn.setStyleSheet(
                "QPushButton { background-color: #248A3D; color: white; padding: 8px; border-radius: 8px; }"
            )
        QTimer.singleShot(2000, self._revert_deploy_buttons)

    def _revert_deploy_buttons(self):
        for btn in (self.deploy_btn, self.editor_deploy_btn, self.canvas_deploy_btn):
            btn.setText("Deploy")
            btn.setStyleSheet(
                "QPushButton { background-color: #007AFF; color: white; padding: 8px; border-radius: 8px; }"
                "QPushButton:disabled { background-color: #B0B0B0; color: #707070; border-radius: 8px; }"
            )

    def _run_worker(self, worker):
        worker.log.connect(self._log)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker))
        worker.start()

    def save(self):
        """Save current changes to the local project folder."""
        self._write_params_to_movement_py()
        self._write_simple_logic_to_movement_py()
        self._log("Saved to project folder.")
        self._flash_save_buttons()

    def _save_from_editor(self):
        """Save triggered from Code Editor tab — no SSH required."""
        if self.editor_stack.currentIndex() == 0:
            self._write_params_to_movement_py()
            self._write_simple_logic_to_movement_py()
        else:
            self._save_full_view_file()
            if (self._full_view_current_file
                    and self._full_view_current_file.endswith("movement.py")):
                self._load_params()
        self._log("Saved to project folder.")
        self._flash_save_buttons()

    def deploy(self):
        if not self.ssh_client:
            self._log("ERROR: Not connected to robot.")
            return
        self.deploy_btn.setEnabled(False)
        self.editor_deploy_btn.setEnabled(False)
        self.canvas_deploy_btn.setEnabled(False)
        self._log("--- Starting deploy ---")
        params = {
            "forward_speed": self.forward_speed.value(),
            "backward_speed": self.backward_speed.value(),
            "turn_speed": self.turn_speed.value(),
            "obstacle_distance": self.obstacle_distance.value(),
            "turn_cw_deg": self.turn_cw.value(),
            "turn_acw_deg": self.turn_acw.value(),
            "colour_detection": self.colour_detection.currentText(),
        }

        for btn in (self.deploy_btn, self.editor_deploy_btn, self.canvas_deploy_btn):
            btn.setText("Deploying...")
        w = DeployWorker(self.ssh_client, MOVEMENT_PY, params)
        w.finished.connect(lambda: self.deploy_btn.setEnabled(True))
        w.finished.connect(lambda: self.editor_deploy_btn.setEnabled(True))
        w.finished.connect(lambda: self.canvas_deploy_btn.setEnabled(True))
        w.finished.connect(self._flash_deploy_buttons)
        self._run_worker(w)

    def start_all(self):
        if not self.ssh_client:
            self._log("ERROR: Not connected to robot.")
            return
        self.start_all_btn.setEnabled(False)
        self._log("--- Starting all nodes ---")
        w = StartAllWorker(self.ssh_client)
        w.log.connect(self._log)
        w.finished.connect(lambda: self.start_all_btn.setEnabled(True))
        self._workers.append(w)
        w.finished.connect(lambda: self._workers.remove(w))
        w.start()

    def stop_all(self):
        if not self.ssh_client:
            self._log("ERROR: Not connected to robot.")
            return
        self.stop_all_btn.setEnabled(False)
        self._log("--- Stopping all nodes ---")
        w = StopAllWorker(self.ssh_client)
        w.log.connect(self._log)
        w.finished.connect(lambda: self.stop_all_btn.setEnabled(True))
        self._workers.append(w)
        w.finished.connect(lambda: self._workers.remove(w))
        w.start()

    def pause_movement(self):
        if not self.ssh_client:
            self._log("ERROR: Not connected to robot.")
            return
        self.pause_btn.setEnabled(False)
        self._log("--- Pausing robot movement ---")
        # 1) Set pause flag so movement node publishes zero if still alive
        # 2) Kill the movement launch + node processes
        # 3) Source workspace, wait for DDS discovery, flood zero velocity
        cmd = (
            "touch /tmp/movement_pause ; "
            "pkill -9 -f 'movement_pkg.launch.py' ; "
            "pkill -9 -f 'movement_pkg/movement' ; "
            "sleep 0.5 ; "
            + ROS_SOURCE_CMD +
            " && source ~/wheeltec_ros2/install/setup.bash && "
            "python3 -c \""
            "import rclpy, time; "
            "from geometry_msgs.msg import Twist; "
            "rclpy.init(); "
            "n = rclpy.create_node('_pause_stop'); "
            "p = n.create_publisher(Twist, '/cmd_vel', 10); "
            "t = Twist(); "
            "time.sleep(2); "
            "end = time.time() + 3; "
            "[p.publish(t) or time.sleep(0.05) for _ in iter(lambda: time.time() < end, False)]; "
            "n.destroy_node(); rclpy.shutdown()\""
        )
        w = SSHCmdWorker(self.ssh_client, cmd, "Pause robot movement")
        w.finished.connect(lambda: self.pause_btn.setEnabled(True))
        self._run_worker(w)

    def start_node(self, name, cmd):
        if not self.ssh_client:
            return
        launch_cmd = f"nohup bash -c '{cmd}' > /tmp/{name}.log 2>&1 &"
        w = SSHCmdWorker(self.ssh_client, launch_cmd, f"Start {name}")
        self._run_worker(w)

    def stop_node(self, name):
        if not self.ssh_client:
            return
        w = SSHCmdWorker(self.ssh_client,
                         f"pkill -f 'ros2 launch.*{name}' ; true",
                         f"Stop {name}")
        self._run_worker(w)

    def check_ros2_nodes(self):
        if not self.ssh_client:
            return
        self._log("--- Checking ROS2 nodes ---")
        w = SSHCmdWorker(
            self.ssh_client,
            ROS_SOURCE_CMD + " && ros2 node list 2>&1",
            "ROS2 node list",
        )
        self._run_worker(w)

    def check_ros2_topics(self):
        if not self.ssh_client:
            return
        self._log("--- Checking ROS2 topics ---")
        cmd = (
            ROS_SOURCE_CMD + " && "
            "ros2 topic list 2>&1 && "
            "echo '--- Checking /tf rate ---' && "
            "timeout 4 ros2 topic hz /tf 2>&1 | head -5 ; "
            "echo '--- Checking /odom rate ---' && "
            "timeout 4 ros2 topic hz /odom 2>&1 | head -5"
        )
        w = SSHCmdWorker(
            self.ssh_client,
            cmd,
            "ROS2 topic list + TF/odom check",
        )
        self._run_worker(w)

    def check_launch_logs(self):
        # --- Git activity section (always shown) ---
        self._log("--- Git Activity ---")
        r = subprocess.run(
            ["git", "log", "--oneline", "-5"],
            cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode == 0 and r.stdout.strip():
            for line in r.stdout.strip().splitlines():
                self._log(f"  {line}")
        else:
            self._log("  No commits yet" + (f": {r.stderr.strip()}" if r.stderr.strip() else ""))

        r = subprocess.run(
            ["git", "status", "--short"],
            cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode == 0:
            status_out = r.stdout.strip()
            self._log(f"  git status: {status_out if status_out else 'clean (nothing to commit)'}")
        else:
            self._log(f"  git status error: {r.stderr.strip()}")

        if not self.ssh_client:
            # No SSH — show local Gazebo simulation log instead
            gazebo_log = os.path.join(_PKG_DIR, ".gazebo_launch.log")
            self._log("--- Gazebo simulation log (local) ---")
            if not os.path.isfile(gazebo_log):
                self._log("No Gazebo log found. Launch Gazebo first.")
                return
            try:
                with open(gazebo_log, "r") as f:
                    lines = f.readlines()
                last_lines = lines[-50:] if len(lines) > 50 else lines
                for line in last_lines:
                    self._log(line.rstrip())
            except Exception as e:
                self._log(f"Error reading Gazebo log: {e}")
            return

        self._log("--- Checking launch logs (last 5 lines each) ---")
        for name, _ in NODES:
            w = SSHCmdWorker(
                self.ssh_client,
                f"echo '=== {name} ===' && tail -5 /tmp/{name}.log 2>&1 || echo 'No log file'",
                f"Log: {name}",
            )
            self._run_worker(w)


    # --- RViz2 integration ---

    def _find_conda_env(self):
        """Locate the conda 'ros_env' environment.

        Returns (conda_prefix, error_message). On success error_message is None.
        """
        conda_exe = shutil.which("conda")
        if conda_exe is None:
            candidates = [
                os.path.expanduser("~/miniforge3/condabin/conda"),
                os.path.expanduser("~/miniforge3/bin/conda"),
                os.path.expanduser("~/miniconda3/condabin/conda"),
                os.path.expanduser("~/miniconda3/bin/conda"),
                os.path.expanduser("~/anaconda3/condabin/conda"),
                os.path.expanduser("~/anaconda3/bin/conda"),
                "/opt/homebrew/Caskroom/miniforge/base/condabin/conda",
            ]
            for c in candidates:
                if os.path.isfile(c):
                    conda_exe = c
                    break

        if conda_exe is None:
            return None, (
                "Conda not found.\n\n"
                "Please install Miniforge first, then run:\n"
                "  bash setup_robostack.sh\n\n"
                "(from the project directory)"
            )

        try:
            result = subprocess.run(
                [conda_exe, "env", "list", "--json"],
                capture_output=True, text=True, timeout=15,
            )
            envs = json.loads(result.stdout).get("envs", [])
            for env_path in envs:
                if env_path.endswith("/ros_env"):
                    return env_path, None
        except Exception as e:
            return None, f"Error querying conda environments: {e}"

        return None, (
            "Conda environment 'ros_env' not found.\n\n"
            "Run this in Terminal:\n"
            "  bash setup_robostack.sh\n\n"
            "(from the project directory)"
        )

    def _get_local_ip_for(self, remote_ip):
        """Return the Mac's local IP address used to reach remote_ip (picks the right interface)."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((remote_ip, 1))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def _generate_fastdds_xml(self, robot_ip, mac_ip=None):
        """Generate FastDDS XML config with unicast initial peers for all robot participants.

        In ROS2/FastDDS each node is a separate DDS participant listening on its own
        metatraffic port: 7410 + 2*k  (domain 0, participant k).
        Enumerating ports 7410-7472 covers up to 32 nodes on the robot.

        NOTE: defaultUnicastLocatorList / metatrafficUnicastLocatorList are intentionally
        omitted — specifying them without a port causes FastDDS to use port=0 which
        silently breaks all data reception.  FastDDS auto-detects the correct local
        interface via the socket used to reach robot_ip.
        """
        locators = "\n".join(
            f"          <locator><udpv4>"
            f"<address>{robot_ip}</address>"
            f"<port>{7410 + 2 * k}</port>"
            f"</udpv4></locator>"
            for k in range(32)  # covers participants 0-31 (ports 7410-7472)
        )

        xml_content = f"""\
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
{locators}
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
"""
        xml_path = os.path.join(_PKG_DIR, ".fastdds.xml")
        with open(xml_path, "w") as f:
            f.write(xml_content)
        return xml_path

    def _generate_cyclonedds_xml(self, robot_ip):
        """Generate CycloneDDS XML config for unicast discovery with the robot."""
        xml_content = f"""\
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
            https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="{robot_ip}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
        xml_path = os.path.join(_PKG_DIR, ".cyclonedds.xml")
        with open(xml_path, "w") as f:
            f.write(xml_content)
        return xml_path

    def _setup_ament_package_for_rviz(self, env):
        """Create a minimal ament index entry so RViz resolves package://movement_pkg/meshes/*.

        The robot's URDF references meshes as package://movement_pkg/meshes/...
        RViz2 uses ament_index_cpp to look them up.  We create a fake ament
        prefix under _PKG_DIR/.ament_index/ with the required marker file and
        a symlink to the real meshes directory, then prepend it to AMENT_PREFIX_PATH.
        """
        ament_root = os.path.join(_PKG_DIR, ".ament_index")
        pkg_share   = os.path.join(ament_root, "share", "movement_pkg")
        index_dir   = os.path.join(ament_root, "share", "ament_index",
                                   "resource_index", "packages")
        meshes_dst  = os.path.join(pkg_share, "meshes")
        meshes_src  = os.path.join(_PKG_DIR, "meshes")

        os.makedirs(pkg_share,  exist_ok=True)
        os.makedirs(index_dir,  exist_ok=True)

        # Marker file telling ament this package exists
        marker = os.path.join(index_dir, "movement_pkg")
        if not os.path.isfile(marker):
            open(marker, "w").close()

        # Symlink meshes/ so package://movement_pkg/meshes/x.STL resolves
        if not os.path.exists(meshes_dst) and os.path.isdir(meshes_src):
            os.symlink(meshes_src, meshes_dst)

        existing = env.get("AMENT_PREFIX_PATH", "")
        if ament_root not in existing:
            env["AMENT_PREFIX_PATH"] = (ament_root + ":" + existing).rstrip(":")

    def _launch_rviz(self):
        """Launch RViz2 in a separate window via the conda ros_env environment."""
        # Guard against double-launch
        if self._rviz_process is not None and self._rviz_process.poll() is None:
            self._log(f"RViz2 is already running (PID {self._rviz_process.pid}).")
            return

        # Find conda environment
        conda_prefix, error = self._find_conda_env()
        if conda_prefix is None:
            self._show_conda_not_installed_dialog()
            return
        self._add_conda_to_roboapps()

        # Get robot IP
        robot_ip = self.ip_input.text().strip()
        if not robot_ip:
            self._log("ERROR: Enter a robot IP address before launching RViz2.")
            return

        # Build subprocess environment
        env = os.environ.copy()
        conda_bin = os.path.join(conda_prefix, "bin")
        conda_lib = os.path.join(conda_prefix, "lib")

        env["PATH"] = conda_bin + ":" + env.get("PATH", "")
        env["CONDA_PREFIX"] = conda_prefix
        env["CONDA_DEFAULT_ENV"] = "ros_env"

        # ROS2 environment variables
        env["AMENT_PREFIX_PATH"] = conda_prefix
        env["CMAKE_PREFIX_PATH"] = conda_prefix
        env["COLCON_PREFIX_PATH"] = conda_prefix

        # Auto-detect Python version in conda env
        py_dirs = glob.glob(os.path.join(conda_prefix, "lib", "python3.*", "site-packages"))
        if py_dirs:
            env["PYTHONPATH"] = py_dirs[0] + ":" + env.get("PYTHONPATH", "")

        env["LD_LIBRARY_PATH"] = conda_lib + ":" + env.get("LD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = conda_lib + ":" + env.get("DYLD_LIBRARY_PATH", "")

        # DDS configuration — detect what the robot actually uses so we match it
        env.pop("ROS_LOCALHOST_ONLY", None)        # never restrict to loopback

        robot_rmw = "rmw_fastrtps_cpp"            # default assumption
        robot_domain = "0"
        if robot_ip != "127.0.0.1" and self.ssh_client:
            try:
                rmw_cmd = (
                    ROS_SOURCE_CMD + " && "
                    "echo \"RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}\" && "
                    "echo \"DOMAIN=${ROS_DOMAIN_ID:-0}\""
                )
                _, stdout, _ = self.ssh_client.exec_command(rmw_cmd, timeout=5)
                out = stdout.read().decode()
                for line in out.splitlines():
                    if line.startswith("RMW="):
                        v = line.split("=", 1)[1].strip()
                        if v:
                            robot_rmw = v
                    elif line.startswith("DOMAIN="):
                        v = line.split("=", 1)[1].strip()
                        if v.isdigit():
                            robot_domain = v
            except Exception:
                pass

        env["ROS_DOMAIN_ID"] = robot_domain

        if robot_ip == "127.0.0.1":
            # Local Gazebo: use CycloneDDS with default multicast on loopback
            env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            self._log("  DDS: CycloneDDS (local Gazebo)")
        elif "cyclone" in robot_rmw.lower():
            # Robot uses CycloneDDS — match it with unicast peer config
            env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
            cyclone_xml = self._generate_cyclonedds_xml(robot_ip)
            env["CYCLONEDDS_URI"] = cyclone_xml
            env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
            env.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
            self._log(f"  DDS: CycloneDDS (matched robot)  domain={robot_domain}")
        else:
            # Robot uses FastDDS — unicast initialPeersList so discovery works over WiFi
            mac_ip = self._get_local_ip_for(robot_ip)
            env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
            env.pop("CYCLONEDDS_URI", None)
            fastdds_xml = self._generate_fastdds_xml(robot_ip)
            env["FASTRTPS_DEFAULT_PROFILES_FILE"] = fastdds_xml
            env["FASTDDS_DEFAULT_PROFILES_FILE"] = fastdds_xml
            # UDPv4 transport avoids shared-memory issues on macOS (FastDDS 3.x)
            env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
            self._log(f"  DDS: FastDDS (matched robot)  robot={robot_ip}  mac={mac_ip or 'unknown'}  domain={robot_domain}")

        # Register movement_pkg as a fake ament package so RViz can resolve
        # package://movement_pkg/meshes/*.STL → _PKG_DIR/meshes/*.STL
        self._setup_ament_package_for_rviz(env)

        # Build command
        rviz_config = os.path.join(_PKG_DIR, "default_view.rviz")
        rviz_exe = os.path.join(conda_bin, "rviz2")
        cmd = [rviz_exe, "-d", rviz_config]

        rviz_log = os.path.join(_PKG_DIR, ".rviz2.log")
        self._log(f"Launching RViz2 ({'local Gazebo' if robot_ip == '127.0.0.1' else robot_ip})...")
        self._log(f"  conda env: {conda_prefix}")
        self._log(f"  RViz2 log:  {rviz_log}")

        try:
            rviz_log_fh = open(rviz_log, "w")
            self._rviz_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=rviz_log_fh,
                start_new_session=True,
            )
            self._log(f"  RViz2 started (PID {self._rviz_process.pid}).")
            # After a short delay, run ros2 topic list locally to confirm DDS discovery
            QTimer.singleShot(4000, lambda: self._check_local_ros2_topics(env))
        except FileNotFoundError:
            self._log(
                "ERROR: rviz2 executable not found in ros_env.\n"
                "  Run: conda activate ros_env && conda install ros-humble-rviz2"
            )
        except Exception as e:
            self._log(f"ERROR launching RViz2: {e}")

    def _check_local_ros2_topics(self, rviz_env):
        """Run ros2 topic list locally (same DDS env as RViz) and log the result.

        Called 4 s after RViz launches so DDS has time to discover peers.
        If the topic list is empty, DDS discovery is not working — the RViz
        window will be showing 'No data' for every display.
        """
        ros2_exe = os.path.join(rviz_env.get("CONDA_PREFIX", ""), "bin", "ros2")
        if not os.path.isfile(ros2_exe):
            return
        self._log("--- Local DDS topic check (4 s after RViz start) ---")
        try:
            r = subprocess.run(
                [ros2_exe, "topic", "list"],
                env=rviz_env,
                capture_output=True, text=True, timeout=8,
            )
            topics = [t for t in r.stdout.strip().splitlines() if t.startswith("/")]
            if topics:
                self._log(f"  DDS OK — {len(topics)} topic(s) visible:")
                for t in topics:
                    self._log(f"    {t}")
            else:
                self._log(
                    "  DDS WARNING: no topics found on the Mac side.\n"
                    "  RViz will show empty displays.\n"
                    "  Check: same ROS_DOMAIN_ID on robot and Mac?\n"
                    "         Robot firewall blocking UDP?\n"
                    "         Robot using a different DDS middleware?"
                )
                if r.stderr.strip():
                    self._log(f"  stderr: {r.stderr.strip()[:300]}")
        except subprocess.TimeoutExpired:
            self._log("  DDS check timed out (ros2 topic list took > 8 s).")
        except Exception as e:
            self._log(f"  DDS check error: {e}")

    def _stop_gazebo(self):
        """Kill any running Gazebo processes (tracked and stale) and free port 11345."""
        if self._gazebo_process is not None:
            try:
                os.killpg(os.getpgid(self._gazebo_process.pid), signal.SIGTERM)
            except Exception:
                pass
            try:
                self._gazebo_process.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._gazebo_process.pid), signal.SIGKILL)
                except Exception:
                    pass
            self._gazebo_process = None
        if hasattr(self, '_gazebo_log_fh') and self._gazebo_log_fh:
            try:
                self._gazebo_log_fh.close()
            except Exception:
                pass
            self._gazebo_log_fh = None
        # Kill any stale gzserver left over from a previous session (frees port 11345).
        # gzclient is intentionally not pkill'd — killing it via pkill on macOS
        # confuses Launch Services and triggers an "open document" dialog on relaunch.
        # It will exit on its own once gzserver is gone.
        subprocess.run(['pkill', '-9', '-f', 'gzserver'], capture_output=True)
        time.sleep(2.0)  # give gzserver time to fully release port 11345

    def _launch_gazebo(self):
        """Launch Gazebo simulation in a separate process via the conda ros_env.
        If Gazebo is already running, just bring back the Simulation Control window."""
        gazebo_handle_alive = (
            self._gazebo_process is not None
            and self._gazebo_process.poll() is None
        )
        gzserver_alive = (
            subprocess.run(['pgrep', '-f', 'gzserver'], capture_output=True).returncode == 0
        )
        gzclient_alive = (
            subprocess.run(['pgrep', '-f', 'gzclient'], capture_output=True).returncode == 0
        )
        if gazebo_handle_alive and gzserver_alive and gzclient_alive:
            # Both server and client are running — just reopen the dialog
            self._log("Gazebo is already running. Reopening Simulation Control.")
            self._show_sim_initiation_dialog()
            return

        # Clean up any previous Gazebo session before starting a fresh one
        self._stop_gazebo()

        # Find conda environment
        conda_prefix, error = self._find_conda_env()
        if conda_prefix is None:
            self._show_conda_not_installed_dialog()
            return
        self._add_conda_to_roboapps()

        # Build subprocess environment (same pattern as _launch_rviz)
        env = os.environ.copy()
        conda_bin = os.path.join(conda_prefix, "bin")
        conda_lib = os.path.join(conda_prefix, "lib")

        env["PATH"] = conda_bin + ":" + env.get("PATH", "")
        env["CONDA_PREFIX"] = conda_prefix
        env["CONDA_DEFAULT_ENV"] = "ros_env"

        # ROS2 environment variables
        env["AMENT_PREFIX_PATH"] = conda_prefix
        env["CMAKE_PREFIX_PATH"] = conda_prefix
        env["COLCON_PREFIX_PATH"] = conda_prefix

        # Auto-detect Python version in conda env
        py_dirs = glob.glob(os.path.join(conda_prefix, "lib", "python3.*", "site-packages"))
        if py_dirs:
            env["PYTHONPATH"] = py_dirs[0] + ":" + env.get("PYTHONPATH", "")

        env["LD_LIBRARY_PATH"] = conda_lib + ":" + env.get("LD_LIBRARY_PATH", "")
        env["DYLD_LIBRARY_PATH"] = conda_lib + ":" + env.get("DYLD_LIBRARY_PATH", "")

        # DDS — simulation is localhost-only, no peer config needed
        env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "0")

        # Gazebo Classic environment variables
        gazebo_share = os.path.join(conda_prefix, "share", "gazebo-11")
        gazebo_models = os.path.join(gazebo_share, "models")
        env["GAZEBO_MODEL_PATH"] = gazebo_models + ":" + env.get("GAZEBO_MODEL_PATH", "")
        env["GAZEBO_RESOURCE_PATH"] = gazebo_share + ":" + env.get("GAZEBO_RESOURCE_PATH", "")
        env["GAZEBO_PLUGIN_PATH"] = conda_lib + ":" + env.get("GAZEBO_PLUGIN_PATH", "")

        # Workarounds for boost mutex race condition on macOS
        env["GAZEBO_IP"] = "127.0.0.1"
        env["GAZEBO_MASTER_URI"] = "http://localhost:11345"
        env["GAZEBO_MODEL_DATABASE_URI"] = ""

        # Build command
        launch_file = os.path.join(_PKG_DIR, "launch", "gazebo_sim.launch.py")
        ros2_exe = os.path.join(conda_bin, "ros2")
        cmd = [ros2_exe, "launch", launch_file]

        self._log("Launching Gazebo simulation...")
        self._log(f"  conda env: {conda_prefix}")
        self._log(f"  launch file: {launch_file}")

        gazebo_log = os.path.join(_PKG_DIR, ".gazebo_launch.log")
        try:
            self._gazebo_log_fh = open(gazebo_log, "w")
            self._gazebo_process = subprocess.Popen(
                cmd,
                env=env,
                stdout=self._gazebo_log_fh,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._log(f"  Gazebo started (PID {self._gazebo_process.pid}).")
            self._log(f"  Log file: {gazebo_log}")

            # Show the Simulation Initiation popup
            self._show_sim_initiation_dialog()

            # Check after 3 seconds if process died immediately
            from PyQt6.QtCore import QTimer
            QTimer.singleShot(3000, self._check_gazebo_status)
        except FileNotFoundError:
            self._log(
                "ERROR: ros2 executable not found in ros_env.\n"
                "  Run: bash setup_robostack.sh"
            )
        except Exception as e:
            self._log(f"ERROR launching Gazebo: {e}")

    def _show_sim_initiation_dialog(self):
        """Create (or raise) the Simulation Initiation popup."""
        script_path = os.path.join(_PKG_DIR, "run_sim.sh")
        if not os.path.isfile(script_path):
            self._log("WARNING: run_sim.sh not found in project folder.")
            return
        if self._sim_dialog is None or not self._sim_dialog.isVisible():
            self._sim_dialog = SimInitiationDialog(script_path, parent=self)
        self._sim_dialog.show()
        self._sim_dialog.raise_()
        self._sim_dialog.activateWindow()

    def _check_gazebo_status(self):
        """Check if Gazebo process died shortly after launch and show errors."""
        if self._gazebo_process is not None and self._gazebo_process.poll() is not None:
            exit_code = self._gazebo_process.returncode
            self._log(f"WARNING: Gazebo exited with code {exit_code}.")
            log_path = os.path.join(_PKG_DIR, ".gazebo_launch.log")
            if os.path.isfile(log_path):
                with open(log_path, "r") as f:
                    tail = f.read()[-1500:]
                if tail.strip():
                    self._log("--- Gazebo log (last lines) ---")
                    for line in tail.strip().splitlines()[-20:]:
                        self._log(f"  {line}")

    def closeEvent(self, event):
        """Clean up RViz2, Gazebo subprocesses, and SSH connection on window close."""
        self._save_canvas_state()
        if self._rviz_process is not None and self._rviz_process.poll() is None:
            self._rviz_process.terminate()
            try:
                self._rviz_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._rviz_process.kill()

        self._stop_gazebo()

        if self._robosim_process is not None and self._robosim_process.poll() is None:
            self._robosim_process.terminate()
            try:
                self._robosim_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._robosim_process.kill()

        if self.ssh_client:
            try:
                self.ssh_client.close()
            except Exception:
                pass

        super().closeEvent(event)

    # ------------------------------------------------------------------
    # Git / GitHub integration
    # ------------------------------------------------------------------

    def _load_git_creds(self):
        """Load saved git credentials from disk."""
        try:
            with open(_GIT_CREDS_FILE, "r") as fh:
                return json.load(fh)
        except Exception:
            return {}

    def _save_git_creds(self, creds: dict):
        """Persist git credentials (username, token, repo_name) to disk."""
        try:
            existing = self._load_git_creds()
            existing.update(creds)
            with open(_GIT_CREDS_FILE, "w") as fh:
                json.dump(existing, fh, indent=2)
        except Exception:
            pass

    def _show_git_menu(self):
        """Show the Git action dropdown beneath the GitHub button."""
        menu = QMenu(self)
        menu.setStyleSheet(
            "QMenu { background: white; border: 1px solid #ddd; border-radius: 8px; "
            "padding: 4px 0; font-size: 13px; }"
            "QMenu::item { padding: 8px 24px; color: #1a1a1a; }"
            "QMenu::item:selected { background: #F0F0F0; color: #1a1a1a; border-radius: 4px; }"
            "QMenu::separator { height: 1px; background: #eee; margin: 4px 0; }"
        )
        init_action = menu.addAction("  Create GitHub Repo")
        push_action = menu.addAction("  Commit & Push")
        pull_action = menu.addAction("  Pull from GitHub")
        menu.addSeparator()
        menu.addAction("  Cancel")

        btn_rect  = self.git_btn.rect()
        btn_pos   = self.git_btn.mapToGlobal(btn_rect.bottomLeft())
        chosen    = menu.exec(btn_pos)

        if chosen == init_action:
            self._git_init()
        elif chosen == push_action:
            self._git_push()
        elif chosen == pull_action:
            self._git_pull()

    # --- git init + GitHub repo creation ---

    def _git_init(self):
        creds  = self._load_git_creds()
        dialog = GitInitDialog(creds, parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        data = dialog.result_creds()

        if data.get("save"):
            self._save_git_creds({
                "username":    data["username"],
                "token":       data["token"],
                "repo_name":   data["repo_name"],
                "description": data["description"],
                "save":        True,
            })

        errors = []

        # 1. git init (idempotent)
        r = subprocess.run(["git", "init"], cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode != 0:
            errors.append(f"git init failed:\n{r.stderr.strip()}")

        # 2. git config author (use GitHub username)
        subprocess.run(["git", "config", "user.name",  data["username"]], cwd=_PKG_DIR)
        subprocess.run(["git", "config", "user.email", f"{data['username']}@users.noreply.github.com"],
                       cwd=_PKG_DIR)

        # 3. Ensure .gitignore hides credential files
        gitignore = os.path.join(_PKG_DIR, ".gitignore")
        hidden = {".git_credentials.json", ".robot_profiles.json", ".node_canvas.json",
                  "__pycache__/", "*.pyc"}
        try:
            existing_lines = open(gitignore).read().splitlines() if os.path.exists(gitignore) else []
            with open(gitignore, "a") as fh:
                for entry in hidden:
                    if entry not in existing_lines:
                        fh.write(entry + "\n")
        except Exception:
            pass

        # 4. Create README.md if requested
        if data["readme"]:
            readme_path = os.path.join(_PKG_DIR, "README.md")
            if not os.path.exists(readme_path):
                try:
                    with open(readme_path, "w") as fh:
                        fh.write(f"# {data['repo_name']}\n\n{data['description']}\n")
                except Exception:
                    pass

        # 5. Create GitHub repo via API
        try:
            payload = json.dumps({
                "name":        data["repo_name"],
                "description": data["description"],
                "private":     data["private"],
                "auto_init":   False,
            }).encode()
            req = urllib.request.Request(
                "https://api.github.com/user/repos",
                data=payload,
                headers={
                    "Authorization": f"token {data['token']}",
                    "Content-Type":  "application/json",
                    "Accept":        "application/vnd.github+json",
                    "User-Agent":    "TestDrive-App",
                },
                method="POST",
            )
            with urllib.request.urlopen(req) as resp:
                repo_info = json.loads(resp.read())
            clone_url = repo_info.get("clone_url", "")
        except urllib.error.HTTPError as e:
            body = e.read().decode(errors="replace")
            errors.append(f"GitHub API error {e.code}:\n{body[:300]}")
            clone_url = ""
        except Exception as e:
            errors.append(f"GitHub API error: {e}")
            clone_url = ""

        # 6. Set remote origin (embed token for auth)
        if clone_url:
            auth_url = clone_url.replace(
                "https://", f"https://{data['username']}:{data['token']}@")
            subprocess.run(["git", "remote", "remove", "origin"],
                           cwd=_PKG_DIR, capture_output=True)
            subprocess.run(["git", "remote", "add", "origin", auth_url],
                           cwd=_PKG_DIR, capture_output=True)

        # 7. Initial commit + push
        subprocess.run(["git", "add", "."], cwd=_PKG_DIR, capture_output=True)
        r = subprocess.run(["git", "commit", "-m", "Initial commit — TestDrive"],
                           cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode != 0 and "nothing to commit" not in r.stdout:
            errors.append(f"git commit failed:\n{r.stderr.strip()}")

        if clone_url:
            r = subprocess.run(
                ["git", "push", "-u", "origin", "HEAD"],
                cwd=_PKG_DIR, capture_output=True, text=True)
            if r.returncode != 0:
                errors.append(f"git push failed:\n{r.stderr.strip()}")

        if errors:
            QMessageBox.warning(self, "Git — Issues Encountered",
                                "\n\n".join(errors))
        else:
            repo_url = f"https://github.com/{data['username']}/{data['repo_name']}"
            QMessageBox.information(
                self, "Git — Repository Created",
                f"Repository created and initial push complete.\n\n{repo_url}")

    # --- git commit + push ---

    def _git_push(self):
        creds  = self._load_git_creds()
        dialog = GitPushDialog(creds, parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        data = dialog.result_data()

        if data.get("save"):
            # Extract username/repo_name from URL for future pre-fill
            m = re.match(r'https://github\.com/([^/]+)/([^/]+?)(?:\.git)?$',
                         data["repo_url"])
            update = {"token": data["token"], "branch": data["branch"]}
            if m:
                update["username"] = m.group(1)
                update["repo_name"] = m.group(2)
            self._save_git_creds(update)

        errors = []
        branch = data.get("branch", "main")

        self._log("--- Git Commit & Push ---")
        self._log(f"Repository: {data['repo_url']}")
        self._log(f"Branch: {branch}")
        self._log(f"Commit message: {data['message']}")

        # Ensure remote is set with auth token
        auth_url = re.sub(r'^https://',
                          f'https://{creds.get("username", "")}:{data["token"]}@',
                          data["repo_url"])
        # If username not in saved creds, try extracting from URL
        m = re.match(r'https://github\.com/([^/]+)', data["repo_url"])
        if m:
            auth_url = data["repo_url"].replace(
                "https://", f"https://{m.group(1)}:{data['token']}@")

        subprocess.run(["git", "remote", "remove", "origin"],
                       cwd=_PKG_DIR, capture_output=True)
        subprocess.run(["git", "remote", "add", "origin", auth_url],
                       cwd=_PKG_DIR, capture_output=True)

        self._log("Running: git add .")
        subprocess.run(["git", "add", "."], cwd=_PKG_DIR, capture_output=True)

        self._log(f"Running: git commit -m \"{data['message']}\"")
        r = subprocess.run(
            ["git", "commit", "-m", data["message"]],
            cwd=_PKG_DIR, capture_output=True, text=True)
        if r.stdout.strip():
            self._log(r.stdout.strip())
        if r.stderr.strip():
            self._log(r.stderr.strip())
        if r.returncode != 0 and "nothing to commit" not in r.stdout and "nothing to commit" not in r.stderr:
            errors.append(f"git commit: {r.stderr.strip() or r.stdout.strip()}")
            self._log(f"ERROR: git commit failed (exit code {r.returncode})")
        else:
            self._log("git commit: OK")

        self._log(f"Running: git pull --rebase origin {branch}")
        r = subprocess.run(
            ["git", "pull", "--rebase", "origin", branch],
            cwd=_PKG_DIR, capture_output=True, text=True)
        if r.stdout.strip():
            self._log(r.stdout.strip())
        if r.stderr.strip():
            self._log(r.stderr.strip())
        if r.returncode != 0:
            errors.append(f"git pull --rebase failed:\n{r.stderr.strip() or r.stdout.strip()}")
            self._log(f"ERROR: git pull --rebase failed (exit code {r.returncode})")
        else:
            self._log("git pull --rebase: OK")

        self._log(f"Running: git push -u origin HEAD:{branch}")
        r = subprocess.run(
            ["git", "push", "-u", "origin", f"HEAD:{branch}"],
            cwd=_PKG_DIR, capture_output=True, text=True)
        if r.stdout.strip():
            self._log(r.stdout.strip())
        if r.stderr.strip():
            self._log(r.stderr.strip())
        if r.returncode != 0:
            errors.append(f"git push failed:\n{r.stderr.strip()}")
            self._log(f"ERROR: git push failed (exit code {r.returncode})")
        else:
            self._log("git push: OK")

        if errors:
            self._log("--- Push finished with errors ---")
            QMessageBox.warning(self, "Git — Push Issues", "\n\n".join(errors))
        else:
            self._log("--- Push complete ---")
            QMessageBox.information(self, "Git — Push Complete",
                                    f"Successful push to:\n{data['repo_url']}  [{branch}]")

    # --- git pull ---

    def _git_pull(self):
        creds = self._load_git_creds()
        token = creds.get("token", "")

        # Check we're inside a git repo
        r = subprocess.run(["git", "rev-parse", "--is-inside-work-tree"],
                           cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode != 0:
            QMessageBox.warning(self, "Git — Not Initialised",
                                "This project is not a git repository yet.\n"
                                "Use 'Initialize & Create GitHub Repo' first.")
            return

        # Show branch selection dialog
        dlg = GitPullDialog(creds, self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        data = dlg.result_data()
        branch = data["branch"]

        # Inject token into remote URL if we have one
        if token:
            r2 = subprocess.run(["git", "remote", "get-url", "origin"],
                                 cwd=_PKG_DIR, capture_output=True, text=True)
            remote_url = r2.stdout.strip()
            if remote_url and "github.com" in remote_url:
                m = re.match(r'https://github\.com/([^/]+)', remote_url)
                if m:
                    auth_url = remote_url.replace(
                        "https://", f"https://{m.group(1)}:{token}@")
                    subprocess.run(["git", "remote", "set-url", "origin", auth_url],
                                   cwd=_PKG_DIR, capture_output=True)

        r = subprocess.run(["git", "pull", "--rebase", "origin", branch],
                           cwd=_PKG_DIR, capture_output=True, text=True)
        if r.returncode != 0:
            QMessageBox.warning(self, "Git — Pull Failed", r.stderr.strip())
        else:
            remote_url = subprocess.run(
                ["git", "remote", "get-url", "origin"],
                cwd=_PKG_DIR, capture_output=True, text=True).stdout.strip()
            clean_url = re.sub(r'https://[^@]+@', 'https://', remote_url)
            QMessageBox.information(self, "Git — Pull Complete",
                                    f"Successful pull from:\n{clean_url}  [{branch}]")


def main():
    app = QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
