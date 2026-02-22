import sys
import traceback
import io
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout,
                             QVBoxLayout, QPushButton, QFrame, QLabel, QFileDialog,
                             QProgressBar, QPlainTextEdit, QSplitter, QCheckBox, QMessageBox)
from PyQt6.QtCore import Qt, QTimer, QRect, QSize
from PyQt6.QtGui import QPainter, QColor, QFont, QSyntaxHighlighter, QTextCharFormat, QFontDatabase
from PyQt6.QtOpenGLWidgets import QOpenGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *

# --- GLOBAL SIMULATION CONSTANTS ---
LIDAR_RANGE = 15.0
LIDAR_RES = 2
ROBOT_RADIUS = 0.25
MAX_LIN_SPEED = 0.12
MAX_ANG_SPEED = 5.0

# Visual configurations
ROBOT_COLORS = [(0, 1, 1), (1, 0, 1), (1, 0.5, 0), (0, 1, 0), (0.6, 0.2, 1)]
OBJ_COLORS = {
    "Red": (1, 0, 0), "Yellow": (1, 1, 0), "Blue": (0, 0, 1), 
    "White": (1, 1, 1), "Green": (0, 1, 0), "Black": (0.1, 0.1, 0.1), "Orange": (1, 0.5, 0)
}

# Font priority list to prevent expensive "alias population" warnings in Qt
PREFERRED_FONTS = ["Consolas", "Monospace", "Menlo", "Courier New"]

def get_monospaced_font(size=13):
    """
    Returns the first available monospaced font from a priority list.
    This prevents Qt from scanning the entire system for aliases.
    """
    font = QFont()
    available_families = QFontDatabase.families()
    for family in PREFERRED_FONTS:
        if family in available_families:
            font.setFamily(family)
            break
    font.setFixedPitch(True)
    font.setPointSize(size)
    return font

class PythonHighlighter(QSyntaxHighlighter):
    """
    A simple syntax highlighter for the Python robot scripts.
    Colors keywords, numbers, and the custom Robot API functions.
    """
    def __init__(self, parent):
        super().__init__(parent)
        self.rules = []
        
        # Keyword format (blue)
        keyword_fmt = QTextCharFormat()
        keyword_fmt.setForeground(QColor("#569cd6"))
        keywords = [
            "self", "def", "if", "else", "while", "for", "import", "return", "np", "print",
            "drive", "holding", "close_enough", "attach", "detach", "distance", "look", 
            "transform_to_map", "battery_level"
        ]
        for w in keywords:
            self.rules.append((f"\\b{w}\\b", keyword_fmt))
            
        # Numeric format (light green)
        num_fmt = QTextCharFormat()
        num_fmt.setForeground(QColor("#b5cea8"))
        self.rules.append(("\\b[0-9]+\\.?[0-9]*\\b", num_fmt))

    def highlightBlock(self, text):
        import re
        for pattern, fmt in self.rules:
            for match in re.finditer(pattern, text):
                self.setFormat(match.start(), match.end() - match.start(), fmt)

class QCodeEditor(QPlainTextEdit):
    """
    A custom text editor widget featuring line numbers and high-contrast styling.
    """
    class LineNumberArea(QWidget):
        def __init__(self, editor):
            super().__init__(editor)
            self.editor = editor
            
        def sizeHint(self):
            return QSize(self.editor.line_number_area_width(), 0)
            
        def paintEvent(self, event):
            self.editor.lineNumberAreaPaintEvent(event)

    def __init__(self):
        super().__init__()
        self.line_number_area = self.LineNumberArea(self)
        self.blockCountChanged.connect(self.update_line_number_area_width)
        self.updateRequest.connect(self.update_line_number_area)
        self.update_line_number_area_width(0)
        
        self.setFont(get_monospaced_font(13))
        self.setStyleSheet("background:#1e1e1e; color:#d4d4d4; border:none;")

    def line_number_area_width(self):
        """Calculates width required for line numbers based on total count."""
        digits = 1
        max_val = max(1, self.blockCount())
        while max_val >= 10:
            max_val //= 10
            digits += 1
        return 15 + self.fontMetrics().horizontalAdvance('9') * digits

    def update_line_number_area_width(self, _):
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def update_line_number_area(self, rect, dy):
        if dy:
            self.line_number_area.scroll(0, dy)
        else:
            self.line_number_area.update(0, rect.y(), self.line_number_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self.update_line_number_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self.line_number_area.setGeometry(QRect(cr.left(), cr.top(), self.line_number_area_width(), cr.height()))

    def lineNumberAreaPaintEvent(self, event):
        """Paints the line number gutter."""
        painter = QPainter(self.line_number_area)
        painter.fillRect(event.rect(), QColor("#2b2b2b"))
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                painter.setPen(QColor("#858585"))
                painter.drawText(0, top, self.line_number_area.width() - 5, self.fontMetrics().height(), 
                                 Qt.AlignmentFlag.AlignRight, str(block_number + 1))
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            block_number += 1

class RobotController(QWidget):
    """
    A pop-up editor window for each robot. Allows writing scripts,
    viewing console output, and toggling simulation state.
    """
    def __init__(self, robot, parent_sim):
        super().__init__()
        self.robot = robot
        self.sim = parent_sim
        self.setWindowTitle(f"Editor: Robot_{robot.id}")
        self.resize(600, 750)
        self.is_dirty = False
        
        main_layout = QVBoxLayout(self)
        splitter = QSplitter(Qt.Orientation.Vertical)
        
        # Code Editor
        self.editor = QCodeEditor()
        self.highlighter = PythonHighlighter(self.editor.document())
        self.editor.setPlainText(self.robot.script)
        self.editor.textChanged.connect(self.on_text_changed)
        splitter.addWidget(self.editor)
        
        # Output Console
        self.console = QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setFont(get_monospaced_font(12))
        self.console.setStyleSheet("background:#000; color:#0f0;")
        splitter.addWidget(self.console)
        
        splitter.setSizes([500, 200])
        main_layout.addWidget(splitter)
        
        # Execution Options
        opt_layout = QHBoxLayout()
        self.loop_check = QCheckBox("Run forever")
        self.loop_check.setStyleSheet("color: white;")
        self.loop_check.setChecked(self.robot.run_forever)
        self.loop_check.stateChanged.connect(self.sync_loop_state)
        opt_layout.addWidget(self.loop_check)
        opt_layout.addStretch()
        main_layout.addLayout(opt_layout)
        
        # Controls
        btn_layout = QHBoxLayout()
        self.toggle_btn = QPushButton("Run")
        self.toggle_btn.clicked.connect(self.handle_toggle)
        btn_layout.addWidget(self.toggle_btn)
        
        for text, func in [("Open", self.file_open), ("Save", self.file_save), ("Clear", self.console.clear)]:
            btn = QPushButton(text)
            btn.clicked.connect(func)
            btn_layout.addWidget(btn)
        main_layout.addLayout(btn_layout)
        
        self.update_ui_state()

    def on_text_changed(self):
        """Marks script as unsaved."""
        self.is_dirty = True

    def sync_loop_state(self, state):
        """Toggles whether script repeats or runs once."""
        self.robot.run_forever = (state == 2)

    def handle_toggle(self):
        """Starts or stops the robot script."""
        if self.robot.is_running:
            self.robot.is_running = False
        else:
            self.robot.script = self.editor.toPlainText()
            self.robot.is_running = True
            self.is_dirty = False
        self.update_ui_state()

    def update_ui_state(self):
        """Visual feedback for the run/stop button."""
        if self.robot.is_running:
            self.toggle_btn.setText("Stop")
            self.toggle_btn.setStyleSheet("background-color: #800; color: white; font-weight: bold;")
        else:
            self.toggle_btn.setText("Run")
            self.toggle_btn.setStyleSheet("background-color: #080; color: white; font-weight: bold;")

    def file_open(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Script", "", "Python (*.py)", options=QFileDialog.Option.DontUseNativeDialog)
        if path:
            self.editor.setPlainText(open(path).read())
            self.is_dirty = False

    def file_save(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Script", "", "Python (*.py)", options=QFileDialog.Option.DontUseNativeDialog)
        if path:
            with open(path, 'w') as f:
                f.write(self.editor.toPlainText())
            self.is_dirty = False
            return True
        return False

    def closeEvent(self, event):
        """Confirmation dialog when closing with unsaved changes."""
        if self.is_dirty:
            reply = QMessageBox.question(self, 'Unsaved Changes', "Save changes to script before closing?", 
                                         QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel)
            if reply == QMessageBox.StandardButton.Save:
                if self.file_save(): event.accept()
                else: event.ignore()
            elif reply == QMessageBox.StandardButton.Discard:
                event.accept()
            else:
                event.ignore()
        else:
            event.accept()

class Robot:
    """
    Data structure for a robot agent.
    Contains position, orientation, battery, and current script state.
    """
    def __init__(self, id_num, spawn_pos=None):
        self.id = id_num
        self.color = ROBOT_COLORS[id_num % len(ROBOT_COLORS)]
        self.pos = np.array(spawn_pos) if spawn_pos is not None else np.array([2.0, 2.0])
        self.yaw = 0.0
        self.energy = 100.0
        self.attached_obj = None
        self.lidar_data = np.zeros(360)
        self.script = "print(f'Battery: {battery_level()}%')\ndrive(0.5, 0.0)"
        self.is_running = False
        self.run_forever = False

class WorldObject:
    """
    Represents objects like boxes, spheres, and chargers.
    Handles distance math for collisions and LiDAR.
    """
    def __init__(self, name, o_type, color_name, physics, size, pos, orientation):
        self.name = name
        self.o_type = o_type.lower()
        self.base_color = OBJ_COLORS.get(color_name, (0.5, 0.5, 0.5))
        self.color_name = color_name
        self.moveable = (physics.lower() == "moveable")
        self.w, self.h, self.d = size
        self.x, self.y, self.z = pos
        self.yaw = orientation[2]
        self.is_held = False # Prevents multiple robots from grabbing the same object
        self.is_charging_active = False # Visual indicator for chargers
    
    def get_dist_to_point(self, px, py):
        """Returns shortest distance from a point (px, py) to the object's surface."""
        dx, dy = px - self.x, py - self.y
        if self.o_type == "box":
            rad = np.radians(-self.yaw)
            lx = dx * np.cos(rad) - dy * np.sin(rad)
            ly = dx * np.sin(rad) + dy * np.cos(rad)
            return np.sqrt(max(abs(lx) - self.w/2, 0)**2 + max(abs(ly) - self.d/2, 0)**2)
        return max(0, np.sqrt(dx**2 + dy**2) - self.w/2)

class MainWindow(QMainWindow):
    """
    The Core Engine. Manages the main 3D view, simulation loop, 
    collisions, and the Robot Scripting API.
    """
    def __init__(self, startup_file=None):
        super().__init__()
        self.setWindowTitle("RoboSim")
        self.resize(1400, 950)
        
        self.world = {'robots': [Robot(0)], 'objects': []}
        self.controllers = {}
        self.active_idx = 0
        self.keys = set()
        
        # Maps and rendering tools
        self.slam_map = np.zeros((500, 500))
        self.quad = gluNewQuadric()
        
        # Camera states
        self.rx, self.ry, self.dist = 45, 45, 15
        self.last_m = None
        
        # Build UI
        self.setup_ui()
        
        # Start sim loop (approx 60 FPS)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sim)
        self.timer.start(16)
        
        if startup_file:
            self.load_world(startup_file)

    def setup_ui(self):
        """Initializes the layout and specialized OpenGL widgets."""
        c = QWidget()
        self.setCentralWidget(c)
        layout = QHBoxLayout(c)
        
        # Main 3D View (wrapped in vertical layout with instruction label)
        main_area = QWidget()
        main_vl = QVBoxLayout(main_area)
        main_vl.setContentsMargins(0, 0, 0, 0)
        main_vl.setSpacing(0)

        instruction = QLabel("To move the robot, press A W D S keys on your keyboard.")
        instruction.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        instruction.setStyleSheet("color: white; background: #222; padding: 2px; font-size: 18px; font-weight: bold;")
        main_vl.addWidget(instruction, 5)

        self.main_view = QOpenGLWidget()
        self.main_view.initializeGL = self.init_gl
        self.main_view.paintGL = self.paint_main
        self.main_view.mousePressEvent = self.view_press
        self.main_view.mouseMoveEvent = self.view_move
        self.main_view.wheelEvent = self.view_zoom
        main_vl.addWidget(self.main_view, 90)

        layout.addWidget(main_area, 3)
        
        # Sidebar
        side = QFrame()
        side.setFixedWidth(280)
        side.setStyleSheet("background:#111; color:white;")
        sl = QVBoxLayout(side)
        
        self.status = QLabel("ACTIVE: ROBOT_0")
        sl.addWidget(self.status)
        
        self.bat_label = QLabel("Battery: 100%")
        sl.addWidget(self.bat_label)
        
        self.side_bat = QProgressBar()
        self.side_bat.setFixedHeight(15)
        sl.addWidget(self.side_bat)
        
        # Secondary Views (POV, LiDAR, SLAM)
        self.pov = QOpenGLWidget()
        self.pov.setFixedHeight(180)
        self.pov.initializeGL = self.init_gl
        self.pov.paintGL = self.paint_pov
        sl.addWidget(self.pov)
        
        self.lidar_ui = QWidget()
        self.lidar_ui.setFixedHeight(250)
        self.lidar_ui.paintEvent = self.paint_lidar
        sl.addWidget(self.lidar_ui)
        
        self.slam_ui = QOpenGLWidget()
        self.slam_ui.setFixedHeight(180)
        self.slam_ui.paintGL = self.paint_slam
        sl.addWidget(self.slam_ui)
        
        # Buttons
        for text, func in [("NEW ROBOT", self.spawn_robot), ("OPEN EDITOR", self.open_editor), 
                           ("LOAD WORLD", self.load_world_dialog), ("ATTACH", self.manual_attach), 
                           ("DETACH", self.manual_detach)]:
            btn = QPushButton(text)
            btn.clicked.connect(func)
            sl.addWidget(btn)
            
        sl.addStretch()
        layout.addWidget(side)

    # --- RENDER ENGINE ---

    def init_gl(self):
        """One-time OpenGL setup for widgets."""
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glClearColor(0.1, 0.1, 0.1, 1.0)

    def setup_viewport(self, widget):
        """Adjusts GL viewport to handle high-DPI scaling."""
        ratio = widget.devicePixelRatio()
        glViewport(0, 0, int(widget.width() * ratio), int(widget.height() * ratio))
        return widget.width() * ratio, widget.height() * ratio

    def draw_box(self, w, d, h):
        """Draws a box of given half-extents."""
        glBegin(GL_QUADS)
        # Normal faces
        faces = [
            ((0,0,1), [(-w,-d,h),(w,-d,h),(w,d,h),(-w,d,h)]),
            ((0,0,-1), [(-w,d,-h),(w,d,-h),(w,-d,-h),(-w,-d,-h)]),
            ((0,1,0), [(-w,d,h),(w,d,h),(w,d,-h),(-w,d,-h)]),
            ((0,-1,0), [(-w,-d,-h),(w,-d,-h),(w,-d,h),(-w,-d,h)]),
            ((1,0,0), [(w,-d,h),(w,-d,-h),(w,d,-h),(w,d,h)]),
            ((-1,0,0), [(-w,-d,-h),(-w,-d,h),(w,d,h),(-w,d,-h)])
        ]
        for n, v in faces:
            glNormal3f(*n)
            for p in v:
                glVertex3f(*p)
        glEnd()

    def draw_closed_cyl(self, r, h):
        """Draws a cylinder with caps (for robots)."""
        gluCylinder(self.quad, r, r, h, 32, 1)
        glPushMatrix()
        glTranslatef(0, 0, h)
        gluDisk(self.quad, 0, r, 32, 1)
        glPopMatrix() 
        glPushMatrix()
        glRotatef(180, 1, 0, 0)
        gluDisk(self.quad, 0, r, 32, 1)
        glPopMatrix()

    def render_scene(self):
        """Draws the entire 3D world."""
        # Draw Grid
        glDisable(GL_LIGHTING)
        glColor3f(0.2, 0.2, 0.2)
        glBegin(GL_LINES)
        for i in range(11):
            glVertex3f(i,0,0)
            glVertex3f(i,10,0)
            glVertex3f(0,i,0)
            glVertex3f(10,i,0)
        glEnd()
        
        # Walls
        glEnable(GL_LIGHTING)
        glColor4f(0.4, 0.4, 0.4, 1.0)
        for p in [(5,0), (5,10)]:
            glPushMatrix()
            glTranslatef(p[0], p[1], 0.25)
            self.draw_box(5, 0.05, 0.25)
            glPopMatrix()
        for p in [(0,5), (10,5)]:
            glPushMatrix()
            glTranslatef(p[0], p[1], 0.25)
            self.draw_box(0.05, 5, 0.25)
            glPopMatrix()
            
        # Draw Objects
        for o in self.world['objects']:
            glPushMatrix()
            z = o.w/2 if o.o_type == "sphere" else o.h/2
            glTranslatef(o.x, o.y, z)
            glRotatef(o.yaw, 0, 0, 1)
            
            c = [1,0,0] if "charger" in o.name.lower() and o.is_charging_active else o.base_color
            # Ghost effect if object is being held
            alpha = 0.4 if o.is_held else 1.0
            glColor4f(c[0], c[1], c[2], alpha)
            
            if o.o_type == "box":
                self.draw_box(o.w/2, o.d/2, o.h/2)
            elif o.o_type == "sphere":
                gluSphere(self.quad, o.w/2, 32, 32)
            elif o.o_type == "cylinder":
                glTranslatef(0, 0, -o.h/2)
                self.draw_closed_cyl(o.w/2, o.h)
            glPopMatrix()
            
        # Draw Robots
        for r in self.world['robots']:
            glPushMatrix()
            glTranslatef(r.pos[0], r.pos[1], 0)
            glRotatef(-r.yaw, 0, 0, 1)
            
            # Desaturate color if not active
            rc = r.color if r.id == self.active_idx else (0.3, 0.3, 0.3)
            glColor4f(rc[0], rc[1], rc[2], 1.0)
            self.draw_closed_cyl(ROBOT_RADIUS, 0.5)
            
            # Simple sensor visor
            glTranslatef(0, 0.2, 0.4)
            glColor3f(0, 0, 0)
            self.draw_box(0.1, 0.05, 0.05)
            glPopMatrix()

    # --- SIMULATION LOGIC ---

    def update_sim(self):
        """Main loop handling physics, API execution, and sensor updates."""
        for o in self.world['objects']:
            o.is_charging_active = False
            
        for r in self.world['robots']:
            # Autonomous Script Execution
            if r.is_running and r.energy > 0:
                r.energy -= 0.005
                buf = io.StringIO()
                try:
                    # Exposed API to Robot Editor
                    api = {
                        "drive": lambda l, a: self.api_drive(r, l, a),
                        "holding": lambda: r.attached_obj is not None,
                        "close_enough": lambda: any(o.moveable and not o.is_held and o.get_dist_to_point(r.pos[0], r.pos[1]) < 0.65 for o in self.world['objects']),
                        "attach": lambda: self.manual_attach(r),
                        "detach": lambda: self.manual_detach(r),
                        "distance": lambda a: float(r.lidar_data[int(a % 360)]),
                        "look": lambda: self.api_look(r),
                        "battery_level": lambda: int(r.energy),
                        "transform_to_map": lambda d, a: [float(r.pos[0] + d*np.sin(np.radians(r.yaw+a))), float(r.pos[1] + d*np.cos(np.radians(r.yaw+a)))],
                        "self": r, "np": np, "print": lambda *args: buf.write(" ".join(map(str, args)) + "\n")
                    }
                    exec(r.script, api)
                    
                    # Update Editor console
                    if buf.getvalue() and r.id in self.controllers:
                        self.controllers[r.id].console.appendPlainText(buf.getvalue().strip())
                    
                    if not r.run_forever:
                        r.is_running = False
                        if r.id in self.controllers:
                            self.controllers[r.id].update_ui_state()
                            
                except Exception as e:
                    if r.id in self.controllers:
                        self.controllers[r.id].console.appendPlainText(f"Error: {e}")
                    r.is_running = False
                    if r.id in self.controllers:
                        self.controllers[r.id].update_ui_state()
            
            # Manual Control for Active Robot
            if r.id == self.active_idx:
                if r.energy > 0:
                    if Qt.Key.Key_W in self.keys: self.api_drive(r, 0.7, 0)
                    if Qt.Key.Key_S in self.keys: self.api_drive(r, -0.5, 0)
                    if Qt.Key.Key_A in self.keys: self.api_drive(r, 0, -0.6)
                    if Qt.Key.Key_D in self.keys: self.api_drive(r, 0, 0.6)
                self.side_bat.setValue(int(r.energy))
                self.bat_label.setText(f"Battery: {int(r.energy)}%")
                
            # Charging logic
            for o in self.world['objects']:
                if "charger" in o.name.lower() and o.get_dist_to_point(r.pos[0], r.pos[1]) < 0.6:
                    r.energy = min(100, r.energy + 0.3)
                    o.is_charging_active = True
                    
            # Update attached object position
            if r.attached_obj: 
                r.attached_obj.x = r.pos[0] + 0.55 * np.sin(np.radians(r.yaw))
                r.attached_obj.y = r.pos[1] + 0.55 * np.cos(np.radians(r.yaw))
                r.attached_obj.yaw = -r.yaw
                
            # Sensor: LiDAR & Mapping
            for deg in range(0, 360, LIDAR_RES):
                rad = np.radians(r.yaw + deg)
                hit = LIDAR_RANGE
                for dist in np.arange(0.3, LIDAR_RANGE, 0.2):
                    tx, ty = r.pos[0] + dist*np.sin(rad), r.pos[1] + dist*np.cos(rad)
                    # Wall or object collision
                    if tx<0 or tx>10 or ty<0 or ty>10 or any(o.get_dist_to_point(tx, ty)<0.15 for o in self.world['objects'] if not o.is_held):
                        hit = dist
                        # Project onto SLAM map
                        self.slam_map[int(ty*50)%500, int(tx*50)%500] = 1
                        break
                r.lidar_data[deg:deg+LIDAR_RES] = hit
                
        # Trigger redraws
        self.main_view.update()
        self.pov.update()
        self.lidar_ui.update()
        self.slam_ui.update()

    def api_drive(self, robot, lin, ang):
        """Calculates kinematics and collision-checks before moving."""
        ny = robot.yaw + (ang * MAX_ANG_SPEED)
        step = lin * MAX_LIN_SPEED
        new_pos = robot.pos + [step * np.sin(np.radians(ny)), step * np.cos(np.radians(ny))]
        
        if not self.check_collision(robot, new_pos, ny):
            robot.pos, robot.yaw = new_pos, ny
            if abs(lin) > 0 or abs(ang) > 0:
                robot.energy -= 0.03

    def api_look(self, robot):
        """Returns a list of visible objects within FOV."""
        fov = []
        for o in self.world['objects']:
            if o.is_held: continue
            dx, dy = o.x - robot.pos[0], o.y - robot.pos[1]
            dist = np.sqrt(dx**2+dy**2)
            bearing = (np.degrees(np.arctan2(dx, dy)) - robot.yaw + 180) % 360 - 180
            
            if abs(bearing) < 45 and dist < LIDAR_RANGE:
                fov.append([
                    str(o.o_type), 
                    str(o.color_name), 
                    round(float(dist), 2), 
                    round(float(bearing), 2),
                    round(float((o.h/dist)*500), 1), # Visual height
                    round(float((o.w/dist)*500), 1)  # Visual width
                ])
        return fov

    def check_collision(self, r, np_, ny_):
        """Physics engine logic: Checks walls, other robots, and held objects."""
        # 1. World Bounds
        if not (0.25 < np_[0] < 9.75 and 0.25 < np_[1] < 9.75):
            return True
            
        # 2. Objects (Static or Unheld)
        if any(o.get_dist_to_point(np_[0], np_[1]) < 0.35 for o in self.world['objects'] if not o.is_held):
            return True
            
        # 3. Other Robots
        if any(np.linalg.norm(np_ - or_.pos) < 0.5 for or_ in self.world['robots'] if or_ != r):
            return True
            
        # 4. Objects held by other robots
        for other in self.world['robots']:
            if other != r and other.attached_obj:
                if other.attached_obj.get_dist_to_point(np_[0], np_[1]) < 0.25:
                    return True
                    
        # 5. Self-Held Object Collision (Extends robot footprint)
        if r.attached_obj:
            cx = np_[0] + 0.55 * np.sin(np.radians(ny_))
            cy = np_[1] + 0.55 * np.cos(np.radians(ny_))
            cr = r.attached_obj.w / 2
            if not (cr < cx < 10-cr and cr < cy < 10-cr): return True
            if any(o.get_dist_to_point(cx, cy) < (cr+0.05) for o in self.world['objects'] if not o.is_held): return True
            if any(np.linalg.norm(np.array([cx, cy]) - or_.pos) < (cr + 0.25) for or_ in self.world['robots'] if or_ != r): return True
            
        return False

    # --- WIDGET PAINTING ---

    def paint_main(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        w, h = self.setup_viewport(self.main_view)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Orbital camera math
        cx = 5 + self.dist * np.cos(np.radians(self.rx)) * np.cos(np.radians(self.ry))
        cy = 5 + self.dist * np.cos(np.radians(self.rx)) * np.sin(np.radians(self.ry))
        gluLookAt(cx, cy, self.dist * np.sin(np.radians(self.rx)), 5, 5, 0, 0, 0, 1)
        self.render_scene()

    def paint_pov(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        w, h = self.setup_viewport(self.pov)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(75, w/h, 0.1, 30.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        r = self.world['robots'][self.active_idx]
        cx, cy = r.pos[0] + 0.3*np.sin(np.radians(r.yaw)), r.pos[1] + 0.3*np.cos(np.radians(r.yaw))
        gluLookAt(cx, cy, 0.5, cx + np.sin(np.radians(r.yaw)), cy + np.cos(np.radians(r.yaw)), 0.45, 0, 0, 1)
        self.render_scene()

    def paint_lidar(self, event):
        p = QPainter(self.lidar_ui)
        p.fillRect(self.lidar_ui.rect(), QColor(0, 15, 0))
        r = self.world['robots'][self.active_idx]
        cx, cy = self.lidar_ui.width() // 2, self.lidar_ui.height() // 2
        
        for deg in range(360):
            d = r.lidar_data[deg] * 8
            rad = np.radians(deg - 90)
            p.setPen(QColor(0, 255, 0) if r.lidar_data[deg] < LIDAR_RANGE else QColor(0, 40, 0))
            p.drawPoint(int(cx + np.cos(rad) * d), int(cy + np.sin(rad) * d))

    def paint_slam(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, 500, 0, 500, -1, 1)
        glBegin(GL_POINTS)
        glColor3f(0, 1, 1)
        y, x = np.where(self.slam_map > 0.5)
        for py, px in zip(y, x):
            glVertex2f(px, py)
        glEnd()

    # --- USER ACTIONS & UTILS ---

    def manual_attach(self, robot=None):
        """Attaches a robot to a nearby moveable object if it isn't already held."""
        r = robot if robot else self.world['robots'][self.active_idx]
        if not r.attached_obj:
            for o in self.world['objects']:
                if o.moveable and not o.is_held and o.get_dist_to_point(r.pos[0], r.pos[1]) < 0.65:
                    r.attached_obj = o
                    o.is_held = True
                    return

    def manual_detach(self, robot=None):
        """Detaches object and frees it for other robots."""
        r = robot if robot else self.world['robots'][self.active_idx]
        if r.attached_obj:
            r.attached_obj.is_held = False
            r.attached_obj = None

    def spawn_robot(self):
        """Adds a new robot to the simulation in a clear spot."""
        import random
        for _ in range(100):
            p = (random.uniform(1, 9), random.uniform(1, 9))
            if not self.check_collision(Robot(-1, p), np.array(p), 0):
                new_r = Robot(len(self.world['robots']), p)
                self.world['robots'].append(new_r)
                self.active_idx = new_r.id
                self.status.setText(f"ACTIVE: ROBOT_{new_r.id}")
                return

    def open_editor(self):
        r = self.world['robots'][self.active_idx]
        if r.id not in self.controllers or not self.controllers[r.id].isVisible():
            self.controllers[r.id] = RobotController(r, self)
        self.controllers[r.id].show()
        self.controllers[r.id].raise_()
        self.controllers[r.id].activateWindow()

    def load_world_dialog(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open World", "", "Python (*.py)", options=QFileDialog.Option.DontUseNativeDialog)
        if path:
            self.load_world(path)

    def load_world(self, path):
        self.world['objects'] = []
        exec(open(path).read(), {"create_object": self.create_obj})

    def create_obj(self, name, o_type, color, physics, size, pos, orientation):
        self.world['objects'].append(WorldObject(name, o_type, color, physics, size, pos, orientation))

    # --- INPUT EVENTS ---

    def keyPressEvent(self, event):
        self.keys.add(event.key())
        # Switching active robot with 0-9 keys
        if Qt.Key.Key_0 <= event.key() <= Qt.Key.Key_9:
            idx = event.key() - Qt.Key.Key_0
            if idx < len(self.world['robots']):
                self.active_idx = idx
                self.status.setText(f"ACTIVE: ROBOT_{idx}")

    def keyReleaseEvent(self, event):
        self.keys.discard(event.key())

    def view_press(self, e):
        self.last_m = e.position()

    def view_move(self, e):
        if self.last_m:
            diff = e.position() - self.last_m
            self.ry += diff.x()*0.4
            self.rx = max(10, min(85, self.rx + diff.y()*0.4))
            self.last_m = e.position()

    def view_zoom(self, e):
        self.dist = max(2, min(40, self.dist - e.angleDelta().y()*0.01))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow(sys.argv[1] if len(sys.argv) > 1 else None)
    window.show()
    sys.exit(app.exec())