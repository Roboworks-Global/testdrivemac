#!/usr/bin/env python3
"""Logbook — a two-pane note-keeping app styled like Mac Notes."""

import json
import os
import sys
import uuid
from datetime import date, datetime, timedelta

from PyQt6.QtCore import (
    QDate, QDateTime, Qt, QTimer, pyqtSignal,
)
from PyQt6.QtGui import (
    QColor, QFont, QFontMetrics, QPalette, QTextDocument,
)
from PyQt6.QtPrintSupport import QPrinter, QPrintDialog
from PyQt6.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox,
    QFileDialog, QFrame, QHBoxLayout, QLabel, QListWidget,
    QListWidgetItem, QMainWindow, QMessageBox, QPushButton,
    QSizePolicy, QSplitter, QTextEdit, QToolBar, QVBoxLayout,
    QWidget,
)

# ─── Persistence ─────────────────────────────────────────────────────────────

_APP_DIR = os.path.dirname(os.path.abspath(__file__))
_DATA_FILE = os.path.join(_APP_DIR, "logbook_data.json")


def _load_notes():
    if os.path.isfile(_DATA_FILE):
        try:
            with open(_DATA_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return []


def _save_notes(notes):
    with open(_DATA_FILE, "w", encoding="utf-8") as f:
        json.dump(notes, f, indent=2, ensure_ascii=False)


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _date_label(iso_date_str: str) -> str:
    """Return 'Today', 'Yesterday', or a formatted date string."""
    try:
        d = date.fromisoformat(iso_date_str)
    except Exception:
        return iso_date_str
    today = date.today()
    if d == today:
        return "Today"
    if d == today - timedelta(days=1):
        return "Yesterday"
    return d.strftime("%B %-d, %Y")


def _note_title(note: dict) -> str:
    body = note.get("body", "").strip()
    if not body:
        return "New Entry"
    first_line = body.split("\n")[0].strip()
    return first_line[:60] or "New Entry"


def _note_preview(note: dict) -> str:
    body = note.get("body", "").strip()
    lines = [l for l in body.split("\n") if l.strip()]
    if len(lines) > 1:
        return lines[1][:80]
    return ""


def _note_time(note: dict) -> str:
    ts = note.get("created_at", "")
    try:
        dt = datetime.fromisoformat(ts)
        return dt.strftime("%-I:%M %p")
    except Exception:
        return ""


# ─── Export dialog ────────────────────────────────────────────────────────────

class ExportDialog(QDialog):
    def __init__(self, note_body: str, note_title: str, parent=None):
        super().__init__(parent)
        self._body = note_body
        self._title = note_title
        self.setWindowTitle("Log Export")
        self.setModal(True)
        self.setFixedSize(380, 200)
        self._build_ui()

    def _build_ui(self):
        self.setStyleSheet("""
            QDialog {
                background: #2C2C2E;
            }
            QLabel {
                color: #EBEBF5;
                font-size: 14px;
            }
            QComboBox {
                background: #3A3A3C;
                color: #EBEBF5;
                border: 1px solid #555;
                border-radius: 6px;
                padding: 6px 12px;
                font-size: 13px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background: #3A3A3C;
                color: #EBEBF5;
                selection-background-color: #0A84FF;
            }
            QPushButton {
                background: #3A3A3C;
                color: #EBEBF5;
                border: 1px solid #555;
                border-radius: 8px;
                padding: 8px 22px;
                font-size: 13px;
            }
            QPushButton:hover { background: #48484A; }
            QPushButton#export_btn {
                background: #0A84FF;
                color: white;
                border: none;
            }
            QPushButton#export_btn:hover { background: #0070E0; }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(14)
        layout.setContentsMargins(24, 24, 24, 20)

        instr = QLabel("Choose the file format to export.")
        layout.addWidget(instr)

        self._fmt_combo = QComboBox()
        self._fmt_combo.addItems(["Text File", "PDF File"])
        layout.addWidget(self._fmt_combo)

        btn_row = QHBoxLayout()
        btn_row.addStretch()
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        export_btn = QPushButton("Export")
        export_btn.setObjectName("export_btn")
        export_btn.clicked.connect(self._do_export)
        btn_row.addWidget(cancel_btn)
        btn_row.addWidget(export_btn)
        layout.addLayout(btn_row)

    def _do_export(self):
        fmt = self._fmt_combo.currentText()
        if fmt == "Text File":
            self._export_text()
        else:
            self._export_pdf()

    def _export_text(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Export as Text", f"{self._title}.txt", "Text Files (*.txt)"
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(f"{self._title}\n{'─' * len(self._title)}\n\n{self._body}\n")
            QMessageBox.information(self, "Export", f"Exported to:\n{path}")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Export Failed", str(e))

    def _export_pdf(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Export as PDF", f"{self._title}.pdf", "PDF Files (*.pdf)"
        )
        if not path:
            return
        try:
            printer = QPrinter(QPrinter.PrinterMode.HighResolution)
            printer.setOutputFormat(QPrinter.OutputFormat.PdfFormat)
            printer.setOutputFileName(path)
            doc = QTextDocument()
            doc.setPlainText(f"{self._title}\n{'─' * len(self._title)}\n\n{self._body}")
            doc.print(printer)
            QMessageBox.information(self, "Export", f"Exported to:\n{path}")
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Export Failed", str(e))


# ─── Note list item widget ────────────────────────────────────────────────────

class NoteItemWidget(QWidget):
    """Custom widget for a note row in the list."""

    def __init__(self, note: dict, parent=None):
        super().__init__(parent)
        self._note_id = note["id"]
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)

        top_row = QHBoxLayout()
        title_lbl = QLabel(_note_title(note))
        title_lbl.setFont(QFont("Helvetica Neue", 13, QFont.Weight.Medium))
        title_lbl.setStyleSheet("color: #F2F2F7;")
        time_lbl = QLabel(_note_time(note))
        time_lbl.setFont(QFont("Helvetica Neue", 11))
        time_lbl.setStyleSheet("color: #8E8E93;")
        top_row.addWidget(title_lbl)
        top_row.addStretch()
        top_row.addWidget(time_lbl)

        preview_lbl = QLabel(_note_preview(note))
        preview_lbl.setFont(QFont("Helvetica Neue", 12))
        preview_lbl.setStyleSheet("color: #8E8E93;")
        preview_lbl.setMaximumWidth(260)

        layout.addLayout(top_row)
        layout.addWidget(preview_lbl)


# ─── Section header item widget ───────────────────────────────────────────────

class SectionHeaderWidget(QWidget):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 4)
        lbl = QLabel(label)
        lbl.setFont(QFont("Helvetica Neue", 12, QFont.Weight.Bold))
        lbl.setStyleSheet("color: #8E8E93;")
        layout.addWidget(lbl)
        layout.addStretch()


# ─── Main Window ──────────────────────────────────────────────────────────────

class LogbookWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Logbook")
        self.resize(900, 620)
        self._notes: list[dict] = _load_notes()
        self._current_id: str | None = None
        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(800)
        self._save_timer.timeout.connect(self._flush_current_note)
        self._inject_testdrive_log()
        self._build_ui()
        self._populate_list()
        self._select_first()

    # ── TestDrive log injection ───────────────────────────────────────────────

    def _inject_testdrive_log(self):
        """Create a new top entry from .testdrive_log.txt every time Logbook starts."""
        log_path = os.path.join(_APP_DIR, ".testdrive_log.txt")
        if not os.path.isfile(log_path):
            return
        try:
            with open(log_path, "r", encoding="utf-8") as f:
                content = f.read().strip()
        except Exception:
            return
        if not content:
            return
        note = {
            "id": str(uuid.uuid4()),
            "created_at": datetime.now().isoformat(),
            "body": content,
        }
        self._notes.insert(0, note)
        _save_notes(self._notes)

    # ── UI Construction ──────────────────────────────────────────────────────

    def _build_ui(self):
        self.setStyleSheet("""
            QMainWindow, QWidget#root {
                background: #1C1C1E;
            }
        """)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        # ── Top toolbar ──────────────────────────────────────────────────────
        toolbar = QWidget()
        toolbar.setFixedHeight(52)
        toolbar.setStyleSheet("background: #2C2C2E; border-bottom: 1px solid #3A3A3C;")
        tb_layout = QHBoxLayout(toolbar)
        tb_layout.setContentsMargins(16, 8, 16, 8)

        title_lbl = QLabel("Logbook")
        title_lbl.setFont(QFont("Helvetica Neue", 17, QFont.Weight.Bold))
        title_lbl.setStyleSheet("color: #F2F2F7;")
        tb_layout.addWidget(title_lbl)
        tb_layout.addStretch()

        self._export_btn = QPushButton("Export")
        self._export_btn.setFixedHeight(32)
        self._export_btn.setStyleSheet("""
            QPushButton {
                background: #0A84FF;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 0 16px;
                font-size: 13px;
            }
            QPushButton:hover { background: #0070E0; }
            QPushButton:pressed { background: #005EC5; }
        """)
        self._export_btn.clicked.connect(self._export)
        tb_layout.addWidget(self._export_btn)

        root_layout.addWidget(toolbar)

        # ── Splitter ─────────────────────────────────────────────────────────
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setStyleSheet("""
            QSplitter::handle { background: #3A3A3C; width: 1px; }
        """)

        # Left pane
        left_pane = QWidget()
        left_pane.setMinimumWidth(240)
        left_pane.setMaximumWidth(320)
        left_pane.setStyleSheet("background: #2C2C2E;")
        left_layout = QVBoxLayout(left_pane)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        self._list = QListWidget()
        self._list.setStyleSheet("""
            QListWidget {
                background: #2C2C2E;
                border: none;
                outline: none;
            }
            QListWidget::item {
                background: transparent;
                border-bottom: 1px solid #3A3A3C;
                padding: 0;
            }
            QListWidget::item:selected {
                background: #3A3A3C;
            }
            QListWidget::item:hover {
                background: #333335;
            }
        """)
        self._list.setSpacing(0)
        self._list.currentItemChanged.connect(self._on_note_selected)
        left_layout.addWidget(self._list)
        splitter.addWidget(left_pane)

        # Right pane
        right_pane = QWidget()
        right_pane.setStyleSheet("background: #1C1C1E;")
        right_layout = QVBoxLayout(right_pane)
        right_layout.setContentsMargins(28, 20, 28, 20)
        right_layout.setSpacing(8)

        self._note_date_lbl = QLabel("")
        self._note_date_lbl.setFont(QFont("Helvetica Neue", 12))
        self._note_date_lbl.setStyleSheet("color: #8E8E93;")
        self._note_date_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_layout.addWidget(self._note_date_lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #3A3A3C;")
        right_layout.addWidget(sep)

        self._editor = QTextEdit()
        self._editor.setStyleSheet("""
            QTextEdit {
                background: #1C1C1E;
                color: #F2F2F7;
                border: none;
                font-family: 'Helvetica Neue';
                font-size: 15px;
                line-height: 1.5;
            }
        """)
        self._editor.setPlaceholderText("Start typing your log entry…")
        self._editor.textChanged.connect(self._on_text_changed)
        right_layout.addWidget(self._editor)

        splitter.addWidget(right_pane)
        splitter.setSizes([280, 620])
        root_layout.addWidget(splitter)

    # ── List population ──────────────────────────────────────────────────────

    def _populate_list(self):
        self._list.clear()
        if not self._notes:
            return

        # Sort notes newest-first by created_at
        sorted_notes = sorted(
            self._notes,
            key=lambda n: n.get("created_at", ""),
            reverse=True,
        )

        # Group by date
        groups: dict[str, list[dict]] = {}
        for note in sorted_notes:
            d = note.get("created_at", "")[:10]
            groups.setdefault(d, []).append(note)

        # Sort groups newest-first
        for date_str in sorted(groups.keys(), reverse=True):
            header_item = QListWidgetItem()
            header_item.setFlags(Qt.ItemFlag.NoItemFlags)
            header_item.setData(Qt.ItemDataRole.UserRole, "__header__")
            header_widget = SectionHeaderWidget(_date_label(date_str))
            header_item.setSizeHint(header_widget.sizeHint())
            self._list.addItem(header_item)
            self._list.setItemWidget(header_item, header_widget)

            for note in groups[date_str]:
                item = QListWidgetItem()
                item.setData(Qt.ItemDataRole.UserRole, note["id"])
                note_widget = NoteItemWidget(note)
                item.setSizeHint(note_widget.sizeHint())
                self._list.addItem(item)
                self._list.setItemWidget(item, note_widget)

    def _select_first(self):
        for i in range(self._list.count()):
            item = self._list.item(i)
            if item and item.data(Qt.ItemDataRole.UserRole) != "__header__":
                self._list.setCurrentItem(item)
                return

    def _refresh_list(self):
        current_id = self._current_id
        self._populate_list()
        # Re-select the same note
        if current_id:
            for i in range(self._list.count()):
                item = self._list.item(i)
                if item and item.data(Qt.ItemDataRole.UserRole) == current_id:
                    self._list.setCurrentItem(item)
                    return

    # ── Event handlers ───────────────────────────────────────────────────────

    def _on_note_selected(self, current: QListWidgetItem, _previous):
        if current is None:
            return
        note_id = current.data(Qt.ItemDataRole.UserRole)
        if note_id == "__header__":
            return
        # Flush previous note before switching
        self._flush_current_note()
        self._current_id = note_id
        note = self._find_note(note_id)
        if note is None:
            return
        self._editor.blockSignals(True)
        self._editor.setPlainText(note.get("body", ""))
        self._editor.blockSignals(False)
        ts = note.get("created_at", "")
        try:
            dt = datetime.fromisoformat(ts)
            self._note_date_lbl.setText(dt.strftime("%A, %B %-d, %Y  %-I:%M %p"))
        except Exception:
            self._note_date_lbl.setText("")

    def _on_text_changed(self):
        self._save_timer.start()

    def _flush_current_note(self):
        if self._current_id is None:
            return
        note = self._find_note(self._current_id)
        if note is None:
            return
        new_body = self._editor.toPlainText()
        if note.get("body") != new_body:
            note["body"] = new_body
            _save_notes(self._notes)
            # Refresh the list item widget
            for i in range(self._list.count()):
                item = self._list.item(i)
                if item and item.data(Qt.ItemDataRole.UserRole) == self._current_id:
                    w = NoteItemWidget(note)
                    item.setSizeHint(w.sizeHint())
                    self._list.setItemWidget(item, w)
                    break

    def _export(self):
        if self._current_id is None:
            QMessageBox.information(self, "Export", "No entry selected.")
            return
        self._flush_current_note()
        note = self._find_note(self._current_id)
        if note is None:
            return
        dlg = ExportDialog(
            note_body=note.get("body", ""),
            note_title=_note_title(note),
            parent=self,
        )
        dlg.exec()

    # ── Utilities ────────────────────────────────────────────────────────────

    def _find_note(self, note_id: str) -> dict | None:
        for n in self._notes:
            if n["id"] == note_id:
                return n
        return None

    def closeEvent(self, event):
        self._flush_current_note()
        super().closeEvent(event)


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Logbook")
    app.setStyle("Fusion")

    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor("#1C1C1E"))
    palette.setColor(QPalette.ColorRole.WindowText, QColor("#F2F2F7"))
    palette.setColor(QPalette.ColorRole.Base, QColor("#2C2C2E"))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#3A3A3C"))
    palette.setColor(QPalette.ColorRole.Text, QColor("#F2F2F7"))
    palette.setColor(QPalette.ColorRole.Button, QColor("#3A3A3C"))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor("#F2F2F7"))
    palette.setColor(QPalette.ColorRole.Highlight, QColor("#0A84FF"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#FFFFFF"))
    app.setPalette(palette)

    win = LogbookWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
