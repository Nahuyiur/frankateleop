"""Shared MUKA Robotics visual tokens for auxiliary PyQt windows."""

from __future__ import annotations

from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets


MUKA_BLUE = "#2b659a"
MUKA_DEEP_BLUE = "#1324a2"
MUKA_NAVY = "#0c015e"
MUKA_CYAN = "#80cde1"
MUKA_ORANGE = "#f87512"
MUKA_RED = "#ce4426"
MUKA_PURPLE = "#603daf"
MUKA_INK = "#12181f"
MUKA_CHARCOAL = "#302e2c"
MUKA_SURFACE = "#f5f7f9"
MUKA_CREAM = "#f9f6e8"
MUKA_LOGO_PATH = Path(__file__).resolve().parent / "assets" / "muka_logo.png"


def make_aux_header(title: str, subtitle: str, status: str) -> QtWidgets.QFrame:
    """Build the shared compact brand header used by read-only operator tools."""
    header = QtWidgets.QFrame()
    header.setObjectName("AuxHeader")
    header.setFixedHeight(88)
    layout = QtWidgets.QHBoxLayout(header)
    layout.setContentsMargins(24, 15, 24, 15)
    layout.setSpacing(14)

    logo_label = QtWidgets.QLabel()
    logo_label.setObjectName("AuxBrandLogo")
    logo_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
    logo_label.setFixedSize(78, 42)
    logo = QtGui.QPixmap(str(MUKA_LOGO_PATH))
    if logo.isNull():
        logo_label.setText("M")
        logo_label.setProperty("fallback", True)
        logo_label.setFixedSize(42, 42)
    else:
        visible_bounds = QtGui.QRegion(logo.mask()).boundingRect()
        if visible_bounds.isValid() and not visible_bounds.isEmpty():
            logo = logo.copy(visible_bounds)
        logo_label.setPixmap(
            logo.scaled(
                logo_label.size(),
                QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                QtCore.Qt.TransformationMode.SmoothTransformation,
            )
        )
    layout.addWidget(logo_label)

    heading = QtWidgets.QVBoxLayout()
    heading.setSpacing(1)
    title_label = QtWidgets.QLabel(title)
    title_label.setObjectName("AuxHeaderTitle")
    subtitle_label = QtWidgets.QLabel(subtitle)
    subtitle_label.setObjectName("AuxHeaderSubtitle")
    heading.addWidget(title_label)
    heading.addWidget(subtitle_label)
    layout.addLayout(heading)
    layout.addStretch()

    status_label = QtWidgets.QLabel(status)
    status_label.setObjectName("AuxHeaderStatus")
    layout.addWidget(status_label)
    return header


AUXILIARY_WINDOW_STYLESHEET = f"""
QMainWindow {{
    background: {MUKA_SURFACE};
}}
QWidget {{
    color: {MUKA_INK};
    font-family: "Noto Sans CJK SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
    font-size: 13px;
}}
QFrame#AuxHeader {{
    background: {MUKA_BLUE};
    border: 0;
}}
QLabel#AuxBrandLogo {{
    background: transparent;
    border: 0;
}}
QLabel#AuxBrandLogo[fallback="true"] {{
    background: {MUKA_NAVY};
    color: white;
    border-radius: 7px;
    font-size: 18px;
    font-weight: 800;
}}
QLabel#AuxHeaderTitle {{
    color: white;
    font-size: 22px;
    font-weight: 800;
}}
QLabel#AuxHeaderSubtitle {{
    color: {MUKA_CYAN};
    font-size: 11px;
    font-weight: 650;
}}
QLabel#AuxHeaderStatus {{
    background: #255985;
    color: white;
    border: 1px solid {MUKA_CYAN};
    border-radius: 7px;
    padding: 7px 11px;
    font-size: 12px;
    font-weight: 700;
}}
QFrame#ToolBarFrame, QFrame#PathBar, QFrame#PlaybackBar {{
    background: white;
    border: 1px solid #d7e0e7;
    border-radius: 8px;
}}
QLabel#SectionTitle {{
    color: {MUKA_INK};
    font-size: 16px;
    font-weight: 800;
}}
QLabel#SectionMeta, QLabel#FilterLabel {{
    color: #667582;
    font-size: 11px;
    font-weight: 650;
}}
QLabel#InfoStrip {{
    background: white;
    color: {MUKA_CHARCOAL};
    border: 1px solid #cbd6df;
    border-left: 4px solid {MUKA_BLUE};
    border-radius: 6px;
    padding: 8px 10px;
}}
QFrame#MetricCard {{
    background: white;
    border: 1px solid #d7e0e7;
    border-top: 3px solid {MUKA_BLUE};
    border-radius: 7px;
}}
QFrame#MetricCard[metric="recording"] {{ border-top-color: {MUKA_CYAN}; }}
QFrame#MetricCard[metric="rest"] {{ border-top-color: #9eabb6; }}
QFrame#MetricCard[metric="remaining"] {{ border-top-color: {MUKA_PURPLE}; }}
QFrame#MetricCard[metric="attempts"] {{ border-top-color: {MUKA_ORANGE}; }}
QLabel#MetricCaption {{
    color: #667582;
    font-size: 12px;
    font-weight: 650;
}}
QLabel#MetricValue {{
    color: {MUKA_INK};
    font-size: 22px;
    font-weight: 800;
}}
QLabel#SummaryBadge {{
    border-radius: 6px;
    padding: 6px 10px;
    font-size: 12px;
    font-weight: 800;
}}
QLabel#SummaryBadge[status="idle"], QLabel#SummaryBadge[status="loading"] {{
    background: #e8f1f6;
    color: {MUKA_BLUE};
    border: 1px solid #bfd3e1;
}}
QLabel#SummaryBadge[status="pass"] {{
    background: #e9f6f0;
    color: #176247;
    border: 1px solid #acd9c7;
}}
QLabel#SummaryBadge[status="warn"] {{
    background: #fff5e9;
    color: #9b4b12;
    border: 1px solid #f0cda6;
}}
QLabel#SummaryBadge[status="fail"] {{
    background: #fff0ed;
    color: #a63220;
    border: 1px solid #e7b3a9;
}}
QLabel#ReviewSummary {{
    color: {MUKA_CHARCOAL};
    padding: 2px 4px;
}}
QLabel#FrameReadout {{
    background: #eef3f6;
    color: {MUKA_NAVY};
    border-radius: 5px;
    padding: 6px 9px;
    font-family: monospace;
    font-size: 12px;
}}
QFrame#MediaWorkspace {{
    background: #e9eef2;
    border: 1px solid #d7e0e7;
    border-radius: 7px;
}}
QLabel#MediaEmptyState {{
    color: #71808d;
    font-size: 14px;
    font-weight: 650;
}}
QLineEdit, QComboBox, QDateEdit {{
    min-height: 32px;
    background: white;
    color: {MUKA_INK};
    border: 1px solid #aebfce;
    border-radius: 6px;
    padding: 0 8px;
    selection-background-color: {MUKA_BLUE};
    selection-color: white;
}}
QLineEdit:focus, QComboBox:focus, QDateEdit:focus {{
    border: 2px solid {MUKA_BLUE};
}}
QPushButton, QToolButton {{
    min-height: 34px;
    background: white;
    color: {MUKA_INK};
    border: 1px solid #aebfce;
    border-radius: 6px;
    padding: 0 12px;
    font-weight: 700;
}}
QToolButton#PlaybackButton {{
    min-width: 36px;
    max-width: 36px;
    min-height: 36px;
    max-height: 36px;
    padding: 0;
}}
QToolButton#PlaybackButton[primary="true"] {{
    background: {MUKA_DEEP_BLUE};
    color: white;
    border-color: {MUKA_DEEP_BLUE};
}}
QPushButton:hover, QToolButton:hover {{
    color: {MUKA_DEEP_BLUE};
    background: #e8f5f8;
    border-color: {MUKA_BLUE};
}}
QPushButton#PrimaryAction {{
    color: white;
    background: {MUKA_BLUE};
    border-color: {MUKA_BLUE};
}}
QPushButton#PrimaryAction:hover {{
    background: {MUKA_DEEP_BLUE};
    border-color: {MUKA_DEEP_BLUE};
}}
QTableWidget {{
    background: white;
    alternate-background-color: #f4f8fa;
    border: 1px solid #cbd6df;
    border-radius: 6px;
    gridline-color: #e1e7eb;
    selection-background-color: #dcecf4;
    selection-color: {MUKA_NAVY};
}}
QTableWidget::item {{
    padding: 5px;
}}
QHeaderView::section {{
    background: #e8f1f6;
    color: {MUKA_NAVY};
    border: 0;
    border-right: 1px solid #cbd6df;
    border-bottom: 1px solid #b9cad8;
    padding: 7px;
    font-weight: 750;
}}
QTabWidget::pane {{
    background: white;
    border: 1px solid #cbd6df;
    border-radius: 6px;
}}
QTabBar::tab {{
    background: #eef3f6;
    color: #4b5965;
    border: 1px solid #cbd6df;
    padding: 8px 15px;
    font-weight: 650;
}}
QTabBar::tab:selected {{
    background: {MUKA_DEEP_BLUE};
    color: white;
    border-color: {MUKA_DEEP_BLUE};
}}
QFrame#reviewCamera {{
    background: white;
    border: 1px solid #d7e0e7;
    border-radius: 7px;
}}
QLabel#CameraName {{
    color: {MUKA_BLUE};
    font-weight: 750;
}}
QLabel#CameraIndicator {{
    color: {MUKA_CYAN};
    font-size: 10px;
}}
QSlider::groove:horizontal {{
    height: 5px;
    background: #cbd6df;
    border-radius: 2px;
}}
QSlider::sub-page:horizontal {{
    background: {MUKA_BLUE};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    width: 15px;
    margin: -5px 0;
    background: {MUKA_DEEP_BLUE};
    border: 2px solid white;
    border-radius: 7px;
}}
QSplitter::handle {{
    background: #d9e3e9;
}}
QToolTip {{
    background: {MUKA_INK};
    color: white;
    border: 1px solid {MUKA_CYAN};
    padding: 4px 6px;
}}
"""
