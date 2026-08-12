from __future__ import annotations

import numpy as np
from PyQt6 import QtCore, QtGui, QtWidgets

from franka_gui.brand_theme import (
    MUKA_BLUE,
    MUKA_CYAN,
    MUKA_INK,
    MUKA_ORANGE,
    MUKA_PURPLE,
    MUKA_RED,
)

from .model import (
    ArmSeries,
    EpisodeReview,
    JOINT_LIMITS_HIGH,
    JOINT_LIMITS_LOW,
    JOINT_NAMES,
    ReviewEvent,
)


TRACE_COLORS = (
    QtGui.QColor("#2b659a"),
    QtGui.QColor("#ce4426"),
    QtGui.QColor("#1324a2"),
    QtGui.QColor("#603daf"),
    QtGui.QColor("#f87512"),
    QtGui.QColor("#4e4ca9"),
    QtGui.QColor("#441c9b"),
)


class CameraView(QtWidgets.QFrame):
    def __init__(self, name: str, parent=None) -> None:
        super().__init__(parent)
        self.setObjectName("reviewCamera")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 8)
        layout.setSpacing(6)
        title_row = QtWidgets.QHBoxLayout()
        title_row.setSpacing(6)
        indicator = QtWidgets.QLabel("●")
        indicator.setObjectName("CameraIndicator")
        title = QtWidgets.QLabel(name)
        title.setObjectName("CameraName")
        title_row.addWidget(indicator)
        title_row.addWidget(title)
        title_row.addStretch()
        self.image = QtWidgets.QLabel("等待画面")
        self.image.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.image.setMinimumSize(180, 90)
        self.image.setStyleSheet("background:#12181f;color:#80cde1;border-radius:5px;")
        layout.addLayout(title_row)
        layout.addWidget(self.image, 1)

    def set_bgr(self, image: np.ndarray) -> None:
        rgb = np.ascontiguousarray(image[:, :, ::-1])
        height, width, channels = rgb.shape
        qimage = QtGui.QImage(rgb.data, width, height, width * channels, QtGui.QImage.Format.Format_RGB888).copy()
        pixmap = QtGui.QPixmap.fromImage(qimage).scaled(
            self.image.size(),
            QtCore.Qt.AspectRatioMode.KeepAspectRatio,
            QtCore.Qt.TransformationMode.SmoothTransformation,
        )
        self.image.setPixmap(pixmap)

    def show_error(self, message: str = "画面读取失败") -> None:
        self.image.clear()
        self.image.setText(message)
        self.image.setStyleSheet(
            "background:#241411;color:#f87512;border:1px solid #ce4426;border-radius:4px;"
        )


class JointTimelineWidget(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.timeline = np.empty(0)
        self.joints = np.empty((0, 7))
        self.events: tuple[ReviewEvent, ...] = ()
        self.keyframes: tuple[int, ...] = ()
        self.cursor_time = 0.0
        self.setMinimumHeight(140)

    def set_data(
        self,
        timeline: np.ndarray,
        arm: ArmSeries,
        events: tuple[ReviewEvent, ...],
        keyframes: tuple[int, ...],
    ) -> None:
        self.timeline = timeline
        self.joints = arm.joints
        self.events = tuple(item for item in events if item.arm == arm.name)
        self.keyframes = keyframes
        self.cursor_time = 0.0
        self.update()

    def set_cursor(self, seconds: float) -> None:
        self.cursor_time = max(0.0, float(seconds))
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#fbfcfd"))
        left, right, top, bottom = 58, 14, 8, 24
        width = max(1, self.width() - left - right)
        height = max(1, self.height() - top - bottom)
        row_gap = 4
        row_height = (height - row_gap * 6) / 7
        duration = max(0.1, float(self.timeline[-1]) if self.timeline.size else 0.1)

        for joint_index, name in enumerate(JOINT_NAMES):
            row_top = top + joint_index * (row_height + row_gap)
            row_bottom = row_top + row_height
            lower = float(JOINT_LIMITS_LOW[joint_index])
            upper = float(JOINT_LIMITS_HIGH[joint_index])
            padding = max(0.08, (upper - lower) * 0.08)
            display_low, display_high = lower - padding, upper + padding

            def y_for(value: float) -> float:
                ratio = (min(display_high, max(display_low, value)) - display_low) / (display_high - display_low)
                return row_bottom - ratio * row_height

            painter.fillRect(QtCore.QRectF(left, y_for(upper), width, y_for(lower) - y_for(upper)), QtGui.QColor("#e8f5f8"))
            painter.setPen(QtGui.QPen(QtGui.QColor(MUKA_ORANGE), 1, QtCore.Qt.PenStyle.DashLine))
            painter.drawLine(QtCore.QPointF(left, y_for(lower)), QtCore.QPointF(left + width, y_for(lower)))
            painter.drawLine(QtCore.QPointF(left, y_for(upper)), QtCore.QPointF(left + width, y_for(upper)))
            painter.setPen(QtGui.QColor(MUKA_INK))
            painter.drawText(QtCore.QRectF(4, row_top, left - 10, row_height), QtCore.Qt.AlignmentFlag.AlignVCenter, name)
            if self.timeline.size and self.joints.shape[0] == self.timeline.size:
                indices = np.unique(np.linspace(0, len(self.timeline) - 1, min(len(self.timeline), max(2, width * 2)), dtype=int))
                path = QtGui.QPainterPath()
                for point_number, source_index in enumerate(indices):
                    x = left + float(self.timeline[source_index]) / duration * width
                    y = y_for(float(self.joints[source_index, joint_index]))
                    path.moveTo(x, y) if point_number == 0 else path.lineTo(x, y)
                painter.setPen(QtGui.QPen(TRACE_COLORS[joint_index], 1.6))
                painter.drawPath(path)
            for item in self.events:
                if item.joint_index != joint_index:
                    continue
                x = left + float(self.timeline[item.frame_index]) / duration * width
                color = QtGui.QColor(MUKA_RED if item.severity == "fail" else MUKA_ORANGE)
                painter.setPen(QtGui.QPen(color, 1.3))
                painter.drawLine(QtCore.QPointF(x, row_top + 2), QtCore.QPointF(x, row_bottom - 2))
            painter.setPen(QtGui.QColor("#d9e3e9"))
            painter.drawLine(QtCore.QPointF(left, row_bottom), QtCore.QPointF(left + width, row_bottom))

        painter.setPen(QtGui.QPen(QtGui.QColor(MUKA_PURPLE), 1.8))
        cursor_x = left + min(duration, self.cursor_time) / duration * width
        painter.drawLine(QtCore.QPointF(cursor_x, top), QtCore.QPointF(cursor_x, self.height() - bottom))
        painter.setPen(QtGui.QColor("#66727e"))
        painter.drawText(left, self.height() - 18, "0.0 s")
        painter.drawText(self.width() - 90, self.height() - 18, 75, 16, QtCore.Qt.AlignmentFlag.AlignRight, f"{duration:.1f} s")


class EventStripWidget(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.review: EpisodeReview | None = None
        self.cursor_time = 0.0
        self.setFixedHeight(48)

    def set_review(self, review: EpisodeReview) -> None:
        self.review = review
        self.update()

    def set_cursor(self, seconds: float) -> None:
        self.cursor_time = seconds
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#fbfcfd"))
        left, right, y, height = 58, 14, 14, 12
        width = max(1, self.width() - left - right)
        painter.setPen(QtGui.QColor(MUKA_INK))
        painter.drawText(4, y - 1, "质量事件")
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor("#dceef3"))
        painter.drawRoundedRect(QtCore.QRectF(left, y, width, height), 4, 4)
        if self.review is None:
            return
        duration = max(0.1, self.review.duration)
        for item in self.review.events:
            x = left + float(self.review.timeline[item.frame_index]) / duration * width
            color = QtGui.QColor(MUKA_RED if item.severity == "fail" else MUKA_ORANGE)
            painter.fillRect(QtCore.QRectF(x - 1.5, y - 3, 3, height + 6), color)
        for frame_index in self.review.keyframes:
            x = left + float(self.review.timeline[frame_index]) / duration * width
            painter.setPen(QtGui.QPen(QtGui.QColor(MUKA_BLUE), 1, QtCore.Qt.PenStyle.DotLine))
            painter.drawLine(QtCore.QPointF(x, y - 7), QtCore.QPointF(x, y + height + 7))
        cursor_x = left + min(duration, self.cursor_time) / duration * width
        painter.setPen(QtGui.QPen(QtGui.QColor(MUKA_PURPLE), 2))
        painter.drawLine(QtCore.QPointF(cursor_x, y - 9), QtCore.QPointF(cursor_x, y + height + 9))
        painter.setPen(QtGui.QColor("#66727e"))
        painter.drawText(left, 44, "青色=正常  橙色=跳变预警  红色=超限/高速度  蓝虚线=关键帧")


class EndEffectorPathWidget(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.pose = np.empty((0, 6))
        self.cursor_index = 0
        self.setMinimumHeight(180)

    def set_pose(self, pose: np.ndarray) -> None:
        self.pose = pose
        self.cursor_index = 0
        self.update()

    def set_cursor(self, index: int) -> None:
        self.cursor_index = max(0, int(index))
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#fbfcfd"))
        panels = (("XY 俯视轨迹", 0, 1), ("XZ 侧视轨迹", 0, 2))
        gap = 24
        panel_width = (self.width() - gap * 3) / 2
        for panel_index, (title, x_axis, y_axis) in enumerate(panels):
            rect = QtCore.QRectF(gap + panel_index * (panel_width + gap), 34, panel_width, self.height() - 55)
            painter.setPen(QtGui.QColor(MUKA_INK))
            painter.drawText(QtCore.QRectF(rect.left(), 8, rect.width(), 22), QtCore.Qt.AlignmentFlag.AlignCenter, title)
            painter.setPen(QtGui.QColor("#d9e3e9"))
            painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
            painter.drawRect(rect)
            if self.pose.shape[0] < 2:
                continue
            x_values, y_values = self.pose[:, x_axis], self.pose[:, y_axis]
            x_low, x_high = float(x_values.min()), float(x_values.max())
            y_low, y_high = float(y_values.min()), float(y_values.max())
            x_span, y_span = max(1e-6, x_high - x_low), max(1e-6, y_high - y_low)

            def point(index: int) -> QtCore.QPointF:
                x = rect.left() + (float(x_values[index]) - x_low) / x_span * rect.width()
                y = rect.bottom() - (float(y_values[index]) - y_low) / y_span * rect.height()
                return QtCore.QPointF(x, y)

            indices = np.unique(np.linspace(0, len(self.pose) - 1, min(len(self.pose), max(2, int(rect.width() * 2))), dtype=int))
            for offset in range(1, len(indices)):
                ratio = offset / max(1, len(indices) - 1)
                start, end = QtGui.QColor(MUKA_CYAN), QtGui.QColor(MUKA_BLUE)
                color = QtGui.QColor(
                    round(start.red() + (end.red() - start.red()) * ratio),
                    round(start.green() + (end.green() - start.green()) * ratio),
                    round(start.blue() + (end.blue() - start.blue()) * ratio),
                )
                painter.setPen(QtGui.QPen(color, 1.8))
                painter.drawLine(point(int(indices[offset - 1])), point(int(indices[offset])))
            index = min(self.cursor_index, len(self.pose) - 1)
            painter.setPen(QtCore.Qt.PenStyle.NoPen)
            painter.setBrush(QtGui.QColor(MUKA_PURPLE))
            painter.drawEllipse(point(index), 4.5, 4.5)


class ScalarTimelineWidget(QtWidgets.QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.timeline = np.empty(0)
        self.series: list[tuple[str, np.ndarray, QtGui.QColor, QtCore.Qt.PenStyle]] = []
        self.cursor_time = 0.0
        self.setMinimumHeight(140)

    def set_series(self, timeline: np.ndarray, series) -> None:
        self.timeline = timeline
        self.series = list(series)
        self.update()

    def set_cursor(self, seconds: float) -> None:
        self.cursor_time = seconds
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#fbfcfd"))
        left, right, top, bottom = 58, 14, 18, 24
        width, height = max(1, self.width() - left - right), max(1, self.height() - top - bottom)
        values = np.concatenate([value[np.isfinite(value)] for _, value, _, _ in self.series if value.size]) if any(value.size for _, value, _, _ in self.series) else np.empty(0)
        if not self.timeline.size or not values.size:
            painter.setPen(QtGui.QColor("#71808d"))
            painter.drawText(self.rect(), QtCore.Qt.AlignmentFlag.AlignCenter, "该 episode 没有夹爪宽度数据")
            return
        low, high = float(values.min()), float(values.max())
        span = max(1e-6, high - low)
        duration = max(0.1, float(self.timeline[-1]))
        for label, series, color, style in self.series:
            if series.size != self.timeline.size:
                continue
            path = QtGui.QPainterPath()
            indices = np.unique(np.linspace(0, len(series) - 1, min(len(series), max(2, width * 2)), dtype=int))
            for point_number, index in enumerate(indices):
                x = left + float(self.timeline[index]) / duration * width
                y = top + (high - float(series[index])) / span * height
                path.moveTo(x, y) if point_number == 0 else path.lineTo(x, y)
            painter.setPen(QtGui.QPen(color, 1.7, style))
            painter.drawPath(path)
        painter.setPen(QtGui.QPen(QtGui.QColor(MUKA_PURPLE), 1.8))
        x = left + min(duration, self.cursor_time) / duration * width
        painter.drawLine(QtCore.QPointF(x, top), QtCore.QPointF(x, top + height))
        painter.setPen(QtGui.QColor("#66727e"))
        painter.drawText(6, 14, "夹爪宽度")
        painter.drawText(left, self.height() - 5, "实线=反馈  虚线=目标")
