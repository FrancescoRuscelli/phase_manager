import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QHBoxLayout, QMenuBar, \
    QAction, QTextEdit, QSizePolicy, QStyleOptionFrame, QStyle
from PyQt5.QtGui import QPainter, QColor, QBrush, QPainterPath, QPen, QFont, QFontMetrics, QCursor
from PyQt5.QtCore import Qt, QTimer, QRectF, pyqtSignal, QSize
from ros_client_class import TimelineROS, PhaseInfo, TimelineInfo
import hashlib
import random

def string_to_color(input_string, seed=None):
    if seed is not None:
        random.seed(seed)
    hash_value = int(hashlib.sha256(input_string.encode()).hexdigest(), 16)
    r = (hash_value & 0xFF0000) >> 16
    g = (hash_value & 0x00FF00) >> 8
    b = hash_value & 0x0000FF
    a = 255
    return (r, g, b, a)


class ElideLabel(QLabel):
    _elideMode = Qt.ElideMiddle

    def __init__(self, text='', parent=None):
        super().__init__(text, parent)
        self.setWordWrap(True)  # Wrap text if it doesn't fit horizontally
        self.elidedText = text

    def setText(self, text):
        self.elidedText = text
        super().setText(text)

    def elideMode(self):
        return self._elideMode

    def setElideMode(self, mode):
        if self._elideMode != mode and mode != Qt.ElideNone:
            self._elideMode = mode
            self.updateGeometry()

    def minimumSizeHint(self):
        return self.sizeHint()

    def sizeHint(self):
        hint = self.fontMetrics().boundingRect(self.text()).size()
        l, t, r, b = self.getContentsMargins()
        margin = self.margin() * 2
        return QSize(min(100, hint.width()) + l + r + margin, min(self.fontMetrics().height(), hint.height()) + t + b + margin)

    def paintEvent(self, event):

        qp = QPainter(self)
        elidedText = self.fontMetrics().elidedText(self.text(), self.elideMode(), self.rect().width())
        qp.drawText(self.rect(), self.alignment(), elidedText)

class PhaseInfoBox(QWidget):
    def __init__(self, tags):
        super().__init__()

        self.grid_layout = QGridLayout(self)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)

        label_texts = {
            (0, 0): "Name:",
            (1, 0): "Initial Node:",
            (2, 0): "Duration:",
            (0, 1): tags[0],
            (1, 1): tags[1],
            (2, 1): tags[2]
        }

        self.grid_layout.setColumnStretch(1, 10)

        self.labels = {}
        for (row, col), text in label_texts.items():
            if col == 0:
                label = QLabel(text)
            else:
                label = ElideLabel(text)

            self.grid_layout.addWidget(label, row, col)
            self.labels[(row, col)] = label
            label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            if col == 1:
                value_font = QFont()
                value_font.setBold(True)
                label.setFont(value_font)

    def setText(self, tags):
        for row in range(3):
            self.labels[(row, 1)].setText(tags[row])

class TimelineWidget(QWidget):
    phase_hovered_signal = pyqtSignal(str, int)

    def __init__(self, timeline):
        super().__init__()

        self.timeline = timeline
        self.rectangle_rounding = [10, 10]
        self.numbers_visible = True
        self.phase_border_color = Qt.black
        self.phase_patch_transparency = 200  # max 255
        self.size_numbers = 8
        self.phases_path = []

        self.setMouseTracking(True)

        self.last_position = 0
        self.last_duration = 0

        self.phase_hovered_i = -1

    def __is_mouse_inside_rect(self, pos):
        for i in range(len(self.phases_path)):
            if self.phases_path[i].contains(pos):
                return i

        return -1

    def enterEvent(self, event):
        self.phase_hovered_i = self.__is_mouse_inside_rect(event.pos())
        self.phase_hovered_signal.emit(self.timeline.name, self.phase_hovered_i)
        # if self.phase_hovered_i > 0:
            # self.update()

    def leaveEvent(self, event):
        self.phase_hovered_i = -1
        self.phase_hovered_signal.emit(self.timeline.name, self.phase_hovered_i)
        # self.update()

    def mouseMoveEvent(self, event):
        self.phase_hovered_i = self.__is_mouse_inside_rect(event.pos())
        self.phase_hovered_signal.emit(self.timeline.name, self.phase_hovered_i)
        # self.update()

    def __nodes_to_lenght(self, nodes):
        return int(nodes / self.timeline.n_nodes * self.width())

    def __draw_phase_patch(self, path: QPainterPath, x_position, y_position, width, height):

        corner_radius_0 = min(width*2,  self.height() * 0.5, self.width() * 0.5, 60)
        corner_radius_1 = min(width/2,  self.height() * 0.2, self.width() * 0.05, 30)

        # corner_radius_0 = min(self.height() * 0.5,  self.width() * 0.1)
        # corner_radius_1 = min(self.height() * 0.2,  self.width() * 0.05)
        path.moveTo(x_position, y_position + corner_radius_1 / 2)
        # path.lineTo(x_position, y_position + height - corner_radius_0)
        path.arcTo(x_position, y_position + height - corner_radius_0, corner_radius_0, corner_radius_0, 180.0, 90.0)
        path.lineTo(x_position + width, y_position + height)
        path.arcTo(x_position + width - corner_radius_1, y_position, corner_radius_1, corner_radius_1, 0, 90.0)
        path.lineTo(x_position + corner_radius_0 / 2, y_position)
        path.arcTo(x_position, y_position, corner_radius_1, corner_radius_1, 90.0, 90.0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        timeline_height = self.height()

        # Draw grid lines
        painter.setPen(QPen(Qt.gray))

        font = QFont()
        font.setPointSize(self.size_numbers)
        painter.setFont(font)

        for i in range(self.timeline.n_nodes):
            x = self.__nodes_to_lenght(i)
            painter.drawLine(x, 0, x, timeline_height)
            if self.numbers_visible:
                painter.drawText(x, timeline_height, str(i))

        self.phases_path.clear()


        painter.setPen(QPen(self.phase_border_color, 0.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for phase_i in range(len(self.timeline.phases)):
            phase = self.timeline.phases[phase_i]
            # print(f'drawing phase: {phase.name} (duration: {phase.duration}, initial node: {phase.initial_node})')
            name = phase.name
            position = phase.initial_node
            position_error = phase.position_error

            brush_pattern = Qt.SolidPattern  # Use CrossPattern as an example pattern
            color = QColor(*string_to_color(name))
            color.setAlpha(self.phase_patch_transparency)


            # if phase_i > 0:
            #     if position < self.timeline.phases[phase_i - 1].initial_node + self.timeline.phases[phase_i - 1].duration:
            if position_error > 0:
                    # color = QColor(Qt.red)
                    brush_pattern = Qt.BDiagPattern

            phase_path = QPainterPath()
            self.__draw_phase_patch(phase_path, self.__nodes_to_lenght(position), 0, self.__nodes_to_lenght(phase.duration), timeline_height)
            # rect = QRectF(self.__nodes_to_lenght(position), 0, self.__nodes_to_lenght(phase.duration), timeline_height)
            # phase_path.addRoundedRect(rect, self.rectangle_rounding[0], self.rectangle_rounding[1])
            self.phases_path.append(phase_path)

            brush = QBrush(brush_pattern)
            brush.setColor(color)
            painter.setBrush(brush)
            painter.drawPath(phase_path)

        # print('=========')

        # check if mouse is inside a phase
        self.phase_hovered_i = self.__is_mouse_inside_rect(self.mapFromGlobal(QCursor.pos()))
        if self.phase_hovered_i > 0:
            self.phase_hovered_signal.emit(self.timeline.name, self.phase_hovered_i)

        # if self.phase_hovered_i is not None:
        #     font = QFont()
        #     font.setPointSize(10)
        #     painter.setFont(font)
        #     painter.setPen(QPen(Qt.black))
        #
        #     phase_hovered = self.timeline.phases[self.phase_hovered_i]
        #     name = phase_hovered.name
        #     metrics = QFontMetrics(font)
        #     text_width = metrics.width(name)
        #     x = self.__nodes_to_lenght(phase_hovered.initial_node)
        #     width = self.__nodes_to_lenght(phase_hovered.duration)
        #     y = timeline_height
        #     painter.drawText(x + width/2 - text_width, y/2, name)



class TimelinesWidget(QWidget):
    def __init__(self, timeline_gui: TimelineROS, n_nodes):
        super().__init__()

        self.timeline_gui = timeline_gui
        self.n_nodes = n_nodes
        self.timelines = dict()
        self.timeline_widgets = []

        self.timeline_name_visible = True
        self.phase_info_box_visible = True
        self.log_box_visible = True

        self.log_box_widget = None

        self.__init_UI()
        # self.__init_notification_display()
        self.__init_phase_info_box()
        self.__init_timeline_box()
        self.__init_timelines()
        self.__add_timeline_name_widget()
        self.__add_timeline_widget()

        for timeline_widget in self.timeline_widgets:
            timeline_widget.phase_hovered_signal.connect(self.__update_display)

    def __init_UI(self):

        self.vbox = QVBoxLayout(self)
        # change only left and right
        left, top, right, bottom = self.vbox.getContentsMargins()
        self.vbox.setContentsMargins(left, top, 0, bottom)

    def __init_timeline_box(self):

        self.timelines_box = QWidget()
        self.hbox = QHBoxLayout()
        self.hbox.setContentsMargins(0, 0, 0, 0)
        self.timelines_box.setLayout(self.hbox)
        self.vbox.addWidget(self.timelines_box, 10)

    def __init_phase_info_box(self):

        self.none_text = ['--', '--', '--']
        self.phase_info_box_widget = PhaseInfoBox(self.none_text)
        self.vbox.addWidget(self.phase_info_box_widget, 1)

    def __init_notification_display(self):

        self.log_box_widget = QTextEdit()
        self.log_box_widget.setReadOnly(True)
        self.log_box_widget.setStyleSheet("color: red; background-color: black;")
        self.vbox.addWidget(self.log_box_widget, 1)

    def __overlap_phase_error_message(self, timeline_name, phase):

        txt = f"Timeline {timeline_name}: phase {phase.name} starting at node {phase.initial_node} overlaps with previous phase of {phase.position_error} nodes"
        self.log_box_widget.append(txt)

    def __update_display(self, timeline_name, phase_i):

        if phase_i >= 0:
            phase = self.timelines[timeline_name].phases[phase_i]
            info_text = [phase.name, str(phase.initial_node), str(phase.duration)]

        else:
            info_text = self.none_text

        self.phase_info_box_widget.setText(info_text)

    def __init_timelines(self):

        for timeline_name in self.timeline_gui.timelines.keys():
            self.timelines[timeline_name] = TimelineInfo(timeline_name, self.n_nodes)

    def __add_timeline_name_widget(self):

        self.label_layout = QVBoxLayout()
        self.label_layout.setContentsMargins(0, 0, 0, 0)

        for name, timeline in self.timelines.items():
            label = ElideLabel(name)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            font = QFont()
            font.setBold(True)
            label.setFont(font)
            label.setVisible(self.timeline_name_visible)  # Set label visibility
            self.label_layout.addWidget(label)

        self.label_box_widget = QWidget()
        self.label_box_widget.setLayout(self.label_layout)

        self.hbox.addWidget(self.label_box_widget)
        self.hbox.setStretchFactor(self.label_box_widget, 1)

    def __add_timeline_widget(self):

        self.timeline_layout = QVBoxLayout()
        self.timeline_layout.setContentsMargins(0, 0, 0, 0)

        for name, timeline in self.timelines.items():
            timeline_widget = TimelineWidget(timeline)
            self.timeline_widgets.append(timeline_widget)
            self.timeline_layout.addWidget(timeline_widget)

        self.timeline_box_widget = QWidget()
        self.timeline_box_widget.setLayout(self.timeline_layout)

        self.hbox.addWidget(self.timeline_box_widget)
        self.hbox.setStretchFactor(self.timeline_box_widget, 15)

    def toggle_labels(self):
        self.timeline_name_visible = not self.timeline_name_visible
        self.label_box_widget.setVisible(self.timeline_name_visible)

    def toggle_node_number(self):
        for i in range(self.timeline_layout.count()):
            self.timeline_layout.itemAt(i).widget().numbers_visible = not self.timeline_layout.itemAt(i).widget().numbers_visible
            self.timeline_layout.itemAt(i).widget().update()

    def toggle_phase_info_box(self):
        self.phase_info_box_visible = not self.phase_info_box_visible
        self.phase_info_box_widget.setVisible(self.phase_info_box_visible)

    def toggle_notification_box(self):
        self.log_box_visible = not self.log_box_visible
        self.log_box_widget.setVisible(self.log_box_visible)

    def update_timeline(self):

        for timeline_name, phases in self.timeline_gui.timelines.items():
            self.timelines[timeline_name].update(phases)

            if self.log_box_widget:
                for phase in phases:
                    if phase.position_error > 0:
                        self.__overlap_phase_error_message(timeline_name, phase)


        self.update()
