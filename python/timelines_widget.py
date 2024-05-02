import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QHBoxLayout, QMenuBar, \
    QAction
from PyQt5.QtGui import QPainter, QColor, QBrush, QPainterPath, QPen, QFont, QFontMetrics
from PyQt5.QtCore import Qt, QTimer, QRectF, pyqtSignal
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
            label = QLabel(text)

            self.grid_layout.addWidget(label, row, col)
            self.labels[(row, col)] = label

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

        corner_radius_0 = 60
        corner_radius_1 = 30
        path.moveTo(x_position, y_position + corner_radius_1/2)
        path.lineTo(x_position, y_position + height - corner_radius_0)
        path.arcTo(x_position, y_position + height - corner_radius_0, min(corner_radius_0, width), corner_radius_0, 180.0, 90.0)
        path.lineTo(x_position + width, y_position + height)

        path.arcTo(x_position + width - corner_radius_1, y_position, min(corner_radius_1, width), corner_radius_1, 0, 90.0)

        path.lineTo(x_position + corner_radius_0/2, y_position)
        path.arcTo(x_position, y_position, min(corner_radius_1, width), corner_radius_1, 90.0, 90.0)

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
        for phase in self.timeline.phases:
            # print(f'drawing phase: {phase.name} (duration: {phase.duration}, initial node: {phase.initial_node})')
            name = phase.name
            position = phase.initial_node
            color = QColor(*string_to_color(name))
            color.setAlpha(self.phase_patch_transparency)

            phase_path = QPainterPath()
            self.__draw_phase_patch(phase_path, self.__nodes_to_lenght(position), 0, self.__nodes_to_lenght(phase.duration), timeline_height)
            # rect = QRectF(self.__nodes_to_lenght(position), 0, self.__nodes_to_lenght(phase.duration), timeline_height)
            # phase_path.addRoundedRect(rect, self.rectangle_rounding[0], self.rectangle_rounding[1])
            self.phases_path.append(phase_path)

            painter.setBrush(color)
            painter.drawPath(phase_path)

        # print('=========')

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
        self.display_visible = True

        self.__init_UI()
        self.__init_display()
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

    def __init_display(self):


        self.none_text = ['--', '--', '--']
        self.display_widget = PhaseInfoBox(self.none_text)
        self.vbox.addWidget(self.display_widget, 1)


    def __update_display(self, timeline_name, phase_i):

        if phase_i >= 0:
            phase = self.timelines[timeline_name].phases[phase_i]
            info_text = [phase.name, str(phase.initial_node), str(phase.duration)]

        else:
            info_text = self.none_text

        self.display_widget.setText(info_text)

    def __init_timelines(self):

        for timeline_name in self.timeline_gui.timelines.keys():
            self.timelines[timeline_name] = TimelineInfo(timeline_name, self.n_nodes)

    def __add_timeline_name_widget(self):

        self.label_layout = QVBoxLayout()
        self.label_layout.setContentsMargins(0, 0, 0, 0)

        for name, timeline in self.timelines.items():
            label = QLabel(name)
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
        self.display_visible = not self.display_visible
        self.display_widget.setVisible(self.display_visible)

    def update_timeline(self):

        for timeline_name, phases in self.timeline_gui.timelines.items():
            self.timelines[timeline_name].update(phases)

        self.update()
