import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QHBoxLayout, QMenuBar, \
    QAction
from PyQt5.QtGui import QPainter, QColor, QBrush, QPainterPath, QPen, QFont
from PyQt5.QtCore import Qt, QTimer, QRectF
import random
import hashlib
from phase_manager.msg import Timeline, TimelineArray
import rospy


def string_to_color(input_string, seed=None):
    if seed is not None:
        random.seed(seed)
    hash_value = int(hashlib.sha256(input_string.encode()).hexdigest(), 16)
    r = (hash_value & 0xFF0000) >> 16
    g = (hash_value & 0x00FF00) >> 8
    b = hash_value & 0x0000FF
    a = 255
    return (r, g, b, a)


class TimelineROS():

    def __init__(self):

        rospy.init_node('timeline_visualizer')
        self.topic_name = '/phase_manager/timelines'

        self.timelines = dict()

        self.__init_ros_callback()

    def __init_ros_callback(self):

        rospy.Subscriber(self.topic_name, TimelineArray, self.timeline_callback)
        # rospy.wait_for_message(self.topic_name, TimelineArray, timeout=1)

    def timeline_callback(self, msg: TimelineArray):

        for timeline in msg.timelines:
            self.timelines[timeline.name] = []
            for phase_name, initial_node, duration in zip(timeline.phases, timeline.initial_nodes, timeline.durations):
                phase_to_add = PhaseInfo(phase_name, initial_node, duration)
                self.timelines[timeline.name].append(phase_to_add)


class PhaseInfo:
    def __init__(self, name, initial_node, duration):
        self.name = name
        self.initial_node = initial_node
        self.duration = duration


class TimelineInfo:
    def __init__(self, name, n_nodes):
        self.name = name
        self.n_nodes = n_nodes
        self.phases = []

    def update(self, phases):
        # Rotate phases
        self.phases = phases


class TimelineWidget(QWidget):
    def __init__(self, timeline):
        super().__init__()
        self.timeline = timeline
        self.rectangle_rounding = [5, 5]

    def __nodes_to_lenght(self, nodes):
        return int(nodes / self.timeline.n_nodes * self.width())

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        timeline_height = self.height()

        # Draw grid lines
        painter.setPen(QPen(Qt.gray))
        for i in range(self.timeline.n_nodes):
            x = self.__nodes_to_lenght(i)
            painter.drawLine(x, 0, x, timeline_height)

        painter.setPen(QPen(QColor(0, 0, 0, 255), 1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))

        for phase in self.timeline.phases:
            print(f'drawing phase: {phase.name} (duration: {phase.duration}, initial node: {phase.initial_node})')
            name = phase.name
            position = phase.initial_node
            color = QColor(*string_to_color(name))

            rectPath = QPainterPath()
            rect = QRectF(self.__nodes_to_lenght(position), 0, self.__nodes_to_lenght(phase.duration), timeline_height)
            rectPath.addRoundedRect(rect, self.rectangle_rounding[0], self.rectangle_rounding[1])

            painter.setBrush(color)
            painter.drawPath(rectPath)

        print('=========')


class TimelinesWidget(QWidget):
    def __init__(self, timeline_gui: TimelineROS, n_nodes):
        super().__init__()

        self.timeline_gui = timeline_gui
        self.n_nodes = n_nodes
        self.timelines = dict()

        self.labels_visible = True

        self.__init_UI()
        self.__init_timelines()
        self.__add_timeline_widget()

    def __init_UI(self):

        self.vbox = QVBoxLayout(self)

    def __init_timelines(self):

        for timeline_name in self.timeline_gui.timelines.keys():
            self.timelines[timeline_name] = TimelineInfo(timeline_name, self.n_nodes)

    def __add_timeline_widget(self):
        for name, timeline in self.timelines.items():
            timeline_widget = TimelineWidget(timeline)
            hbox = QHBoxLayout()
            label = QLabel(name)
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            font = QFont()
            font.setBold(True)
            label.setFont(font)
            label.setVisible(self.labels_visible)  # Set label visibility
            hbox.addWidget(label)
            hbox.addWidget(timeline_widget)
            hbox.setStretchFactor(label, 1)
            hbox.setStretchFactor(timeline_widget, 15)
            self.vbox.addLayout(hbox)

    def __toggle_labels(self):
        self.labels_visible = not self.labels_visible
        for i in range(self.vbox.count()):
            layout_item = self.vbox.itemAt(i)
            if layout_item and isinstance(layout_item, QHBoxLayout):
                label_widget = layout_item.itemAt(0).widget()
                if label_widget and isinstance(label_widget, QLabel):
                    label_widget.setVisible(self.labels_visible)

    def update_timeline(self):

        for timeline_name, phases in self.timeline_gui.timelines.items():
            self.timelines[timeline_name].update(phases)

        self.update()


class PhaseManagerGUI(QMainWindow):
    def __init__(self, timeline_ros: TimelineROS, n_nodes):
        super().__init__()

        self.timeline_ros = timeline_ros
        self.timelines_widget = TimelinesWidget(timeline_ros, n_nodes)

        self.initUI()

    def initUI(self):
        menubar = self.menuBar()
        # font = menubar.font()
        # font.setPointSize(8)  # Set smaller font size for menu items
        # menubar.setFont(font)
        # menubar.setFixedHeight(19)

        # self.label_action = QAction('Show Labels', self)
        # self.label_action.setCheckable(True)
        # self.label_action.setChecked(True)
        # self.label_action.triggered.connect(self.__toggle_labels)
        # view_menu.addAction(self.label_action)

        # Create a central widget to hold other widgets
        self.setCentralWidget(self.timelines_widget)

        # Create a layout for the central widget

        # Add a label below the menu

        self.setWindowTitle('Timeline GUI')
        self.setGeometry(100, 100, 600, 400)

    def update(self):
        self.timelines_widget.update_timeline()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # ----------------- artificial phases ----------------

    num_phases = 2
    phase_durations = [12, 15, 3, 7, 22, 11]
    phase_names = ['phase_q', 'phase1', 'verylongname_phase_with_some_other_stuff_written', 'something_else']
    timeline_names = ['a', 's', 'ball_1', 'd']

    timelines = dict()

    for name in timeline_names:
        phases = []
        phase_position = 0
        for i in range(num_phases):
            phase_duration = random.choice(phase_durations)
            phases.append(PhaseInfo(random.choice(phase_names), phase_position, phase_duration))
            phase_position += phase_duration

        timelines[name] = phases

    timeline_ros = TimelineROS()
    timeline_ros.timelines = timelines
    gui = PhaseManagerGUI(timeline_ros, 60)
    gui.update()

    # timer = QTimer()
    # timer.timeout.connect(lambda: gui.update_timeline(timelines))
    # timer.start(10)  # Update every second

    gui.show()
    sys.exit(app.exec_())