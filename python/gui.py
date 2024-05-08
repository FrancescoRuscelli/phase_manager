import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QHBoxLayout, QMenuBar, QPushButton, QAction, QStatusBar
from PyQt5.QtGui import QPainter, QColor, QBrush, QPainterPath, QPen, QFont
from PyQt5.QtCore import Qt, QTimer, QRectF

import rospy
import random
from timelines_widget import TimelinesWidget
from ros_client_class import TimelineROS, PhaseInfo, TimelineInfo

class PhaseManagerGUI(QMainWindow):
    def __init__(self, timeline_ros: TimelineROS, n_nodes):
        super().__init__()

        self.timeline_ros = timeline_ros
        self.timelines_widget = TimelinesWidget(timeline_ros, n_nodes)
        self.running = True

        self.__init_UI()
        self.__init_menu()
        self.__init_timeline_widget()
        self.__init_run_button()
        self.__init_status_bar()

    def __init_UI(self):
        self.setWindowTitle('Timeline GUI')
        self.setGeometry(100, 100, 800, 600)

        central_widget = QWidget()
        self.layout = QVBoxLayout(central_widget)
        self.setCentralWidget(central_widget)

    def __init_timeline_widget(self):

        self.layout.addWidget(self.timelines_widget)
        self.layout.setContentsMargins(0, 0, 0, 0)

    def __init_run_button(self):

        # Create button and connect its clicked signal
        self.button = QPushButton("Stop")
        self.button.clicked.connect(self.toggle_button)

        # Add button to layout
        self.layout.addWidget(self.button)

    def __activate_status_bar(self, flag):

        if flag:
            self.status_bar.setStyleSheet("background-color: green")
            # self.status_label.setText("running")
        else:
            self.status_bar.setStyleSheet("background-color: red")
            # self.status_label.setText("stopped")

    def __init_status_bar(self):

        # Create status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

        # self.status_label = QLabel()
        # self.status_label.setAlignment(Qt.AlignCenter)
        # self.status_bar.addWidget(self.status_label, 1)
        self.__activate_status_bar(True)


    def __init_menu(self):

        menubar = self.menuBar()
        font = menubar.font()
        font.setPointSize(8)  # Set smaller font size for menu items
        menubar.setFont(font)
        self.view_menu = menubar.addMenu('View')

        # toggle labels
        self.label_action = QAction('Show Labels', self)
        self.label_action.setCheckable(True)
        self.label_action.setChecked(True)
        self.label_action.triggered.connect(self.timelines_widget.toggle_labels)
        self.view_menu.addAction(self.label_action)

        # toggle numbers
        self.node_number_action = QAction('Show Nodes', self)
        self.node_number_action.setCheckable(True)
        self.node_number_action.setChecked(True)
        self.node_number_action.triggered.connect(self.timelines_widget.toggle_node_number)
        self.view_menu.addAction(self.node_number_action)

        # toggle phase info box
        self.phase_info_box_action = QAction('Show Phase Info', self)
        self.phase_info_box_action.setCheckable(True)
        self.phase_info_box_action.setChecked(True)
        self.phase_info_box_action.triggered.connect(self.timelines_widget.toggle_phase_info_box)
        self.view_menu.addAction(self.phase_info_box_action)

        # toggle notification box
        # self.log_box_action = QAction('Show Notification Display', self)
        # self.log_box_action.setCheckable(True)
        # self.log_box_action.setChecked(True)
        # self.log_box_action.triggered.connect(self.timelines_widget.toggle_notification_box)
        # self.view_menu.addAction(self.log_box_action)

    def toggle_button(self):
        if self.button.text() == "Run":
            self.button.setText("Stop")
            self.__activate_status_bar(True)
            self.running = True
        else:
            self.button.setText("Run")
            self.__activate_status_bar(False)
            self.running = False

    def update(self):
        if self.running:
            self.timelines_widget.update_timeline()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # ----------------- artificial phases ----------------

    # num_phases = 10
    # phase_durations = [1, 3, 15, 3, 7, 22, 11]
    # phase_names = ['phase_q', 'phase1', 'verylongname_phase_with_some_other_stuff_written', 'something_else']
    # timeline_names = ['a', 's', 'ball_1', 'd']
    #
    # timelines = dict()
    #
    # for name in timeline_names:
    #     phases = []
    #     phase_position = 0
    #     for i in range(num_phases):
    #         phase_duration = random.choice(phase_durations)
    #         phases.append(PhaseInfo(random.choice(phase_names), phase_position, phase_duration))
    #         phase_position += phase_duration
    #
    #     timelines[name] = phases
    #
    # timelines['a'][2].initial_node = timelines['a'][2].initial_node - 2
    #
    # timeline_ros = TimelineROS()
    # timeline_ros.timelines = timelines
    # gui = PhaseManagerGUI(timeline_ros, 50)
    # gui.update()
    #
    # gui.show()
    # sys.exit(app.exec_())
    # ============================================================
    #
    timeline_ros = TimelineROS()
    gui = PhaseManagerGUI(timeline_ros, 50)

    timer = QTimer()
    timer.timeout.connect(gui.update)
    timer.start(10)

    gui.show()
    sys.exit(app.exec_())
