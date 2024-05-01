import rospy
from phase_manager.msg import Timeline, TimelineArray
class TimelineROS:

    def __init__(self):

        rospy.init_node('timeline_visualizer')
        self.topic_name = '/phase_manager/timelines'

        self.timelines = dict()

        self.__init_ros_callback()

    def __init_ros_callback(self):

        rospy.Subscriber(self.topic_name, TimelineArray, self.timeline_callback)
        rospy.wait_for_message(self.topic_name, TimelineArray, timeout=1)

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