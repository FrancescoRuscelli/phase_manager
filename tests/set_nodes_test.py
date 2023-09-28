from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import trajectoryGenerator, utils
from horizon.ros import replay_trajectory
import horizon.utils.analyzer as analyzer

import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 15
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)



pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')

phase_1 = pyphase.Phase(5, 'phase_1')

phase_1.addVariableBounds(a, np.array([[0, 0, 0, 0, 0]]), np.array([[0, 0, 0, 0, 0]]))
timeline_1.registerPhase(phase_1)

# adding phases to timeline(
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)

phase_1.setDuration(10)
print(phase_1.getNNodes())
exit()
print(phase_1.getVariablesInfo())
phase_1.setElementNodes()
for phase in timeline_1.getPhases():
    print(phase.getName())


# print(a.getLowerBounds())
# pm._shift_phases()
#
# print(a.getLowerBounds())
