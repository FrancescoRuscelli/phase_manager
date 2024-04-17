from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 50
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
b = prb.createStateVariable('b', 1)

cnsrt1 = prb.createConstraint('cnsrt1', a, nodes=[])
cnsrt2 = prb.createConstraint('cnsrt2', 2 * b, nodes=[])
pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')

phase_1 = pyphase.Phase(1, 'phase_1')
phase_2 = pyphase.Phase(5, 'phase_2')

phase_1.addItem(cnsrt1)
phase_2.addItem(cnsrt2)

timeline_1.registerPhase(phase_1)
timeline_1.registerPhase(phase_2)


timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)


print("inserting phase: ")
timeline_1.addPhase(phase_2, 5)
# print("empty nodes: ", timelineactive_phase_1.getEmptyNodes())
# timeline_1.addPhase(phase_2)
# timeline_1.addPhase(phase_2, 0)
# timeline_1.addPhase(phase_2)
# timeline_1.addPhase(phase_2)
# timeline_1.addPhase(phase_1)

pos_phase = 0
for phase in timeline_1.getPhases():
    print(f"{pos_phase}. {phase.getName()}: ")
    print(f"     initial position: {phase.getPosition()}")
    print(f"     active nodes: {phase.getActiveNodes()}")
    pos_phase += 1
#
# print("=======================================")
# for name_c, c in prb.getConstraints().items():
#     print(name_c, ":  ", c.getNodes())