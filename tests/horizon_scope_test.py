from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 10
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')
phase_1 = pyphase.Phase(4, 'phase_1')



phase_1.addVariableBounds(a, np.array([[0, 0, 0, 0, 0]]), np.array([[0, 0, 0, 0, 0]]))
timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)

print("======= ACTIVE PHASES: =======")
for elem in timeline_1.getActivePhases():
    print(f"{elem.getName()}: {elem}.")
    print(f"   Phase position: {elem.getPosition()}")
    print(f"   Phase duration: {elem.getNNodes()}")
    print(f"   Active nodes: {elem.getActiveNodes()}")
    print(f"   =====================================")

print("======= ALL PHASES: ========")
for elem in timeline_1.getPhases():
    print(f"{elem.getName()}: {elem}.")
    print(f"   Phase position: {elem.getPosition()}")
    print(f"   Phase duration: {elem.getNNodes()}")
    print(f"   Active nodes: {elem.getActiveNodes()}")
    print(f"   =====================================")

print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
for i in range(30):
    timeline_1.shift_phases()
    print("SHIFTING PHASES: ")

    # print("======= ACTIVE PHASES: =======")
    # for elem in timeline_1.getActivePhases():
    #     print(f"{elem.getName()}: {elem}.")
    #     print(f"   Phase position: {elem.getPosition()}")
    #     print(f"   Phase duration: {elem.getNNodes()}")
    #     print(f"   Active nodes: {elem.getActiveNodes()}")
    #     print(f"   =====================================")

    print("======= ALL PHASES: ========")
    for elem in timeline_1.getPhases():
        print(f"{elem.getName()}: {elem}.")
        print(f"   Phase position: {elem.getPosition()}")
        print(f"   Phase duration: {elem.getNNodes()}")
        print(f"   Active nodes: {elem.getActiveNodes()}")
        print(f"   =====================================")
exit()
