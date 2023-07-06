from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 20
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')
phase_1 = pyphase.Phase(5, 'phase_1')



phase_1.addVariableBounds(a, np.array([[0, 0, 0, 0, 0]]), np.array([[0, 0, 0, 0, 0]]))
timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)

for elem in timeline_1.getActivePhases():
    print(f"{elem.getName()}: {elem}. Active nodes: {elem.getNodes()}")

exit()
