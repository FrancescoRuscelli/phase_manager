from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import time
import numpy as np

ns = 15
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
par = prb.createParameter('par', 1)

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')
phase_1 = pyphase.Phase(5, 'phase_1')
phase_1.addVariableBounds(a, np.array([[1, 2, 3, 4, 5]]), np.array([[-1, -2, -3, -4, -5]]))
phase_1.addParameterValues(par, np.array([[1., 2., 3.]]), [2, 3, 4])
timeline_1.registerPhase(phase_1)

# tic = time.time()
# print(time.time() - tic)

# adding phases to timeline
# timeline_1.addPhase(phase_1)

# print('variable before:')
# print(phase_1.getVariablesInfo())
# print(a.getUpperBounds())

phase_1.setDuration(10)
# print('new nodes: ', phase_1.getNNodes())

# print('variable after:')
# print(phase_1.getVariablesInfo())
# print(a.getUpperBounds())

# phase_1.setElementNodes('a', [0, 1, 2], np.array([[-5, -4, -3]]), np.array([[5, 4, 3]]))
# phase_1.setElementNodes('a', [1, 2], np.array([[-4, -3]]), np.array([[4, 3]]))
# phase_1.setElementNodes('a', [1, 2], np.array([[-4, -3]]))
phase_1.setElementNodes(par.getName(), [1, 2], np.array([[-4, -3]]))

timeline_1.addPhase(phase_1)

print(par.getValues())
print(a.getBounds())
# print(a.getLowerBounds())
# print(a.getUpperBounds())
exit()





# for phase in timeline_1.getPhases():
#     print(phase.getName())
