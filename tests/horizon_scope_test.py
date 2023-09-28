from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 10
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
cnsrt_1 = prb.createConstraint('cnsrt_1', a / 2, nodes=[])
par = prb.createParameter('par', 1)
print(par.getValues())
pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')

phase_1 = pyphase.Phase(4, 'phase_1')

# phase_1.addConstraint(cnsrt_1, nodes=[0, 3])
# phase_1.addVariableBounds(a, np.array([[-1, -1]]), np.array([[1, 1]]), [2, 3])
phase_1.addVariableBounds(a, np.array([[-1, -1, -1, -1]]), np.array([[1, 1, 1, 1]]))
# phase_1.addParameterValues(par, np.array([[1., 2., 3., 4.]]))
timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)
# timeline_1.shift_phases()
timeline_1.shift_phases()
timeline_1.shift_phases()
# timeline_1.shift_phases()

print(a.getBounds())
# print(par.getValues())
# print(phase_1.getConstraintsInfo())
# phase_1.setDuration(15)
# print(phase_1.getConstraintsInfo())

# phase_1.addVariableBounds(a, np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]),
#                              np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]))

# phase_1.addVariableBounds(a, np.array([[0, 0, 0, 0, 0, 0]]),
#                              np.array([[0, 0, 0, 0, 0, 0]]),
#                              nodes=[0, 1, 2, 3, 4, 5])

# print('constraints and active nodes:')
# for elem in phase_1.getConstraints():
#     print(f"{elem.getName()}: {elem.getNodes()}")

print('variables, active nodes and bounds:')
for elem in phase_1.getVariables():
    print(f"{elem.getName()}: {elem.getNodes()}, \n{elem.getBounds()}")
exit()

# phase_1.setElementNodes("cnsrt_1", [0, 1])
phase_1.setElementNodes("a", [2, 3], np.array([[2, 4]]))

timeline_1.addPhase(phase_1)

# print('constraints and active nodes:')
# for elem in phase_1.getConstraints():
#     print(f"{elem.getName()}: {elem.getNodes()}")

print('variables, active nodes and bounds:')
for elem in phase_1.getVariables():
    print(f"{elem.getName()}: {elem.getNodes()}, \n{elem.getBounds()}")


exit()
# print(phase_1.getCosts())
# print(phase_1.getParameters())
# print(phase_1.getVariables())
# print(phase_1.getItems())



timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)
timeline_1.addPhase(phase_1)

print('constraints and active nodes:')
for elem in phase_1.getConstraints():
    print(f"{elem.getName()}: {elem.getNodes()}")

timeline_1.addPhase(phase_1)
exit()

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
for i in range(31):
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
















