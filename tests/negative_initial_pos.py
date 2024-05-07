from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import phase_manager.pytimeline as pytimeline
from colorama import Fore, Back, Style
import numpy as np

def print_timeline_phases():
    print(Fore.RED + "phases: " + Style.RESET_ALL)
    for phase in timeline_1.getPhases():
        print(
            Style.BRIGHT + f"   {phase.getName()} " + Style.RESET_ALL + f"at node " + Style.BRIGHT + f"{phase.getPosition()}" + Style.RESET_ALL + f": {phase.getActiveNodes()}")


ns = 20
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
b = prb.createStateVariable('b', 1)

cnsrt1 = prb.createConstraint('cnsrt1', a, nodes=[])
cnsrt2 = prb.createConstraint('cnsrt2', 2 * b, nodes=[])
cnsrt3 = prb.createConstraint('cnsrt3', 3 * a, nodes=[])
cnsrt4 = prb.createConstraint('cnsrt4', 3 * a * b, nodes=[])

pm = pymanager.PhaseManager(ns)
timeline_1 = pm.createTimeline('timeline_1')

phase_1n = timeline_1.createPhase(1, 'phase_1n')
phase_1n.addItem(cnsrt1)
phase_1n.addItem(cnsrt2)

phase_5n = timeline_1.createPhase(5, 'phase_5n')
phase_5n.addItem(cnsrt3)
phase_5n.addItem(cnsrt4)

for i in range(1):
    timeline_1.addPhase(phase_5n)

print_timeline_phases()

timeline_1.addPhase(phase_1n, 1, absolute_position=True)
timeline_1.addPhase(phase_1n, 2, absolute_position=True)
timeline_1.addPhase(phase_1n, 3, absolute_position=True)
timeline_1.addPhase(phase_1n, 4, absolute_position=True)

# for i in range(1):
# timeline_1.addPhase(phase_5n, 10, absolute_position=True)
print("py-insert phase")
timeline_1.addPhase(phase_1n, 7, absolute_position=True)
timeline_1.addPhase(phase_1n, 8, absolute_position=True)
timeline_1.addPhase(phase_1n, 9, absolute_position=True)
timeline_1.addPhase(phase_1n, 10, absolute_position=True)
timeline_1.addPhase(phase_5n, 11, absolute_position=True)
print_timeline_phases()

exit()
print(" initial condition: ")

print("cnsrt1: ", cnsrt1.getNodes())
print("cnsrt2: ", cnsrt2.getNodes())
print("cnsrt3: ", cnsrt3.getNodes())
print("cnsrt4: ", cnsrt4.getNodes())
#
num_shift = 100
for i in range(num_shift):
    print(f" SHIFTING PHASES: iteration {i}")

    pm.shift()
    print("phases: ")
    for phase in timeline_1.getPhases():
        print(f"{phase.getName()} at node {phase.getPosition()}: {phase.getActiveNodes()}")

    print("horizon elements: ")
    print("cnsrt1: ", cnsrt1.getNodes())
    print("cnsrt2: ", cnsrt2.getNodes())
    print("cnsrt3: ", cnsrt3.getNodes())
    print("cnsrt4: ", cnsrt4.getNodes())
    input()
#
# exit()
#