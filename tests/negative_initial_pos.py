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


def print_items():
    print("cnsrt1: ", cnsrt1.getNodes())
    print("cnsrt2: ", cnsrt2.getNodes())
    print("cnsrt3: ", cnsrt3.getNodes())
    print("cnsrt4: ", cnsrt4.getNodes())

ns = 15
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

a = prb.createStateVariable('a', 1)
b = prb.createStateVariable('b', 1)

cnsrt1 = prb.createConstraint('cnsrt1', a, nodes=[])
cnsrt2 = prb.createConstraint('cnsrt2', 2 * b, nodes=[])
cnsrt3 = prb.createConstraint('cnsrt3', 3 * a, nodes=[])
cnsrt4 = prb.createConstraint('cnsrt4', 3 * a * b, nodes=[])

pm = pymanager.PhaseManager(ns, True)
timeline_1 = pm.createTimeline('timeline_1')

phase_1n = timeline_1.createPhase(1, 'phase_1n')
phase_1n.addItem(cnsrt1)
phase_1n.addItem(cnsrt2)

phase_5n = timeline_1.createPhase(5, 'phase_5n')
phase_5n.addItem(cnsrt3)
phase_5n.addItem(cnsrt4)

phase_8n = timeline_1.createPhase(8, 'phase_8n')
phase_8n.addItem(cnsrt3)
phase_8n.addItem(cnsrt4)

phase_30n = timeline_1.createPhase(30, 'phase_30n')
phase_30n.addItem(cnsrt1)
phase_30n.addItem(cnsrt4)

phase_29n = timeline_1.createPhase(29, 'phase_29n')
phase_29n.addItem(cnsrt1)
phase_29n.addItem(cnsrt4)

phase_15n = timeline_1.createPhase(15, 'phase_15n')
phase_15n.addItem(cnsrt1)
phase_15n.addItem(cnsrt4)
# ============================================================================
#test 1
# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_5n)
# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_5n)
# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_5n)
# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_1n)

# ============================================================================
#test 2
# timeline_1.addPhase(phase_29n)
# timeline_1.addPhase(phase_1n)
# timeline_1.addPhase(phase_1n)

# ============================================================================
#test 3
# timeline_1.addPhase(phase_15n)
# timeline_1.addPhase(phase_30n)

# ============================================================================
#test 4
# timeline_1.addPhase(phase_8n, pos=28, absolute_position=True)

# ============================================================================
# test 5
# timeline_1.addPhase(phase_15n)
# timeline_1.addPhase(phase_8n, pos=0)
# timeline_1.addPhase(phase_5n, pos=0)
# timeline_1.addPhase(phase_15n, pos=0)
# timeline_1.addPhase(phase_15n)

# ============================================================================
# test 6
# timeline_1.addPhase(phase_15n)
# timeline_1.addPhase(phase_8n, pos=0)
# timeline_1.addPhase(phase_1n, pos=1)
# timeline_1.addPhase(phase_5n, pos=3)
# timeline_1.addPhase(phase_15n)

# ============================================================================
# test 7
# timeline_1.addPhase(phase_15n)
# timeline_1.addPhase(phase_8n, 20, absolute_position=True)
# timeline_1.addPhase(phase_1n, 22, absolute_position=True)
# should fail to add phase_1n
# ============================================================================
# test 8
# timeline_1.addPhase(phase_15n)
# timeline_1.addPhase(phase_8n, 20, absolute_position=True)
# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_1n, 20, absolute_position=True)
# should fail to add phase_1n
# ============================================================================
# test 8
# timeline_1.addPhase(phase_15n)
# pm.shift()
# pm.shift()

# timeline_1.addPhase(phase_8n)
# timeline_1.addPhase(phase_8n, absolute_position=True)
# timeline_1.addPhase(phase_1n)
# print_timeline_phases()
# should fail to add phase_1n
# ============================================================================
# test 9
timeline_1.addPhase(phase_15n)
print_timeline_phases()
exit()
pm.shift()

timeline_1.addPhase(phase_8n, pos=0, absolute_position=False)


# timeline_1.addPhase(phase_8n)

exit()
for i in range(3):
    pm.shift()
print_timeline_phases()

print("======================================================================")
timeline_1.addPhase(phase_8n, 2)
# timeline_1.addPhase(phase_8n, 0, absolute_position=True)
print_timeline_phases()
# print_items()
exit()
# ============================================================================

for i in range(30):
    timeline_1.addPhase(phase_1n)

timeline_1.addPhase(phase_8n, 0, absolute_position=True)

for i in range(25):
    pm.shift()

    print_timeline_phases()
    print_items()
exit()
# timeline_1.addPhase(phase_1n, 2, absolute_position=True)
# timeline_1.addPhase(phase_1n, 3, absolute_position=True)
# timeline_1.addPhase(phase_1n, 4, absolute_position=True)
# timeline_1.addPhase(phase_5n, 5, absolute_position=True)
# timeline_1.addPhase(phase_5n, 6, absolute_position=True)
# timeline_1.addPhase(phase_1n, 7, absolute_position=True)
# timeline_1.addPhase(phase_1n, 10, absolute_position=True)
# timeline_1.addPhase(phase_5n, 11, absolute_position=True)
exit()
# print(" initial condition: ")
#
# print("cnsrt1: ", cnsrt1.getNodes())
# print("cnsrt2: ", cnsrt2.getNodes())
# print("cnsrt3: ", cnsrt3.getNodes())
# print("cnsrt4: ", cnsrt4.getNodes())
# #
num_shift = 100
for i in range(num_shift):
    print(f" SHIFTING PHASES: iteration {i}")

    # if i == 4:
    #     print('+==================================+')
    #     timeline_1.addPhase(phase_5n, 6, absolute_position=True)

    pm.shift()


    print_timeline_phases()
    input()
#
# exit()
#