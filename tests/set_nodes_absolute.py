import colorama
import phase_manager.pymanager
from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

def printElemInfo(elem):
    print(f"  |     Phase position: {elem.getPosition()}")
    print(f"  |     Phase duration: {elem.getNNodes()}")
    print(f"  |     Active nodes of phase: {elem.getActiveNodes()}")
    print(f"  |_____________________________________________________________|")
def printAllPhases(timeline: phase_manager.pymanager.SinglePhaseManager, add_element_info=False):
    elem_num = 1
    print(f"{colorama.Style.BRIGHT}==================== ALL PHASES: ===================={colorama.Style.RESET_ALL}")
    for elem in timeline.getPhases():
        print(f"{colorama.Fore.RED}- {elem_num}: {colorama.Style.RESET_ALL}", end="")
        print(f"{elem.getName()}", end="")
        print(f": {elem}")
        if add_element_info:
            printElemInfo(elem)
        elem_num += 1
def printActivePhases(timeline: phase_manager.pymanager.SinglePhaseManager, add_element_info=False):
    elem_num = 1
    print(f"{colorama.Style.BRIGHT}==================== ACTIVE PHASES: ===================={colorama.Style.RESET_ALL}")
    for elem in timeline.getActivePhases():
        print(f"{colorama.Fore.RED}- {elem_num}: {colorama.Style.RESET_ALL}", end="")
        print(f"{elem.getName()}: {elem}.")
        if add_element_info:
            printElemInfo(elem)
        elem_num += 1

if __name__ == '__main__':

    ns = 10
    dt = 0.01
    prb = Problem(ns, receding=True)
    prb.setDt(dt)

    pm = pymanager.PhaseManager(ns)
    timeline_1 = pm.addTimeline('timeline_1')

    # adding stuff to the problem
    a = prb.createStateVariable('a', 1)
    par = prb.createParameter('par', 1)
    cnsrt_1 = prb.createConstraint('cnsrt_1', a / 2, nodes=[])

    phase_1 = pyphase.Phase(4, 'phase_1')
    phase_1.addParameterValues(par, np.array([[1., 3., 4.]]), [0, 2, 3])

    phase_2 = pyphase.Phase(5, 'phase_2')
    phase_2.addVariableBounds(a, np.array([[-1, -2, -3, -4, -5]]), np.array([[1, 2, 3, 4, 5]]))
    phase_2.addConstraint(cnsrt_1, nodes=[0, 3])

    phase_inserted = pyphase.Phase(3, 'phase_inserted')

    timeline_1.registerPhase(phase_1)
    timeline_1.registerPhase(phase_2)
    timeline_1.registerPhase(phase_inserted)

    # timeline_1.addPhase(phase_1)
    # timeline_1.addPhase(phase_2)
    # timeline_1.addPhase(phase_1)
    # timeline_1.addPhase(phase_1)
    # timeline_1.addPhase(phase_1)
    timeline_1.addPhase(phase_1, 0, absolute_position=True)
    timeline_1.addPhase(phase_1, 4, absolute_position=True)
    timeline_1.addPhase(phase_1, 9, absolute_position=True)
    timeline_1.addPhase(phase_1, 13, absolute_position=True)
    # timeline_1.addPhase(phase_inserted, 4, absolute_position=True)
    # timeline_1.addPhase(phase_1)
    # timeline_1.addPhase(phase_inserted, 4, absolute_position=True)

    # timeline_1.clear()

    printAllPhases(timeline_1, add_element_info=True)
    # printActivePhases(timeline_1, add_element_info=True)
    exit()
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    for i in range(5):
        timeline_1.shift()
        print("SHIFTING PHASES: ")

        printAllPhases(timeline_1, add_element_info=True)














