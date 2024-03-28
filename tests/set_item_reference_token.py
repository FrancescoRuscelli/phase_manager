from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase
import time
import numpy as np

ns = 15
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)

class FakeItem:
    def __init__(self, par):

        self.name = 'fake_item'
        self.ref = par
        self.nodes = list(range(50))

    def getName(self):

        return self.name

    def setNodes(self, nodes, erasing=True):

        self.nodes = nodes

    def assign(self, val, nodes=None):

        print(f'    fake item: calling assing with values {val} at nodes {nodes}')
        # necessary method for using this task as an item + reference in phaseManager
        self.ref.assign(val, nodes)
        return 1

    def getValues(self):
        # necessary method for using this task as an item + reference in phaseManager
        return self.ref.getValues()


a = prb.createStateVariable('a', 1)
par = prb.createParameter('par', 1)

print('initial values: ', par.getValues())

fake_item = FakeItem(par)
pm = pymanager.PhaseManager(ns)
timeline_1 = pm.addTimeline('timeline_1')
phase_1 = pyphase.Phase(5, 'phase_1')

# phase_1.addItemReference(fake_item, np.array([[10., 11., 12., 13., 14.]]))
phase_1.addItemReference(fake_item, np.array([[2]]), nodes=[1])
timeline_1.registerPhase(phase_1)

timeline_1.addPhase(phase_1)
# timeline_1.addPhase(phase_1)

print('values after adding phase: ', par.getValues())

print('==========================================================')
for i in range(7):
    pm.shift()
    print('values after shifting: ', par.getValues())

# print("modifying phase: ")
# timeline_1.getPhases()[1].setItemReference('fake_item', np.array([[7., 7., 7., 7., 7.]]))
#
# print('values after modifying phase: ', par.getValues())
#





# for phase in timeline_1.getPhases():
#     print(phase.getName())
