from horizon.problem import Problem
import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

import numpy as np

ns = 10
dt = 0.01
prb = Problem(ns, receding=True)
prb.setDt(dt)
class PhaseTester:
    def __init__(self):
        pass

    def _create_prb(self):

        ns = 10
        dt = 0.01
        prb = Problem(ns, receding=True)
        prb.setDt(dt)

    def _create_phase_manager(self):

        self.pm = pymanager.PhaseManager(ns)
        self.timeline_1 = self.pm.addTimeline('timeline_1')

    def _setup(self):

        self._create_prb()
        self._create_phase_manager()
    def test_constraint(self):

        self._setup()
        a = prb.createStateVariable('a', 1)
        cnsrt_1 = prb.createConstraint('cnsrt_1', a / 2, nodes=[])
        phase_1 = pyphase.Phase(4, 'phase_1')
        phase_1.addConstraint(cnsrt_1, nodes=[0, 3])

        self.timeline_1.registerPhase(phase_1)
        self.timeline_1.addPhase(phase_1)

        for num_it in range(5):

            print(cnsrt_1.getNodes())
            # print('constraints and active nodes:')
            # for elem in phase_1.getConstraints():
            #     print(f"{elem.getName()}: {elem.getNodes()}")

            self.pm._shift_phases()


    def test_parameter(self):

        self._setup()
        par = prb.createParameter('par', 1)
        phase_1 = pyphase.Phase(4, 'phase_1')
        phase_1.addParameterValues(par, np.array([[1., 3., 4.]]), [0, 2, 3])
        # phase_1.addParameterValues(par, np.array([[1., 3., 4., 5.]]))

        self.timeline_1.registerPhase(phase_1)
        self.timeline_1.addPhase(phase_1)

        for num_it in range(5):
            print(par.getValues())

            self.pm._shift_phases()


    def test_variable(self):

        self._setup()
        a = prb.createStateVariable('a', 1)
        phase_1 = pyphase.Phase(4, 'phase_1')
        phase_1.addVariableBounds(a, np.array([[-1, -2]]), np.array([[1, 2]]), [1, 3])
        phase_1.addVariableBounds(a, np.array([[-1, -1, -1, -1]]), np.array([[1, 1, 1, 1]]))

        self.timeline_1.registerPhase(phase_1)
        self.timeline_1.addPhase(phase_1)

        for num_it in range(5):
            print(a.getBounds())

        # print('variables, active nodes and bounds:')
        # for elem in phase_1.getVariables():
        #     print(f"{elem.getName()}: {elem.getNodes()}, \n{elem.getBounds()}")
        # exit()

            self.pm._shift_phases()




if __name__ == '__main__':

    pt = PhaseTester()
    # pt.test_variable()
    pt.test_parameter()
    # pt.test_constraint()