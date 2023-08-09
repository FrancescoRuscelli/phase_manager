#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <phase_manager/phase_manager.h>

namespace py = pybind11;


PYBIND11_MODULE(pymanager, m) {

    py::class_<SinglePhaseManager, SinglePhaseManager::Ptr>(m, "SinglePhaseManager")
            .def(py::init<int, std::string>())
            .def("registerPhase", &SinglePhaseManager::registerPhase)
            .def("addPhase", static_cast<bool (SinglePhaseManager::*)(std::vector<Phase::Ptr>, int)>(&SinglePhaseManager::addPhase), py::arg("phases"), py::arg("pos") = -1)
            .def("addPhase", static_cast<bool (SinglePhaseManager::*)(Phase::Ptr, int)>(&SinglePhaseManager::addPhase), py::arg("phase"), py::arg("pos") = -1)
            .def("getRegisteredPhase", &SinglePhaseManager::getRegisteredPhase)
            .def("getRegisteredPhases", &SinglePhaseManager::getRegisteredPhases)
            .def("getEmptyNodes", &SinglePhaseManager::getEmptyNodes)
            .def("getActivePhases", &SinglePhaseManager::getActivePhases)
            .def("getPhases", &SinglePhaseManager::getPhases)
            .def("shift_phases", &SinglePhaseManager::_shift_phases)
            ;


    py::class_<PhaseManager>(m, "PhaseManager")
            .def(py::init<int>())
            .def("addTimeline", &PhaseManager::addTimeline)
            .def("getTimelines", static_cast<SinglePhaseManager::Ptr (PhaseManager::*)(std::string)>(&PhaseManager::getTimelines))
            .def("getTimelines", static_cast<std::unordered_map<std::string, SinglePhaseManager::Ptr> (PhaseManager::*)()>(&PhaseManager::getTimelines))
            .def("addPhase", &PhaseManager::addPhase)
            .def("registerPhase", &PhaseManager::registerPhase)
            .def("_shift_phases", &PhaseManager::_shift_phases)
            .def("getNodes", &PhaseManager::getNodes)
            ;

}
