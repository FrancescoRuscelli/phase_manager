#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <phase_manager/phase_manager.h>
#include <phase_manager/timeline.h>
#include <phase_manager/phase.h>

namespace py = pybind11;


PYBIND11_MODULE(pymanager, m) {


    py::class_<PhaseManager, PhaseManager::Ptr>(m, "PhaseManager")
            .def(py::init<int>())
            .def("createTimeline", &PhaseManager::createTimeline)
            .def("getTimelines", static_cast<Timeline::Ptr (PhaseManager::*)(std::string)>(&PhaseManager::getTimelines))
            .def("getTimelines", static_cast<std::unordered_map<std::string, Timeline::Ptr> (PhaseManager::*)()>(&PhaseManager::getTimelines))
//            .def("addPhase", &PhaseManager::addPhase)
//            .def("registerPhase", &PhaseManager::registerPhase)
            .def("shift", &PhaseManager::shift)
            .def("clear", &PhaseManager::clear)
            .def("getNodes", &PhaseManager::getNodes)
            ;

}
