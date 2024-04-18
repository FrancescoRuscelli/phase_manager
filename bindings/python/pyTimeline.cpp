#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <phase_manager/timeline.h>
#include <phase_manager/phase_manager.h>
#include <phase_manager/phase.h>

namespace py = pybind11;


PYBIND11_MODULE(pytimeline, m) {


    py::class_<Timeline, Timeline::Ptr>(m, "Timeline")
            .def(py::init<PhaseManager&, int, std::string>())
            .def("getName", &Timeline::getName)
            .def("createPhase", &Timeline::createPhase)
            .def("addPhase", static_cast<bool (Timeline::*)(std::vector<Phase::Ptr>, int, bool)>(&Timeline::addPhase), py::arg("phases"), py::arg("pos") = -1, py::arg("absolute_position") = false)
            .def("addPhase", static_cast<bool (Timeline::*)(Phase::Ptr, int, bool)>(&Timeline::addPhase), py::arg("phase"), py::arg("pos") = -1, py::arg("absolute_position") = false)
//            .def("getRegisteredPhase", &Timeline::getRegisteredPhase)
//            .def("getRegisteredPhases", &Timeline::getRegisteredPhases)
            .def("getEmptyNodes", &Timeline::getEmptyNodes)
            .def("getActivePhases", &Timeline::getActivePhases)
            .def("getPhases", &Timeline::getPhases)
            .def("shift", &Timeline::shift)
            .def("clear", &Timeline::clear)
            ;

}
