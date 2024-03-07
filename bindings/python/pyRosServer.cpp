#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <phase_manager/ros_server_class.h>

namespace py = pybind11;
using namespace HorizonPhases;


PYBIND11_MODULE(pyrosserver, m) {

    py::class_<RosServerClass, RosServerClass::Ptr>(m, "RosServerClass")
            .def(py::init<PhaseManager::Ptr>())
            .def("run", &RosServerClass::run)
            ;
}
