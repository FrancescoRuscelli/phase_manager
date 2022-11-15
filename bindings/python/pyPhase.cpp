#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/pytypes.h>
#include <phase_manager/phase.h>

namespace py = pybind11;

struct PyObjWrapper : ItemBase {

public:

    typedef std::shared_ptr<PyObjWrapper> Ptr;

    PyObjWrapper(py::object pyobj):
        _pyobj(pyobj)
    {
    }

    std::string getName() { return _pyobj.attr("getName").cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim").cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes").cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
        _pyobj.attr("setNodes") = nodes;
        return true;
    }


private:
    py::object _pyobj;
};

struct PyObjWrapperWithBounds : ItemWithBoundsBase {
    PyObjWrapperWithBounds(py::object pyobj):
        _pyobj(pyobj)
    {
    }
    std::string getName() {return _pyobj.attr("getName").cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim").cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes").cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
        _pyobj.attr("setNodes") = nodes;
        return true;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
        _pyobj.attr("setBounds") = lower_bounds, upper_bounds;
        return true;
    }


private:
    py::object _pyobj;
};

struct PyObjWrapperWithValues : ItemWithValuesBase {
    PyObjWrapperWithValues(py::object pyobj):
        _pyobj(pyobj)
    {
    }
    std::string getName() {return _pyobj.attr("getName").cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim").cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes").cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
        _pyobj.attr("setNodes") = nodes;
        return true;
    }

    bool assign(Eigen::MatrixXd values)
    {
        _pyobj.attr("assign") = values;
        return true;
    }


private:
    py::object _pyobj;
};


bool add_cost_pyobject(Phase& self,
                       py::object item,
                       std::vector<int> nodes = {})
{
    ItemBase::Ptr item_converted = std::make_shared<PyObjWrapper>(item);
    self.addCost(item_converted, nodes);
}

bool add_constraint_pyobject(Phase& self,
                             py::object item,
                             std::vector<int> nodes = {})
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addConstraint(item_converted, nodes);
}

bool add_variable_pyobject(Phase& self, py::object item,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes = {})
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addVariableBounds(item_converted, lower_bounds, upper_bounds, nodes);
}

bool add_parameter_pyobject(Phase& self, py::object item,
                           Eigen::MatrixXd values,
                           std::vector<int> nodes = {})
{
    ItemWithValuesBase::Ptr item_converted = std::make_shared<PyObjWrapperWithValues>(item);
    self.addParameterValues(item_converted, values, nodes);
}


PYBIND11_MODULE(pyphase, m) {

    py::class_<Phase>(m, "Phase")
            .def(py::init<int, std::string>())
            .def("getName", &Phase::getName)
            .def("getNNodes", &Phase::getNNodes)
            .def("addCost", add_cost_pyobject)
            .def("addConstraint", add_constraint_pyobject) //, ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes = {})
            .def("addVariableBounds", add_variable_pyobject)
            .def("addCost", add_parameter_pyobject);
//            .def("addCost", &Phase::addVariableBounds);


}
//            .def(py::init<std::string>(), py::arg("namespace") = "cartesian")
//            .def("__repr__", ci_repr)
//            .def("update", &RosClient::update, py::arg("time") = 0, py::arg("period") = 0)
//            .def("getControlMode", &RosClient::getControlMode)
//            .def("setControlMode", &RosClient::setControlMode)
//            .def("getBaseLink", &RosClient::getBaseLink)
//            .def("setBaseLink", &RosClient::setBaseLink)
//            .def("loadController", &RosClient::loadController,
//                 py::arg("controller_name"),
//                 py::arg("problem_description_name") = "",
//                 py::arg("problem_description_string") = "",
//                 py::arg("force_reload") = true)
//            .def("getVelocityLimits", py_get_velocity_limits)
//            .def("getAccelerationLimits", py_get_acceleration_limits)
//            .def("setVelocityLimits", &RosClient::setVelocityLimits)
//            .def("setAccelerationLimits", &RosClient::setAccelerationLimits)
//            .def("setReferencePosture",  (bool (RosClient::*)(const XBot::JointNameMap&)) &RosClient::setReferencePosture)
//            .def("setTargetPose", py_send_target_pose,
//                 py::arg("task_name"),
//                 py::arg("pose"),
//                 py::arg("time"),
//                 py::arg("incremental") = false)
//            .def("setWaypoints", py_send_waypoints,
//                 py::arg("task_name"),
//                 py::arg("waypoints"),
//                 py::arg("incremental") = false)
//            .def("waitReachCompleted", &RosClient::waitReachCompleted,
//                 py::arg("task_name"),
//                 py::arg("timeout") = 0.0)
//            .def("getPoseReference", py_get_pose_reference)
//            .def("getDesiredInteraction", py_get_interaction_reference)
//            .def("setPoseReference", &RosClient::setPoseReference)
//            .def("setForceReference", &RosClient::setForceReference)
//            .def("setDesiredStiffness", &RosClient::setDesiredStiffness)
//            .def("setDesiredDamping", &RosClient::setDesiredDamping)
//            .def("resetWorld", (bool (RosClient::*)(const Eigen::Affine3d&)) &RosClient::resetWorld)
//            .def("resetWorld", (bool (RosClient::*)(const std::string&))     &RosClient::resetWorld)
//            .def("setVelocityReference", (bool (RosClient::*)(const std::string&,
//                                                              const Eigen::Vector6d&))  &RosClient::setVelocityReference)
//            //        .def("setVelocityReferenceAsync", &RosClient::setVelocityReferenceAsync)
//            .def("setVelocityReference", (bool (RosClient::*)(const std::string&,
//                                                              const Eigen::Vector6d&,
//                                                              const std::string&))  &RosClient::setVelocityReference)
//            //        .def("stopVelocityReferenceAsync", &RosClient::stopVelocityReferenceAsync)
//            .def("getPoseFromTf", py_get_pose_from_tf);

