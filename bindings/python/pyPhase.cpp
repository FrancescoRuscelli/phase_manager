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

    std::string getName() { return _pyobj.attr("getName")().cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim")().cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes")().cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
        _pyobj.attr("setNodes")(nodes);
        return true;
    }


private:
    py::object _pyobj;
};

struct PyObjWrapperWithBounds : ItemWithBoundsBase {
    PyObjWrapperWithBounds(py::object pyobj):
        _pyobj(pyobj)
    {
        // what if empty?
        _lower_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());
        _upper_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());

        _initial_lower_bounds =  _lower_bounds;
        _initial_upper_bounds =  _upper_bounds;
    }
    std::string getName(){return _pyobj.attr("getName")().cast<std::string>();}
    int getDim() {return _pyobj.attr("getDim")().cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes")().cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
//        std::cout << "(pyphase) setting nodes: " << std::endl;
//        for (auto node: nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;
        _pyobj.attr("setNodes")(nodes);
        return true;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
//        std::cout << "(pyphase) setting bounds: " << lower_bounds << std::endl;
        _pyobj.attr("setBounds")(lower_bounds, upper_bounds);
        return true;
    }


private:
    py::object _pyobj;
};

struct PyObjWrapperWithValues : ItemWithValuesBase {
    PyObjWrapperWithValues(py::object pyobj):
        _pyobj(pyobj)
    {
        // what if empty?
        _values = Eigen::MatrixXd::Zero(getDim(), getNodes().size());

        _initial_values = _values;
    }
    std::string getName() {return _pyobj.attr("getName")().cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim")().cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes")().cast<std::vector<int>>(); }

    bool setNodes(std::vector<int> nodes)
    {
        _pyobj.attr("setNodes")(nodes);
        return true;
    }

    bool assign(Eigen::MatrixXd values)
    {
        _pyobj.attr("assign")(values);
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
                             std::vector<int> nodes)
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addConstraint(item_converted, nodes);
}

bool add_variable_pyobject(Phase& self,
                           py::object item,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes)
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addVariableBounds(item_converted, lower_bounds, upper_bounds, nodes);
}

bool add_parameter_pyobject(Phase& self, py::object item,
                           Eigen::MatrixXd values,
                           std::vector<int> nodes)
{
    ItemWithValuesBase::Ptr item_converted = std::make_shared<PyObjWrapperWithValues>(item);
    self.addParameterValues(item_converted, values, nodes);
}


PYBIND11_MODULE(pyphase, m) {

    py::class_<Phase, Phase::Ptr>(m, "Phase")
            .def(py::init<int, std::string>())
            .def("getName", &Phase::getName)
            .def("getNNodes", &Phase::getNNodes)
            .def("addCost", add_cost_pyobject, py::arg("item"), py::arg("nodes") = std::vector<int>())
            .def("addConstraint", add_constraint_pyobject, py::arg("item"), py::arg("nodes") = std::vector<int>())
            .def("addVariableBounds", add_variable_pyobject, py::arg("item"), py::arg("lower_bounds"), py::arg("upper_bounds"), py::arg("nodes") = std::vector<int>())
            .def("addParameterValues", add_parameter_pyobject, py::arg("item"), py::arg("values"), py::arg("nodes") = std::vector<int>());

}
