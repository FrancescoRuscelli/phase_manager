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

    bool setNodesInternal(std::vector<int> nodes, bool erasing)
    {
        _pyobj.attr("setNodes")(nodes, erasing);
        return true;
    }

    py::object getPyObject()
    {
        return _pyobj;
    }

private:
    py::object _pyobj;
};

struct PyObjWrapperWithBounds : ItemWithBoundsBase {
    PyObjWrapperWithBounds(py::object pyobj):
        _pyobj(pyobj)
    {
//        _lower_bounds = - std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());
//        _upper_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(getDim(), getNodes().size());

        //TODO: why inf? better like this
        // initial bounds are required when clearing the phase: the current value is restored to the initial one.
        _lower_bounds = std::get<0>(getBounds());
        _upper_bounds = std::get<1>(getBounds());

        _initial_lower_bounds =  _lower_bounds;
        _initial_upper_bounds =  _upper_bounds;

    }
    std::string getName(){return _pyobj.attr("getName")().cast<std::string>();}
    int getDim() {return _pyobj.attr("getDim")().cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes")().cast<std::vector<int>>(); }
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> getBounds() {return _pyobj.attr("getBounds")().cast<std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>>(); }

    bool setNodesInternal(std::vector<int> nodes, bool erasing)
    {
//        std::cout << "(pyphase) setting nodes: " << std::endl;
//        for (auto node: nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;
        _pyobj.attr("setNodes")(nodes, erasing);
        return true;
    }

    bool setBoundsInternal(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
    {
//        std::cout << "(pyphase) setting bounds: " << lower_bounds << std::endl;
        _pyobj.attr("setBounds")(lower_bounds, upper_bounds, nodes);
        return true;
    }

    py::object getPyObject()
    {
        return _pyobj;
    }

private:
    py::object _pyobj;
};

struct PyObjWrapperWithValues : ItemWithValuesBase {
    PyObjWrapperWithValues(py::object pyobj):
        _pyobj(pyobj)
    {
//        std::cout << getName() << std::endl;
        // TODO
        // what if empty?
        // what if the parameter starts with zero nodes
        _values = getValues();
        _initial_values = _values;

//        std::cout << "initialization of param: " << _values << std::endl;

//        std::cout << "with nodes: " << std::endl;
//        for (auto node: getNodes())
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

    }
    std::string getName() {return _pyobj.attr("getName")().cast<std::string>(); }
    int getDim() {return _pyobj.attr("getDim")().cast<int>(); }
    std::vector<int> getNodes() {return _pyobj.attr("getNodes")().cast<std::vector<int>>(); }
    Eigen::MatrixXd getValues() {return _pyobj.attr("getValues")().cast<Eigen::MatrixXd>(); }

    bool setNodesInternal(std::vector<int> nodes, bool erasing)
    {
        _pyobj.attr("setNodes")(nodes, erasing);
        return true;
    }

    bool assignInternal(Eigen::MatrixXd values, std::vector<int> nodes)
    {
        _pyobj.attr("assign")(values, nodes);
        return true;
    }

    bool assignInternal(Eigen::MatrixXd values)
    {
        _pyobj.attr("assign")(values);
        return true;
    }


    py::object getPyObject()
    {
        return _pyobj;
    }

private:
    py::object _pyobj;
};


bool add_item_pyobject(Phase& self,
                       py::object item,
                       std::vector<int> nodes = {})
{
    ItemBase::Ptr item_converted = std::make_shared<PyObjWrapper>(item);
    self.addItem(item_converted, nodes);
    return true;
}

bool add_item_reference_pyobject(Phase& self, py::object item,
                                 Eigen::MatrixXd values,
                                 std::vector<int> nodes)
{
    ItemWithValuesBase::Ptr item_converted = std::make_shared<PyObjWrapperWithValues>(item);
    self.addItemReference(item_converted, values, nodes);
    return true;
}

bool add_cost_pyobject(Phase& self,
                       py::object item,
                       std::vector<int> nodes = {})
{
    ItemBase::Ptr item_converted = std::make_shared<PyObjWrapper>(item);
    self.addCost(item_converted, nodes);
    return true;
}

bool add_constraint_pyobject(Phase& self,
                             py::object item,
                             std::vector<int> nodes)
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addConstraint(item_converted, nodes);
    return true;
}

bool add_variable_pyobject(Phase& self,
                           py::object item,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes)
{
    ItemWithBoundsBase::Ptr item_converted = std::make_shared<PyObjWrapperWithBounds>(item);
    self.addVariableBounds(item_converted, lower_bounds, upper_bounds, nodes);
    return true;
}

bool add_parameter_pyobject(Phase& self, py::object item,
                            Eigen::MatrixXd values,
                            std::vector<int> nodes)
{
    ItemWithValuesBase::Ptr item_converted = std::make_shared<PyObjWrapperWithValues>(item);
    self.addParameterValues(item_converted, values, nodes);
    return true;
}


auto _get_constraints_list(Phase& phase)
{
    auto elems = phase.getConstraints();

    std::vector<py::object> py_elem_list;

    for (auto it : elems)
    {
        py_elem_list.push_back(std::dynamic_pointer_cast<PyObjWrapperWithBounds>(it)->getPyObject());
    }

    return py_elem_list;

}

auto _get_items_list(Phase& phase)
{
    auto elems = phase.getItems();

    std::vector<py::object> py_elem_list;

    for (auto it : elems)
    {
        py_elem_list.push_back(std::dynamic_pointer_cast<PyObjWrapper>(it)->getPyObject());
    }

    return py_elem_list;

}

auto _get_parameters_list(Phase& phase)
{
    auto elems = phase.getParameters();

    std::vector<py::object> py_elem_list;

    for (auto it : elems)
    {
        py_elem_list.push_back(std::dynamic_pointer_cast<PyObjWrapperWithValues>(it)->getPyObject());
    }

    return py_elem_list;

}

auto _get_variables_list(Phase& phase)
{
    auto elems = phase.getVariables();

    std::vector<py::object> py_elem_list;

    for (auto it : elems)
    {
        py_elem_list.push_back(std::dynamic_pointer_cast<PyObjWrapperWithBounds>(it)->getPyObject());
    }

    return py_elem_list;

}

auto _get_costs_list(Phase& phase)
{
    auto elems = phase.getCosts();

    std::vector<py::object> py_elem_list;

    for (auto it : elems)
    {
        py_elem_list.push_back(std::dynamic_pointer_cast<PyObjWrapper>(it)->getPyObject());
    }

    return py_elem_list;

}

auto _get_constraints_map(Phase& phase)
{
    auto constraints = phase.getConstraintsInfo();

    std::map<py::object, std::vector<int>> elem_python;

    for (auto elem : constraints)
    {
        auto elem_converted = std::dynamic_pointer_cast<PyObjWrapperWithBounds>(elem.first);

        if (!elem_converted)
        {
//            std::cout << "what happens " << typeid(*elem.first).name() << std::endl;
            continue;
        }

        elem_python.insert({elem_converted->getPyObject(), elem.second->nodes});
    }

    return elem_python;
}

auto _get_items_map(Phase& phase)
{
    auto items = phase.getItemsInfo();

    std::map<py::object, std::vector<int>> elem_python;

    for (auto elem : items)
    {
        auto elem_converted = std::dynamic_pointer_cast<PyObjWrapperWithBounds>(elem.first);

        if (!elem_converted)
        {
//            std::cout << "what happens " << typeid(*elem.first).name() << std::endl;
            continue;
        }

        elem_python.insert({elem_converted->getPyObject(), elem.second->nodes});
    }

    return elem_python;
}

auto _get_variables_map(Phase& phase)
{
    auto variables = phase.getVariablesInfo();

    std::map<py::object, std::vector<int>> elem_python;

    for (auto elem : variables)
    {
        auto elem_converted = std::dynamic_pointer_cast<PyObjWrapperWithBounds>(elem.first);

        if (!elem_converted)
        {
//            std::cout << "what happens " << typeid(*elem.first).name() << std::endl;
            continue;
        }

        elem_python.insert({elem_converted->getPyObject(), elem.second->nodes});
    }

    return elem_python;
}

auto _get_costs_map(Phase& phase)
{
    auto costs = phase.getCostsInfo();

    std::map<py::object, std::vector<int>> elem_python;

    for (auto elem : costs)
    {
        auto elem_converted = std::dynamic_pointer_cast<PyObjWrapper>(elem.first);

        if (!elem_converted)
        {
//            std::cout << "what happens " << typeid(*elem.first).name() << std::endl;
            continue;
        }

        elem_python.insert({elem_converted->getPyObject(), elem.second->nodes});
    }

    return elem_python;
}

auto _get_parameters_map(Phase& phase)
{
    auto parameters = phase.getParametersInfo();

    std::map<py::object, std::vector<int>> elem_python;

    for (auto elem : parameters)
    {
        auto elem_converted = std::dynamic_pointer_cast<PyObjWrapperWithValues>(elem.first);

        if (!elem_converted)
        {
//            std::cout << "what happens " << typeid(*elem.first).name() << std::endl;
            continue;
        }

        elem_python.insert({elem_converted->getPyObject(), elem.second->nodes});
    }

    return elem_python;
}

PYBIND11_MODULE(pyphase, m) {

    py::class_<Phase, Phase::Ptr>(m, "Phase")
            .def(py::init<int, std::string>())
            .def("getName", &Phase::getName)
            .def("getNNodes", &Phase::getNNodes)
            .def("setDuration", &Phase::setDuration)
            .def("addItem", add_item_pyobject, py::arg("item"), py::arg("nodes") = std::vector<int>())
            .def("addItemReference", add_item_reference_pyobject, py::arg("item"), py::arg("values"), py::arg("nodes") = std::vector<int>())
            .def("addCost", add_cost_pyobject, py::arg("item"), py::arg("nodes") = std::vector<int>())
            .def("addConstraint", add_constraint_pyobject, py::arg("item"), py::arg("nodes") = std::vector<int>())
            .def("addVariableBounds", add_variable_pyobject, py::arg("item"), py::arg("lower_bounds"), py::arg("upper_bounds"), py::arg("nodes") = std::vector<int>())
            .def("addParameterValues", add_parameter_pyobject, py::arg("item"), py::arg("values"), py::arg("nodes") = std::vector<int>())
            .def("getConstraintsInfo", _get_constraints_map) // py::return_value_policy::reference_internal
            .def("getItemsInfo", _get_items_map)
            .def("getParametersInfo", _get_parameters_map)
            .def("getCostsInfo", _get_costs_map)
            .def("getVariablesInfo", _get_variables_map)
            .def("getItems", _get_items_list)
            .def("getParameters", _get_parameters_list)
            .def("getCosts", _get_costs_list)
            .def("getVariables", _get_variables_list)
            .def("getConstraints", _get_constraints_list)
            .def("setElementNodes", &Phase::setElemNodes, py::arg("elem_name"), py::arg("nodes"), py::arg("value_1") = Eigen::MatrixXd(), py::arg("value_2") = Eigen::MatrixXd())

            ;

    py::class_<PhaseToken, PhaseToken::Ptr>(m, "PhaseToken")
            .def("getName", &PhaseToken::getName)
            .def("getActiveNodes", &PhaseToken::getActiveNodes)
            .def("getPosition", &PhaseToken::getPosition)
            .def("getNNodes", &PhaseToken::getNNodes)

            ;
}
