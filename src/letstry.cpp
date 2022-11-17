#include <phase_manager/phase_manager.h>
#include <phase_manager/class_resolver.h>
# include <iostream>
#include <chrono>

class ClassWithoutBounds
{};

class Cost
{
public:

    using CostPtr = std::shared_ptr<Cost>;

    Cost(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
    }

    bool setNodes(std::vector<int> nodes)
    {
        _nodes = nodes;
    }

    std::vector<int> getNodes()
    {
        return _nodes;
    }

    std::string getName()
    {
        return _name;
    }

    int getDim()
    {
        return _dim;
    }

private:

    std::string _name;
    int _dim;
    std::vector<int> _nodes;
};

class Parameter
{
public:

    using ParameterPtr = std::shared_ptr<Parameter>;

    Parameter(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
    }

    bool setNodes(std::vector<int> nodes)
    {
        _nodes = nodes;
    }

    bool assign(Eigen::MatrixXd value)
    {
        _values = value;
    }

    std::vector<int> getNodes()
    {
        return _nodes;
    }

    Eigen::MatrixXd getValues()
    {
        return _values;
    }

    std::string getName()
    {
        return _name;
    }

    int getDim()
    {
        return _dim;
    }

private:

    std::string _name;
    int _dim;
    std::vector<int> _nodes;
    Eigen::MatrixXd _values;
};

class Variable
{
public:

    using VariablePtr = std::shared_ptr<Variable>;

    Variable(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
    }

    bool setNodes(std::vector<int> nodes)
    {
        _nodes = nodes;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
        _lower_bounds = lower_bounds;
        _upper_bounds = upper_bounds;
    }

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> getBounds()
    {
        return {_lower_bounds, _upper_bounds};
    }

    std::vector<int> getNodes()
    {
        return _nodes;
    }

    std::string getName()
    {
        return _name;
    }

    int getDim()
    {
        return _dim;
    }

private:

    std::string _name;
    int _dim;
    std::vector<int> _nodes;
    Eigen::MatrixXd _lower_bounds;
    Eigen::MatrixXd _upper_bounds;
};

class Constraint
{
public:

    using ConstraintPtr = std::shared_ptr<Constraint>;

    Constraint(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
    }

    bool setNodes(std::vector<int> nodes)
    {
        _nodes = nodes;
    }

    std::string getName()
    {
        return _name;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
        _lower_bounds = lower_bounds;
        _upper_bounds = upper_bounds;
    }

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> getBounds()
    {
        return {_lower_bounds, _upper_bounds};
    }

    std::vector<int> getNodes()
    {
        return _nodes;
    }

    int getDim()
    {
        return _dim;
    }

private:

    std::string _name;
    int _dim;
    std::vector<int> _nodes;
    Eigen::MatrixXd _lower_bounds;
    Eigen::MatrixXd _upper_bounds;

};

int main()
{
    std::cout << has_set_bounds<ClassWithoutBounds>() << std::endl;
    std::cout << has_set_bounds<Constraint>() << std::endl;

//     all of this comes from outside
    Cost::CostPtr fake_stance_cc_1 = std::make_shared<Cost>("stance_cc_1", 1);

    Constraint::ConstraintPtr fake_stance_c_1 = std::make_shared<Constraint>("stance_c_1", 1);
    Constraint::ConstraintPtr fake_stance_c_2 = std::make_shared<Constraint>("stance_c_2", 1);

    Constraint::ConstraintPtr fake_flight_c_1 = std::make_shared<Constraint>("flight_c_1", 1);

    Parameter::ParameterPtr fake_flight_p_1 = std::make_shared<Parameter>("flight_p_1", 3);

    Variable::VariablePtr fake_var_1 = std::make_shared<Variable>("var_1", 3);

    Eigen::MatrixXd bounds_var_1 = Eigen::MatrixXd::Zero(3,5);
    std::vector<int> prb_nodes(50);
    std::iota(prb_nodes.begin(), prb_nodes.end(), 50);

//    for (auto element : prb_nodes)
//    {
//        std::cout << element << " ";
//    }
    fake_flight_p_1->setNodes(prb_nodes);
    fake_var_1->setNodes(prb_nodes);

    Eigen::MatrixXd values_par_1 = Eigen::MatrixXd::Zero(3,50);
    fake_flight_p_1->assign(values_par_1);

    // ==================================================================================================================

    // convert variable to something usable by phase_manager
    ItemBase::Ptr stance_cc_1 = std::make_shared<Wrapper<Cost>>(fake_stance_cc_1);
    ItemWithBoundsBase::Ptr var_1 = std::make_shared<WrapperWithBounds<Variable>>(fake_var_1);
    ItemWithBoundsBase::Ptr stance_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_1);
    ItemWithBoundsBase::Ptr stance_c_2 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_2);
    ItemWithBoundsBase::Ptr flight_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_flight_c_1);
    ItemWithValuesBase::Ptr flight_p_1 = std::make_shared<WrapperWithValues<Parameter>>(fake_flight_p_1);

    PhaseManager app(50);
    auto timeline_1 = app.addTimeline("first_timeline");

//    SinglePhaseManager app(20);

    Phase::Ptr stance = std::make_shared<Phase>(5, "stance");
    Phase::Ptr flight = std::make_shared<Phase>(5, "flight");

    std::vector<int> my_nodes;
    my_nodes.insert(my_nodes.end(), {2, 3, 4});

    stance->addConstraint(stance_c_1); //, my_nodes);
    stance->addConstraint(stance_c_2);

    stance->addVariableBounds(var_1, bounds_var_1, bounds_var_1);
    flight->addVariableBounds(var_1, bounds_var_1, bounds_var_1);

    flight->addConstraint(flight_c_1);

    Eigen::MatrixXd values(3, 5);
    values << 1.5, 1.5, 1.5, 1.5, 1.5,
              1.5, 1.5, 1.5, 1.5, 1.5,
              1.5, 1.5, 1.5, 1.5, 1.5;

    flight->addParameterValues(flight_p_1, values);


    //  without registering, cannot link horizon constraints to updater
    timeline_1->registerPhase(stance);
    timeline_1->registerPhase(flight);

//    auto start_time = std::chrono::high_resolution_clock::now();
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
    timeline_1->addPhase(stance);
//    timeline_1->addPhase(flight, 1);


    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : fake_stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : fake_stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : fake_flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "parameter flight_p_1 has values: " << std::endl;
    std::cout << fake_flight_p_1->getValues() << std::endl;

    std::cout << "variable var_1 has bounds: " << std::endl;
    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
    std::cout << std::endl;

//    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
//    std::cout << "elapsed time: " << elapsed_time.count() << std::endl;


//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();


//    std::cout << "constraint stance_c_1 has nodes: ";
//    for (int i : fake_stance_c_1->getNodes())
//    {
//        std::cout << i << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "constraint stance_c_2 has nodes: ";
//    for (int i : fake_stance_c_2->getNodes())
//    {
//        std::cout << i << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "constraint flight_c_1 has nodes: ";
//    for (int i : fake_flight_c_1->getNodes())
//    {
//        std::cout << i << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "parameter flight_p_1 has values: " << std::endl;
//    std::cout << fake_flight_p_1->getValues() << std::endl;

//    std::cout << "variable var_1 has bounds: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;

}
