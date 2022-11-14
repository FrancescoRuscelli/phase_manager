#include <phase_manager/phase_manager.h>
#include <phase_manager/class_resolver.h>
# include <iostream>
#include <chrono>

class Porcodio
{};

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
    std::cout << has_set_bounds<Porcodio>() << std::endl;
    std::cout << has_set_bounds<Constraint>() << std::endl;

//     all of this comes from outside
    Constraint::ConstraintPtr fake_stance_c_1 = std::make_shared<Constraint>("stance_c_1", 1);
    Constraint::ConstraintPtr fake_stance_c_2 = std::make_shared<Constraint>("stance_c_2", 1);

    Constraint::ConstraintPtr fake_flight_c_1 = std::make_shared<Constraint>("flight_c_1", 1);

    Variable::VariablePtr fake_var_1 = std::make_shared<Variable>("var_1", 3); //Variable::MakeVariable("var_1");

    Eigen::MatrixXd bounds_var_1 = Eigen::MatrixXd::Zero(3,5);
    std::vector<int> prb_nodes;
    prb_nodes.insert(prb_nodes.end(), {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20});
    fake_var_1->setNodes(prb_nodes);

    // ==================================================================================================================

    // convert variable to somethiong usable
    ItemWithBoundsBase::ItemWithBoundsBasePtr var_1 = std::make_shared<WrapperWithBounds<Variable>>(fake_var_1);
    ItemWithBoundsBase::ItemWithBoundsBasePtr stance_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_1);
    ItemWithBoundsBase::ItemWithBoundsBasePtr stance_c_2 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_2);
    ItemWithBoundsBase::ItemWithBoundsBasePtr  flight_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_flight_c_1);

    SinglePhaseManager app(20);

    Phase::PhasePtr stance = std::make_shared<Phase>(5, "stance");
    Phase::PhasePtr flight = std::make_shared<Phase>(5, "flight");

    std::vector<int> my_nodes;
    my_nodes.insert(my_nodes.end(), {2, 3, 4});

    stance->addConstraint(stance_c_1, my_nodes);
    stance->addConstraint(stance_c_2);

    stance->addVariableBounds(var_1, bounds_var_1, bounds_var_1);
    flight->addVariableBounds(var_1, bounds_var_1, bounds_var_1);

    flight->addConstraint(flight_c_1);


    //  without registering, cannot link horizon constraints to updater
    app.registerPhase(stance);
    app.registerPhase(flight);

//    auto start_time = std::chrono::high_resolution_clock::now();
    app.addPhase(stance);
    app.addPhase(flight);


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

    std::cout << "variable var_1 has bounds: " << std::endl;
    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
    std::cout << std::endl;

//    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
//    std::cout << "elapsed time: " << elapsed_time.count() << std::endl;

//    app.addPhase(flight);
//    app.addPhase(stance);
//    app.getRegisteredPhase("penis");

    app._shift_phases();
    app._shift_phases();
    app._shift_phases();
    app._shift_phases();
    app._shift_phases();


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

    std::cout << "variable var_1 has bounds: " << std::endl;
    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
    std::cout << std::endl;

}
