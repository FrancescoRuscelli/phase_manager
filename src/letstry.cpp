#include <phase_manager/phase_manager.h>
#include <phase_manager/class_resolver.h>
# include <iostream>
#include <chrono>



class Constraint
{
public:

    using ConstraintPtr = std::shared_ptr<Constraint>;

    Constraint(std::string name):
        _name(name)
    {
    }

    bool setNodes(std::vector<int> nodes)
    {
        _nodes = nodes;
    }

    bool setBounds(Eigen::VectorXd bounds)
    {
        _bounds = bounds;
    }

    std::vector<int> getNodes()
    {
        return _nodes;
    }

private:

    std::string _name;
    std::vector<int> _nodes;
    Eigen::VectorXd _bounds;
};

int main()
{

    Constraint::ConstraintPtr stance_c_1 = std::make_shared<Constraint>("stance_c_1");
    Constraint::ConstraintPtr stance_c_2 = std::make_shared<Constraint>("stance_c_2");

    Constraint::ConstraintPtr flight_c_1 = std::make_shared<Constraint>("flight_c_1");

//    Constraint template_constraint("diocane");

    SinglePhaseManager app(12);

    Phase::PhasePtr stance = std::make_shared<Phase>(5, "stance");
    Phase::PhasePtr flight = std::make_shared<Phase>(5, "flight");

    std::vector<int> my_nodes;
    my_nodes.insert(my_nodes.end(), {2, 3, 4});

    stance->addConstraint(stance_c_1, my_nodes);
    stance->addConstraint(stance_c_2);

    flight->addConstraint(flight_c_1);


    //  without registering, cannot link horizon constraints to updater
    app.registerPhase(stance);
    app.registerPhase(flight);

//    auto start_time = std::chrono::high_resolution_clock::now();
    app.addPhase(stance);
    app.addPhase(flight);

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

//    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
//    std::cout << "elapsed time: " << elapsed_time.count() << std::endl;

//    app.addPhase(flight);
//    app.addPhase(stance);
//    app.getRegisteredPhase("penis");

    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;


    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app.addPhase(stance);
    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;



    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;



    app._shift_phases();

    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;



    app._shift_phases();
    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app._shift_phases();
    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    app._shift_phases();
    std::cout << "constraint stance_c_1 has nodes: ";
    for (int i : stance_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint stance_c_2 has nodes: ";
    for (int i : stance_c_2->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    std::cout << "constraint flight_c_1 has nodes: ";
    for (int i : flight_c_1->getNodes())
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}
