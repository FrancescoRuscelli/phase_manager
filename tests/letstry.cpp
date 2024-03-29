#include <phase_manager/phase_manager.h>
#include <phase_manager/class_resolver.h>
# include <iostream>
#include <chrono>

//class ClassWithoutBounds
//{};

//class Cost
//{
//public:

//    using CostPtr = std::shared_ptr<Cost>;

//    Cost(std::string name, int dim):
//        _name(name),
//        _dim(dim)
//    {
//    }

//    bool setNodes(std::vector<int> nodes)
//    {
//        _nodes = nodes;
//        return true;
//    }

//    std::vector<int> getNodes()
//    {
//        return _nodes;
//    }

//    std::string getName()
//    {
//        return _name;
//    }

//    int getDim()
//    {
//        return _dim;
//    }

//private:

//    std::string _name;
//    int _dim;
//    std::vector<int> _nodes;
//};

//class Parameter
//{
//public:

//    using ParameterPtr = std::shared_ptr<Parameter>;

//    Parameter(std::string name, int dim):
//        _name(name),
//        _dim(dim)
//    {
//    }

//    bool setNodes(std::vector<int> nodes)
//    {
//        _nodes = nodes;
//        return true;
//    }

//    bool assign(Eigen::MatrixXd value)
//    {
//        _values = value;
//        return true;
//    }

//    std::vector<int> getNodes()
//    {
//        return _nodes;
//    }

//    Eigen::MatrixXd getValues()
//    {
//        return _values;
//    }

//    std::string getName()
//    {
//        return _name;
//    }

//    int getDim()
//    {
//        return _dim;
//    }

//private:

//    std::string _name;
//    int _dim;
//    std::vector<int> _nodes;
//    Eigen::MatrixXd _values;
//};

class Variable
{
public:

    using VariablePtr = std::shared_ptr<Variable>;

    Variable(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
        _lower_bounds.setZero(1, 50);
        _upper_bounds.setZero(1, 50);
    }

    bool setNodes(std::vector<int> nodes, bool erasing)
    {
        _nodes = nodes;
        return true;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
    {

//        std::cout << "setting bounds of variable at nodes:" << std::endl;

//        for (int elem : nodes)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;

//        std::cout << "with values: " << lower_bounds << std::endl;

        for (int i = 0; i < nodes.size(); i++) {
            _lower_bounds(nodes[i]) = lower_bounds(i);
        }

        for (int i = 0; i < nodes.size(); i++) {
            _upper_bounds(nodes[i]) = upper_bounds(i);
        }

        return true;
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

    bool setNodes(std::vector<int> nodes, bool erasing)
    {
        if (erasing)
        {
            _nodes = nodes;
        }
        else
        {
            std::vector<int>::iterator p = _nodes.end();
            _nodes.insert(p, nodes.begin(), nodes.end());
        }
        return true;
    }

    std::string getName()
    {
        return _name;
    }

    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
    {
        _lower_bounds = lower_bounds;
        _upper_bounds = upper_bounds;
        return true;
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

class Item
{
public:

    using ItemPtr = std::shared_ptr<Item>;

    Item(std::string name, int dim):
        _name(name),
        _dim(dim)
    {
    }

    bool setNodes(std::vector<int> nodes, bool erasing)
    {
//        if (erasing)
//        {
        _nodes = nodes;
//        }
//        else
//        {
//            std::vector<int>::iterator p = _nodes.end();
//            _nodes.insert(p, nodes.begin(), nodes.end());
//        }
        return true;
    }

    bool setValues(Eigen::MatrixXd values)
    {
        _values = values;
        return true;
    }

    Eigen::MatrixXd getValues()
    {
        return _values;
    }

    std::string getName()
    {
        return _name;
    }

    bool assign(Eigen::MatrixXd values, std::vector<int> nodes = {})
    {
        if (!nodes.empty())
        {
            int val_i = 0;
            for (auto node : nodes)
            {
                _values(node) = values(val_i);
                val_i++;
            }
        }
        else
        {
            _values = values;
        }
        return true;
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
    Eigen::MatrixXd _values;

};

int main()
{
    int n_nodes = 50;
//    std::cout << has_set_bounds<ClassWithoutBounds>() << std::endl;
//    std::cout << has_set_bounds<Constraint>() << std::endl;

////     all of this comes from outside
//    Cost::CostPtr fake_stance_cc_1 = std::make_shared<Cost>("stance_cc_1", 1);
//    Variable::VariablePtr fake_var_1 = std::make_shared<Variable>("var_1", 3);
//    Constraint::ConstraintPtr fake_stance_c_1 = std::make_shared<Constraint>("stance_c_1", 1);
    Item::ItemPtr fake_item_1 = std::make_shared<Item>("item_1", 1);
//    Constraint::ConstraintPtr fake_stance_c_2 = std::make_shared<Constraint>("stance_c_2", 1);

//    Constraint::ConstraintPtr fake_flight_c_1 = std::make_shared<Constraint>("flight_c_1", 1);

//    Parameter::ParameterPtr fake_flight_p_1 = std::make_shared<Parameter>("flight_p_1", 3);

    // fake setting of bounds
    // Print the initialized matrix
    std::vector<int> prb_nodes(n_nodes);
//    std::vector<int> bounds_nodes;
    std::iota(prb_nodes.begin(), prb_nodes.end(), 0);

//    for (int i = 0; i < 50; ++i) {
//            bounds_nodes.push_back(i);
//        }

//    int value = 25;
//    Eigen::MatrixXd bounds_lim(1, n_nodes);
//    bounds_lim.setConstant(value);

//    fake_var_1->setBounds(-bounds_lim, bounds_lim, bounds_nodes); // third argument here is useless
//    fake_var_1->setNodes(prb_nodes, true);

    // check nodes of variable
//    std::vector<int> fake_var_nodes = fake_var_1->getNodes();
//    std::cout << "fake_var_1 active on: " << std::endl;
//    for (int elem : fake_var_nodes)
//    {
//        std::cout << elem << " ";
//    }
//    std::cout << std::endl;


    // check bounds
//    std::cout << "variable fake_var_1 has bounds: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::get<1>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;


////    for (auto element : prb_nodes)
////    {
////        std::cout << element << " ";
////    }
//    fake_flight_p_1->setNodes(prb_nodes);
//    fake_var_1->setNodes(prb_nodes, true);

    fake_item_1->setNodes(prb_nodes, true);
    Eigen::MatrixXd values_item_1 = Eigen::MatrixXd::Zero(1, n_nodes);
    fake_item_1->assign(values_item_1);

    std::cout << "fake_item_1 initial nodes: ";
    for (auto node : fake_item_1->getNodes())
    {
         std::cout << node << " ";
    }
    std::cout << std::endl;
    std::cout << "fake_item_1 initialized with values: " << fake_item_1->getValues() << std::endl;
//    Eigen::MatrixXd values_par_1 = Eigen::MatrixXd::Zero(3,50);
//    fake_flight_p_1->assign(values_par_1);

//    // ==================================================================================================================

//    // convert variable to something usable by phase_manager
//    ItemBase::Ptr stance_cc_1 = std::make_shared<Wrapper<Cost>>(fake_stance_cc_1);
//    ItemWithBoundsBase::Ptr var_1 = std::make_shared<WrapperWithBounds<Variable>>(fake_var_1);
//    ItemWithBoundsBase::Ptr stance_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_1);
//    ItemWithBoundsBase::Ptr stance_c_2 = std::make_shared<WrapperWithBounds<Constraint>>(fake_stance_c_2);
//    ItemWithBoundsBase::Ptr flight_c_1 = std::make_shared<WrapperWithBounds<Constraint>>(fake_flight_c_1);
//    ItemWithValuesBase::Ptr flight_p_1 = std::make_shared<WrapperWithValues<Parameter>>(fake_flight_p_1);
    ItemWithValuesBase::Ptr flight_p_1 = std::make_shared<WrapperWithValues<Item>>(fake_item_1);


    PhaseManager app(n_nodes);
    auto timeline_1 = app.addTimeline("first_timeline");

    int stance_duration = 5;
    Phase::Ptr stance = std::make_shared<Phase>(stance_duration, "stance");
//    Phase::Ptr flight = std::make_shared<Phase>(5, "flight");

//    std::vector<int> my_nodes;
//    my_nodes.insert(my_nodes.end(), {2, 3, 4});

    Eigen::MatrixXd initial_values(1, stance_duration);
    initial_values << 2, 2, 2, 2, 2;

    stance->addItemReference(flight_p_1, initial_values);
//    stance->addConstraint(stance_c_1); //, my_nodes);
//    stance->addConstraint(stance_c_2);



//    Eigen::MatrixXd bounds_var_1 = Eigen::MatrixXd::Zero(1,5);
//    std::vector<int> var_phase_nodes;
//    for (int i = 0; i < 5; ++i) {
//            var_phase_nodes.push_back(i);
//        }


//    stance->addVariableBounds(var_1, bounds_var_1, bounds_var_1, var_phase_nodes);


//    flight->addVariableBounds(var_1, bounds_var_1, bounds_var_1);

//    flight->addConstraint(flight_c_1);

//    flight->addParameterValues(flight_p_1, values);

//    std::cout << "variable var_1 has bounds: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;

//    //  without registering, cannot link horizon constraints to updater
    timeline_1->registerPhase(stance);
//    timeline_1->registerPhase(flight);

////    auto start_time = std::chrono::high_resolution_clock::now();
    for (int phase_num = 0; phase_num < 10; phase_num++)
    {
        timeline_1->addPhase(stance);
    }
//    std::cout << "ADD PHASE: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;


    //    this creates phasetokens
//    auto start_time = std::chrono::high_resolution_clock::now();
//    for (int i =0; i < 1000; i++)
//    {
//        timeline_1->addPhase(stance);
//    }

//    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;

//    std::cout << elapsed_time.count() << std::endl; //0.00026454 // 0.002 // 0.0008

    //    std::cout << "fake_item_1 current nodes: ";

//    for (auto node : fake_item_1->getNodes())
//    {
//         std::cout << node << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "fake_item_1 current values: " << fake_item_1->getValues() << std::endl;
////    timeline_1->addPhase(stance);

////    std::cout << "ADD PHASE: " << std::endl;
////    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
////    std::cout << std::endl;

//    for (auto phase_i : timeline_1->getPhases())
//    {
//        std::cout <<" active nodes of phase '" << phase_i->getName() << "': ";
//        for (auto node_i : phase_i->getActiveNodes())
//        {
//            std::cout << node_i << " ";
//        }
//        std::cout << std::endl;
//    }


//    for (auto pair : stance->getItemsReferenceInfo())
//    {
//        std::cout << pair.first << ": " << pair.second << std::endl;
//    }



    Eigen::MatrixXd new_values(1, stance_duration);
    new_values << 7, 7, 7, 7, 7;

    Eigen::MatrixXd new_values_1(1, stance_duration);
    new_values_1 << 4, 4, 4, 4, 4;

    Eigen::MatrixXd new_values_2(1, stance_duration);
    new_values_2 << 5, 5, 5, 5, 5;

    Eigen::MatrixXd new_values_3(1, stance_duration);
    new_values_3 << 6, 6, 6, 6, 6;


    std::cout << "fake_item_1 current values: " << fake_item_1->getValues() << std::endl;
    timeline_1->getPhases()[0]->setItemReference(flight_p_1->getName(), new_values);
    timeline_1->getPhases()[1]->setItemReference(flight_p_1->getName(), new_values_1);
    timeline_1->getPhases()[4]->setItemReference(flight_p_1->getName(), new_values_2);
    timeline_1->getPhases()[7]->setItemReference(flight_p_1->getName(), new_values_3);

    timeline_1->getPhases()[0]->update();
    timeline_1->getPhases()[1]->update();
    timeline_1->getPhases()[4]->update();
    timeline_1->getPhases()[7]->update();

    std::cout << "fake_item_1 current values: " << fake_item_1->getValues() << std::endl;
//    timeline_1->getPhases()[10]->setItemReference(flight_p_1, new_values);


//    std::cout << "fake_item_1 current values: " << fake_item_1->getValues() << std::endl;
//    for (int i =0; i < 6; i++)
//    {
//        timeline_1->shift();
//    }
//    std::cout << "fake_item_1 current values: " << fake_item_1->getValues() << std::endl;


//    timeline_1->addPhase(stance);
//    timeline_1->addPhase(flight, 10);


////    std::cout << "active Phases" << std::endl;
////    for (auto active_phase: timeline_1->getActivePhase())
////    {
////        std::cout << active_phase << std::endl;
////    }


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

////    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
////    std::cout << "elapsed time: " << elapsed_time.count() << std::endl;

// bounds not set to zero

//    timeline_1->_shift_phases();

//    std::cout << "SHIFT ONCE: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;

//    timeline_1->_shift_phases();

//    std::cout << "SHIFT ONCE: " << std::endl;
//    std::cout << std::get<0>(fake_var_1->getBounds()) << std::endl;
//    std::cout << std::endl;

//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
//    timeline_1->_shift_phases();
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
