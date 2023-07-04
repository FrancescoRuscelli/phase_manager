#include <phase_manager/phase.h>

Phase::Phase(int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name)
{
}

std::string Phase::getName()
{
    return _name;
}

int Phase::getNNodes()
{
    return _n_nodes;
}

std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getItems()
{
    return _items_base;
}

std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getItemsReference()
{
    return _items_ref;
}

std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> Phase::getConstraints()
{
    return _constraints;
}

std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getCosts()
{
    return _costs;
}

std::unordered_map<ItemWithBoundsBase::Ptr, Phase::BoundsContainer> Phase::getVariables()
{
    return _variables;
}

std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getParameters()
{
    return _parameters;
}

std::string PhaseToken::getName()
{
    return _abstract_phase->getName();
}

std::vector<int> PhaseToken::getActiveNodes()
{
    return _active_nodes;
}

PhaseToken::PhaseToken(Phase::Ptr phase):
    _abstract_phase(phase)

{
//    _active_nodes = std::make_shared<std::vector<int>>();
    //    _n_nodes = _abstract_phase.getNNodes();
}

int PhaseToken::_get_n_nodes()
{
    return _abstract_phase->getNNodes();
}

std::vector<int>& PhaseToken::_get_active_nodes()
{
    return _active_nodes;
}

Phase::Ptr PhaseToken::get_phase()
{
    return _abstract_phase;
}

bool PhaseToken::_update_items(int initial_node)
{
    for (auto item_map : _abstract_phase->getItems())
    {
//        std::cout << "active nodes in phase: ";
//        for (auto n : item_map.second)
//        {
//            std::cout << n << " ";
//        }
//        std::cout << std::endl;
        auto pair_nodes = _compute_horizon_nodes(item_map.second, initial_node);
//        std::cout << "updating item: " << item_map.first->getName() << std::endl << "Nodes: ";
//        for (int node : pair_nodes.second)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;
        // adding nodes/resetting nodes is a duty of horizon
        item_map.first->setNodes(pair_nodes.second, false);
    }

    return true;
}

bool PhaseToken::_update_item_reference(int initial_node)
{
    for (auto item_ref_map : _abstract_phase->getItemsReference())
    {
        auto pair_nodes = _compute_horizon_nodes(item_ref_map.second.nodes, initial_node);

//        std::cout << "value assigned by user:" << item_ref_map.second.values << std::endl;

        Eigen::MatrixXd bring_me_to_eigen_3_4_val;
        bring_me_to_eigen_3_4_val.resize(item_ref_map.second.values.rows(), pair_nodes.first.size());


        for (int col_i = 0; col_i < bring_me_to_eigen_3_4_val.cols(); col_i++)
        {
            bring_me_to_eigen_3_4_val.col(col_i) = item_ref_map.second.values.col(pair_nodes.first.at(col_i));
        }

//        std::cout << "assigning values:" << bring_me_to_eigen_3_4_val << "to nodes:" << std::endl;
//        for (int node : pair_nodes.second)
//                {
//                    std::cout << node << " ";
//                }
//                std::cout << std::endl;

        item_ref_map.first->setNodes(pair_nodes.second, false);
        item_ref_map.first->assign(bring_me_to_eigen_3_4_val, pair_nodes.second);
//        item_ref_map.first->addValues(pair_nodes.second, bring_me_to_eigen_3_4_val);
//        item_ref_map.first->addValues(pair_nodes.second, item_ref_map.second.values(Eigen::indexing::all, pair_nodes.first));
    }
    return true;
}


bool PhaseToken::_update_constraints(int initial_node)
{
    for (auto cnstr_map : _abstract_phase->getConstraints())
    {
        auto pair_nodes = _compute_horizon_nodes(cnstr_map.second, initial_node);
        cnstr_map.first->setNodes(pair_nodes.second, false);
//        cnstr_map.first->addNodes(pair_nodes.second);
    }

    return true;
}

bool PhaseToken::_update_variables(int initial_node)
{
    // should this also take care of setNodes()?
    // right now only bounds

    for (auto var_map : _abstract_phase->getVariables())
    {
        auto pair_nodes = _compute_horizon_nodes(var_map.second.nodes, initial_node);

        Eigen::MatrixXd bring_me_to_eigen_3_4_lb;
        bring_me_to_eigen_3_4_lb.resize(var_map.second.lower_bounds.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < bring_me_to_eigen_3_4_lb.cols(); col_i++)
        {
            bring_me_to_eigen_3_4_lb.col(col_i) = var_map.second.lower_bounds.col(pair_nodes.first.at(col_i));
        }


        Eigen::MatrixXd bring_me_to_eigen_3_4_ub;
        bring_me_to_eigen_3_4_ub.resize(var_map.second.upper_bounds.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < bring_me_to_eigen_3_4_ub.cols(); col_i++)
        {
            bring_me_to_eigen_3_4_ub.col(col_i) = var_map.second.upper_bounds.col(pair_nodes.first.at(col_i));
        }

//            return std::make_pair(active_fun_nodes, horizon_nodes);

        var_map.first->setBounds(bring_me_to_eigen_3_4_lb,
                                 bring_me_to_eigen_3_4_ub,
                                 pair_nodes.second);

//        var_map.first->setNodes(pair_nodes.second, true);


//        var_map.first->addBounds(pair_nodes.second,
//                                 bring_me_to_eigen_3_4_lb,
//                                 bring_me_to_eigen_3_4_ub);

//        var_map.first->addBounds(pair_nodes.second,
//                                 var_map.second.lower_bounds(Eigen::indexing::all, pair_nodes.first),
//                                 var_map.second.upper_bounds(Eigen::indexing::all, pair_nodes.first));

    }

    return true;
}

bool PhaseToken::_update_costs(int initial_node)
{
    for (auto cost_map : _abstract_phase->getCosts())
    {
        auto pair_nodes = _compute_horizon_nodes(cost_map.second, initial_node);

        // add nodes without erasing the old ones
        cost_map.first->setNodes(pair_nodes.second, false);
//        cost_map.first->addNodes(pair_nodes.second);

//        std::cout << " adding horizon nodes: ";
//        for (auto elem : pair_nodes.second)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;
    }

    return true;
}

bool PhaseToken::_update_parameters(int initial_node)
{
    for (auto par_map : _abstract_phase->getParameters())
    {
        auto pair_nodes = _compute_horizon_nodes(par_map.second.nodes, initial_node);

        Eigen::MatrixXd bring_me_to_eigen_3_4_val;
        bring_me_to_eigen_3_4_val.resize(par_map.second.values.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < bring_me_to_eigen_3_4_val.cols(); col_i++)
        {
            bring_me_to_eigen_3_4_val.col(col_i) = par_map.second.values.col(pair_nodes.first.at(col_i));
        }

//        std::cout << "assigning values:" << bring_me_to_eigen_3_4_val << "to nodes:" << std::endl;
//        for (int node : pair_nodes.second)
//            {
//                std::cout << node << " ";
//            }
//            std::cout << std::endl;

        par_map.first->assign(bring_me_to_eigen_3_4_val, pair_nodes.second);

//        par_map.first->addValues(pair_nodes.second, bring_me_to_eigen_3_4_val);
//        par_map.first->addValues(pair_nodes.second, par_map.second.values(Eigen::indexing::all, pair_nodes.first));

    }

    return true;
}

std::pair<std::vector<int>, std::vector<int>> PhaseToken::_compute_horizon_nodes(std::vector<int> nodes, int initial_node)
{
    ///
    /// \brief _active_nodes: active nodes of the phase
    /// \brief nodes: nodes of the item
    /// \brief initial_node: where the item is positioned in the horizon, in terms of nodes
    /// \brief active_item_nodes: nodes of item in phase that are active
    /// \brief horizon_nodes: active nodes of item in horizon
    std::vector<int> active_item_nodes;

    // check which node of the item (constraint, cost, var...) is active inside the phase, given the active nodes of the phase
    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_item_nodes));

    std::vector<int> horizon_nodes(active_item_nodes.size());
    // active phase nodes             : [2 3 4 5]
    // active nodes of item in phase  : [3 4]
    // phase position in horizon      : 7
    // item node position in horizon  : 7 + 3 (node 0 of 'active nodes') - 2 (first node of 'active_phase_nodes') = 8
    // item node position in horizon  : 7 + 4 (node 1 of 'active nodes') - 2 (first node of 'active_phase_nodes') = 9
    for (int node_i = 0; node_i < horizon_nodes.size(); node_i++)
    {
        horizon_nodes[node_i] = initial_node + active_item_nodes[node_i] - _active_nodes[0];
    }

    return std::make_pair(active_item_nodes, horizon_nodes);

}


bool PhaseToken::_update(int initial_node)
{
    _update_items(initial_node);
    _update_item_reference(initial_node);
    _update_constraints(initial_node);
    _update_variables(initial_node);
    _update_costs(initial_node);
    _update_parameters(initial_node);
    return true;
}
