#include <phase_manager/phase.h>

Phase::Phase(int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name)
{

//    setMap("items", &_items_base);
//    _elem_map["items_ref"] = _items_ref;
//    _elem_map["constraints"] = _constraints;
//    _elem_map["costs"] = _costs;
//    _elem_map["variables"] = _variables;
//    _elem_map["parameters"] = _parameters;

}

std::string Phase::getName()
{
    return _name;
}

int Phase::getNNodes()
{
    return _n_nodes;
}

bool Phase::setDuration(int new_n_nodes)
{
    double stretch_factor = static_cast<double>(new_n_nodes)/_n_nodes;

    _n_nodes = new_n_nodes;

    for (auto& elem : _items_base)
    {
        _stretch(elem.second, stretch_factor);
    }

    for (auto& elem : _constraints)
    {
        _stretch(elem.second, stretch_factor);

    }

    // todo: add all the other containers

//    _items_base; OK
//    _items_ref; X
//    _constraints; OK
//    _costs; X
//    _variables; X
//    _parameters; X

    return true;
}

void Phase::_stretch(std::vector<int>& nodes, double stretch_factor)
{


    int new_initial_node = static_cast<int>(std::round(nodes[0] * stretch_factor));
    int new_duration = static_cast<int>(std::round(nodes.size() * stretch_factor));

    nodes.clear();

    for (auto it = new_initial_node; it < new_duration + new_initial_node; it++)
    {
        if (it <= (_n_nodes - 1))
            nodes.push_back(it);
    }

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

const std::vector<int>& PhaseToken::getActiveNodes()
{
    /*
     * get the active nodes of the phase (WARNING: relative nodes, the are NOT the absolute nodes in the horizon)
     */
    return _active_nodes;
}

const int PhaseToken::getPosition()
{
    return _initial_node;
}

const int PhaseToken::getNNodes()
{
    return _abstract_phase->getNNodes();
}

PhaseToken::PhaseToken(Phase::Ptr phase):
    _abstract_phase(phase)

{
//    _active_nodes = std::make_shared<std::vector<int>>();
    //    _n_nodes = _abstract_phase.getNNodes();
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
        auto pair_nodes = _compute_horizon_nodes(item_map.second, initial_node);
//        std::cout << "updating item: '" << item_map.first->getName() << "'. Nodes: ";
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
    /*
     * input:
     * item nodes inside the phase (relative nodes)
     * initial node of the phase (position of phase in horizon)
     * output:
     * active nodes of item relative to the phase
     * absolute nodes in horizon

     * _active_nodes: active nodes of the phase (relative nodes)
     * nodes: nodes of the item (relative nodes)
     * initial_node: where the phase is positioned in the horizon, in terms of nodes
     * active_item_nodes: item nodes in phase that are active (intersection between active node of the phase and nodes of the item in the phase)
     * horizon_nodes: active nodes of item in horizon

    */

    std::vector<int> active_item_nodes;

    // check which node of the item (constraint, cost, var...) is active inside the phase, given the active nodes of the phase
    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_item_nodes));

    std::vector<int> horizon_nodes(active_item_nodes.size());
    // active phase nodes             : [2 3 4 5]
    // active nodes of item in phase  : [3 4]
    // phase position in horizon      : 7
    // item node position in horizon  : 7 + 3 (node 0 of 'active nodes') - 2 (first node of '_active_nodes') = 8
    // item node position in horizon  : 7 + 4 (node 1 of 'active nodes') - 2 (first node of '_active_nodes') = 9
    for (int node_i = 0; node_i < horizon_nodes.size(); node_i++)
    {
        horizon_nodes[node_i] = initial_node + active_item_nodes[node_i] - _active_nodes[0];
    }

    return std::make_pair(active_item_nodes, horizon_nodes);

}

//bool PhaseToken::_set_position(int initial_node)
//{
//    _initial_node = initial_node;
//}

bool PhaseToken::_update(int initial_node)
{
    /*
     * update items contained in phase in horizon based on the position of the phase
     */
    _initial_node = initial_node;

    if (!_active_nodes.empty())
    {
        _update_items(initial_node);
        _update_item_reference(initial_node);
        _update_constraints(initial_node);
        _update_variables(initial_node);
        _update_costs(initial_node);
        _update_parameters(initial_node);
    }
    return true;

}
