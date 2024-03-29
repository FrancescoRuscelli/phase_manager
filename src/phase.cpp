#include <phase_manager/phase.h>

Phase::Phase(int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name)

{
    _init_nodes(_n_nodes);

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
    // Stretch the duration of the phase.
    // The function computes a stretching factor given the number of new nodes
    // If the phase contains:
    //  - values for the parameters: it replicates old ones
    //  - bounds for variables: it replicates old ones
    std::cout << "This feature is still experimental. " << std::endl;

    double stretch_factor = static_cast<double>(new_n_nodes)/_n_nodes;
    auto stretch_map = _stretch(_vec_nodes, stretch_factor);
    _n_nodes = new_n_nodes;
    _init_nodes(_n_nodes); // require to recompute vec_nodes and set_nodes;

    for (auto& elem : _info_items_base)
    {
        _stretch(elem.second->nodes, stretch_factor);
    }

    for (auto& elem : _info_items_ref)
    {
        auto old_nodes = elem.second->nodes;

        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
        // setting the values of the new nodes to zero
        Eigen::MatrixXd val_temp(elem.second->values.rows(), elem.second->nodes.size());

        int initial_col = 0;
        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
        {
            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
            {
                val_temp.col(initial_col + col_j) = elem.second->values.col(col_i);
            }
            initial_col += stretch_map[old_nodes[col_i]].size();
        }

        elem.second->values = val_temp;
    }

    for (auto& elem : _info_constraints)
    {
        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
    }

    for (auto& elem : _info_costs)
    {
        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
    }

    for (auto& elem : _info_variables)
    {
        auto old_nodes = elem.second->nodes;
        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);

        // setting the bounds of the new nodes to zero
        Eigen::MatrixXd lb_temp(elem.second->lower_bounds.rows(), elem.second->nodes.size());
        Eigen::MatrixXd ub_temp(elem.second->upper_bounds.rows(), elem.second->nodes.size());

        int initial_col = 0;
        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
        {
            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
            {
                lb_temp.col(initial_col + col_j) = elem.second->lower_bounds.col(col_i);
                ub_temp.col(initial_col + col_j) = elem.second->upper_bounds.col(col_i);
            }
            initial_col += stretch_map[old_nodes[col_i]].size();
        }

        elem.second->lower_bounds = lb_temp;
        elem.second->upper_bounds = ub_temp;
    }

    for (auto& elem : _info_parameters)
    {
        auto old_nodes = elem.second->nodes;
        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);

        // set new values with copies of the values at the old nodes
        // for each new values, set it with the expanded old node
        Eigen::MatrixXd val_temp(elem.second->values.rows(), elem.second->nodes.size());

        int initial_col = 0;
        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
        {
            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
            {
                val_temp.col(initial_col + col_j) = elem.second->values.col(col_i);
            }
            initial_col += stretch_map[old_nodes[col_i]].size();
        }

        elem.second->values = val_temp;

//        std::cout << "old nodes: " << std::endl;
//        for (auto node : old_nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

//        std::cout << "nodes: " << std::endl;
//        for (auto node : elem.second->nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

//        std::cout << "values: " << std::endl;
//        std::cout << elem.second->values << std::endl;

    }

    return true;
}

InfoContainer::Ptr Phase::_get_info_element(std::string elem_name)
{
    auto it = _elem_map.find(elem_name);
    if (it == _elem_map.end())
    {
        return nullptr;
    }


    auto elem = it->second;
    auto itt = _info_elements.find(elem);


    if (itt == _info_elements.end())
    {
        throw std::runtime_error(std::string("There is a problem here! Contact the mantainer. Element exists in map but there are no info about it? "));
    }

    return itt->second;
}


bool Phase::setElemNodes(std::string elem_name, std::vector<int> nodes, const Eigen::MatrixXd& value_1, const Eigen::MatrixXd& value_2)
{
    ///
    /// \brief set new nodes, values or bounds of element, overwriting everyhting.
    /// if element only has nodes, value_1 and value_2 are ignored
    /// if element has bounds, value_1 is lower bounds and value_2 is upper bounds (if no value_2 is specified, lower bounds == upper bounds)
    /// if element has values, value_1 is value

    auto info = _get_info_element(elem_name);

    if (!info)
    {
        throw std::runtime_error(std::string("Element does not exist in phase."));
    }

    auto active_nodes = _check_active_nodes(nodes);

    info->nodes = active_nodes;

    if (std::dynamic_pointer_cast<BoundsContainer>(info))
    {
        auto bounds_info = std::dynamic_pointer_cast<BoundsContainer>(info);

        if (value_1.rows() == 0 && value_1.cols() == 0)
        {
            throw std::runtime_error(std::string("No value provided for derived element."));
        }

        if ((value_1.cols() != nodes.size()) || (value_1.rows() != bounds_info->lower_bounds.rows()))
        {
            throw std::runtime_error(std::string("Wrong dimension of lower bounds inserted."));
        }

        bounds_info->lower_bounds = value_1;

        if (value_2.rows() == 0 && value_2.cols() == 0)
        {
            bounds_info->upper_bounds = value_1;
        }
        else
        {
            if ((value_2.cols() != nodes.size()) || (value_2.rows() != bounds_info->upper_bounds.rows()))
            {
                throw std::runtime_error(std::string("Wrong dimension of upper bounds inserted."));
            }

            bounds_info->upper_bounds = value_2;
        }
    }
    else if (std::dynamic_pointer_cast<ValuesContainer>(info))
    {

        auto value_info = std::dynamic_pointer_cast<ValuesContainer>(info);

        if (value_1.rows() == 0 && value_1.cols() == 0)
        {
            throw std::runtime_error(std::string("No value provided for derived element."));
        }

        if (value_2.rows() != 0 && value_2.cols() != 0)
        {
            std::cout << "Second value has no meaning for a parameter. Ignoring." << std::endl;
        }

        if ((value_1.cols() != nodes.size()) || (value_1.rows() != value_info->values.rows()))
        {
            throw std::runtime_error(std::string("Wrong dimension of values inserted."));
        }

        value_info->values = value_1;

    }
    else
    {
        std::cout << "it's a base container. Changing nodes only" << std::endl;
    }

    return true;

}

std::unordered_set<int> Phase::getSetNodes()
{
    return _set_nodes;
}

//bool Phase::setElemNodes(std::string elem_name, std::vector<int> new_nodes, Eigen::MatrixXd upper_bounds)
//{
//    if (info->getType() == "bounds")
//    {
//        std::cout << "it's a bound container. Should do something with bounds" << std::endl;
//        auto bounds_info = std::dynamic_pointer_cast<BoundsContainer>(info);
//        std::cout << bounds_info->upper_bounds << std::endl;

//        if (values != nullptr)
//        {
//            bounds_info->upper_bounds = new_values;
//        }
//        else
//        {
//            throw std::cout << "diocane" << std::endl;
//        }

//    }

//    if (info->getType() == "values")
//    {
//        std::cout << "it's a values container. Should do something with values" << std::endl;
////        std::cout << info->values << std::endl;
//    }

//    // what if there are bounds or values?

//    return true;
//}

std::unordered_map<int, std::vector<int>> Phase::_stretch(std::vector<int> nodes, double stretch_factor)
{
    // stretches the duration of the phase, assigning to each old node a list of new node
    // (some of these list will be empty if the stretch_factor < 1)

    std::unordered_map<int, std::vector<int>> stretch_map;

    int initial_node = 0;
    for (int element : nodes)
    {

        int new_element = static_cast<int>( (element + 1) * stretch_factor);
        std::vector<int> stretch_nodes;
        for (int it = initial_node; it < new_element; it++)
        {
            stretch_nodes.push_back(it);
            initial_node = it + 1;
        }
        stretch_map[element] = stretch_nodes;
    }

    return stretch_map;
}

std::vector<int> Phase::_extract_stretch_nodes(std::unordered_map<int, std::vector<int>> stretch_map, std::vector<int> nodes)
{
    std::vector<int> new_nodes;

    for (auto node : nodes)
    {
        for (auto new_node : stretch_map[node])
        {
            new_nodes.push_back(new_node);
        }
    }

    return new_nodes;
}

std::vector<ItemBase::Ptr> Phase::getItems()
{
    return _items_base;
}

std::vector<ItemWithValuesBase::Ptr> Phase::getItemsReference()
{
    return _items_ref;
}

std::vector<ItemWithBoundsBase::Ptr> Phase::getConstraints()
{
    return _constraints;
}

std::vector<ItemBase::Ptr> Phase::getCosts()
{
    return _costs;
}

std::vector<ItemWithBoundsBase::Ptr> Phase::getVariables()
{
    return _variables;
}

std::vector<ItemWithValuesBase::Ptr> Phase::getParameters()
{
    return _parameters;
}

std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getItemsInfo()
{
    return _info_items_base;
}

std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getItemsReferenceInfo()
{
    return _info_items_ref;
}

std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> Phase::getConstraintsInfo()
{
    return _info_constraints;
}

std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getCostsInfo()
{
    return _info_costs;
}

std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> Phase::getVariablesInfo()
{
    return _info_variables;
}

std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getParametersInfo()
{
    return _info_parameters;
}

bool Phase::_init_nodes(int n_nodes)
{
    _set_nodes.clear();
    // create unordered set for phase nodes
    for (int i = 0; i < n_nodes; i++) {
        _set_nodes.insert(i);
    }

    _vec_nodes.clear();
    for (int i = 0; i < n_nodes; i++) {
        _vec_nodes.push_back(i);
    }

    return true;
    //    std::cout << "set nodes: " << std::endl;
    //    for (auto node : _set_nodes) {
    //        std::cout << node << " ";
    //    }
    //    std::cout << std::endl;

    //    std::cout << "vec nodes: " << std::endl;
    //    for (auto node : _vec_nodes) {
    //        std::cout << node << " ";
    //    }
    //    std::cout << std::endl;

}


//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getItems()
//{
//    return _items_base;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getItemsReference()
//{
//    return _items_ref;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> Phase::getConstraints()
//{
//    return _constraints;
//}

//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getCosts()
//{
//    return _costs;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, Phase::BoundsContainer> Phase::getVariables()
//{
//    return _variables;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getParameters()
//{
//    return _parameters;
//}


PhaseToken::PhaseToken(Phase::Ptr phase):
    _abstract_phase(phase),
    _initial_node(0)
{
    // copy construct a private info container
    for (auto pair : _abstract_phase->getItemsReferenceInfo())
    {
        ValuesContainer::Ptr item_ref_copy = std::make_unique<ValuesContainer>(pair.second);

//        values_copy(pair.second);
//        values_copy->values = pair.second->values;
//        values_copy->nodes = pair.second->nodes;
        _info_items_ref_token[pair.first] = item_ref_copy;
    }

    for (auto pair: _abstract_phase->getParametersInfo())
    {
        ValuesContainer::Ptr par_ref_copy = std::make_unique<ValuesContainer>(pair.second);
        _info_parameters_token[pair.first] = par_ref_copy;
    }

    //    _active_nodes = std::make_shared<std::vector<int>>();
    //    _n_nodes = _abstract_phase.getNNodes();

//    for (auto element : _info_items_ref_token)
//    {
//        std::cout << element.first->getName() << std::endl;
//    }
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

bool PhaseToken::setItemReference(std::string item_name, Eigen::MatrixXd values)
{
    // set item reference for independent phase token
    bool bool_found = false;
    for (auto item : _info_items_ref_token)
    {
        if (item.first->getName() == item_name)
        {
            int col_values = 0;
            // set reference of the specified nodes
            for (auto node : _info_items_ref_token[item.first]->nodes)
            {
                _info_items_ref_token[item.first]->values.col(node) = values.col(col_values);
                col_values++;
            }
            bool_found = true;
        }
    }

//    _update_item_reference(_initial_node);
    return bool_found;
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
    for (auto item_map : _abstract_phase->getItemsInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(item_map.second->nodes, initial_node);
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
    // update item with references that are independent to the abstract phase, but copied (and changeable) inside the phase token.
    for (auto item_ref_map : _info_items_ref_token)
//    for (auto item_ref_map : _abstract_phase->getItemsReferenceInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(item_ref_map.second->nodes, initial_node);

//        std::cout << "item to which values are assigned: " << item_ref_map.first << std::endl;
//        std::cout << "value assigned by user: " << item_ref_map.second->values << " (dim: " << item_ref_map.second->values.size() << ")" << std::endl;
//        std::cout << "item nodes: ";
//        for (auto node : item_ref_map.second->nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

        // when shifting, the pair_nodes.first shifts too. I need to set the correct values from --> item_ref_map.second->values
        // example: when the phase is half in the past, only the phase nodes [3, 4, 5] are active. Hence, I have to take the values at those position in the matrix item_ref_map.second

        // create a matrix of values to assign
        Eigen::MatrixXd bring_me_to_eigen_3_4_val;
        bring_me_to_eigen_3_4_val.resize(item_ref_map.second->values.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < bring_me_to_eigen_3_4_val.cols(); col_i++)
        {
            bring_me_to_eigen_3_4_val.col(col_i) = item_ref_map.second->values.col(pair_nodes.first[col_i]);
        }

        item_ref_map.first->setNodes(pair_nodes.second, false);
        item_ref_map.first->assign(bring_me_to_eigen_3_4_val, pair_nodes.second);
//        item_ref_map.first->addValues(pair_nodes.second, bring_me_to_eigen_3_4_val);
//        item_ref_map.first->addValues(pair_nodes.second, item_ref_map.second.values(Eigen::indexing::all, pair_nodes.first));
    }
    return true;
}


bool PhaseToken::_update_constraints(int initial_node)
{
    for (auto cnstr_map : _abstract_phase->getConstraintsInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(cnstr_map.second->nodes, initial_node);
        cnstr_map.first->setNodes(pair_nodes.second, false);
//        cnstr_map.first->addNodes(pair_nodes.second);
    }

    return true;
}

bool PhaseToken::_update_variables(int initial_node)
{
    // should this also take care of setNodes()?
    // right now only bounds
    for (auto var_map : _abstract_phase->getVariablesInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(var_map.second->nodes, initial_node);
        // take only the active nodes (columns) from the bounds matrix and set it to "lb"

        Eigen::MatrixXd lb(var_map.second->lower_bounds.rows(), pair_nodes.first.size());
        Eigen::MatrixXd ub(var_map.second->upper_bounds.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < pair_nodes.first.size(); col_i++)
        {
            // find the column of the values matrix corresponding to the active nodes
            auto itr = std::find(var_map.second->nodes.begin(), var_map.second->nodes.end(), pair_nodes.first[col_i]);
            int index = std::distance(var_map.second->nodes.begin(), itr);
            lb.col(col_i) = var_map.second->lower_bounds.col(index);
            ub.col(col_i) = var_map.second->upper_bounds.col(index);
        }

        var_map.first->setBounds(lb,
                                 ub,
                                 pair_nodes.second);

//        var_map.first->addBounds(pair_nodes.second,
//                                 var_map.second.lower_bounds(Eigen::indexing::all, pair_nodes.first),
//                                 var_map.second.upper_bounds(Eigen::indexing::all, pair_nodes.first));

    }

    return true;
}

bool PhaseToken::_update_costs(int initial_node)
{
    for (auto cost_map : _abstract_phase->getCostsInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(cost_map.second->nodes, initial_node);

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
    for (auto par_map : _abstract_phase->getParametersInfo())
    {
        auto pair_nodes = _compute_horizon_nodes(par_map.second->nodes, initial_node);

//        std::cout << " par_map nodes: ";
//        for (auto elem : par_map.second->nodes)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;

        Eigen::MatrixXd val(par_map.second->values.rows(), pair_nodes.first.size());

        for (int col_i = 0; col_i < pair_nodes.first.size(); col_i++)
        {
            // find the column of the values matrix corresponding to the active nodes
            auto itr = std::find(par_map.second->nodes.begin(), par_map.second->nodes.end(), pair_nodes.first[col_i]);
            int index = std::distance(par_map.second->nodes.begin(), itr);
            val.col(col_i) = par_map.second->values.col(index);
        }

//        std::cout << " adding horizon nodes: ";
//        for (auto elem : pair_nodes.second)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;

//        std::cout << "val:" << std::endl;
//        std::cout << val << std::endl;

        // this work when shifting because the phase_manager reset all the nodes before
        // here it assign nodes only where needed
        par_map.first->assign(val, pair_nodes.second);
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
     * initial_node: where the phase is positioned in the horizon, in terms of node
     * active_item_nodes: item nodes in phase that are active (intersection between active node of the phase and nodes of the item in the phase)
     * horizon_nodes: active nodes of item in horizon

    */

    std::vector<int> active_item_nodes; // Vector to store positions of elements in 'nodes' that are in '_active_nodes'

    // check which node of the item (constraint, cost, var...) is active inside the phase, given the active nodes of the phase
    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_item_nodes));

//    for (int i = 0; i < nodes.size(); ++i) {
//        if (get_phase()->getSetNodes().count(nodes[i]) > 0) {
//            // Element found in 'get_phase()->getSetNodes()', store its position in vector 'active_item_nodes'
//            active_item_nodes.push_back(i);
//        }
//    }


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

bool PhaseToken::_set_position(int initial_node)
{
    _initial_node = initial_node;
    return true;
}

//bool PhaseToken::_set_position(int initial_node)
//{
//    _initial_node = initial_node;
//}

bool PhaseToken::update()
{
    /*
     * update items contained in phase in horizon based on the position of the phase
     */

//    std::cout << "updating phase: '" << getName() << "' at node: " << _initial_node << std::endl;
    if (!_active_nodes.empty())
    {
        _update_items(_initial_node);
        _update_item_reference(_initial_node);
        _update_constraints(_initial_node);
        _update_variables(_initial_node);
        _update_costs(_initial_node);
        _update_parameters(_initial_node);
    }
    return true;

}
