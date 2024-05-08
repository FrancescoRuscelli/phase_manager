#include <phase_manager/phase.h>
#include <phase_manager/timeline.h>

Phase::Phase(Timeline& timeline, int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name),
    _timeline(timeline)
//    _phase_manager(phase_manager)

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

//bool Phase::setDuration(int new_n_nodes)
//{
//    // Stretch the duration of the phase.
//    // The function computes a stretching factor given the number of new nodes
//    // If the phase contains:
//    //  - values for the parameters: it replicates old ones
//    //  - bounds for variables: it replicates old ones
//    std::cout << "This feature is still experimental. " << std::endl;

//    double stretch_factor = static_cast<double>(new_n_nodes)/_n_nodes;
//    auto stretch_map = _stretch(_vec_nodes, stretch_factor);
//    _n_nodes = new_n_nodes;
//    _init_nodes(_n_nodes); // require to recompute vec_nodes and set_nodes;

//    for (auto& elem : _info_items_base)
//    {
//        _stretch(elem.second->nodes, stretch_factor);
//    }

//    for (auto& elem : _info_items_ref)
//    {
//        auto old_nodes = elem.second->nodes;

//        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
//        // setting the values of the new nodes to zero
//        Eigen::MatrixXd val_temp(elem.second->values.rows(), elem.second->nodes.size());

//        int initial_col = 0;
//        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
//        {
//            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
//            {
//                val_temp.col(initial_col + col_j) = elem.second->values.col(col_i);
//            }
//            initial_col += stretch_map[old_nodes[col_i]].size();
//        }

//        elem.second->values = val_temp;
//    }

//    for (auto& elem : _info_constraints)
//    {
//        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
//    }

//    for (auto& elem : _info_costs)
//    {
//        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);
//    }

//    for (auto& elem : _info_variables)
//    {
//        auto old_nodes = elem.second->nodes;
//        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);

//        // setting the bounds of the new nodes to zero
//        Eigen::MatrixXd lb_temp(elem.second->lower_bounds.rows(), elem.second->nodes.size());
//        Eigen::MatrixXd ub_temp(elem.second->upper_bounds.rows(), elem.second->nodes.size());

//        int initial_col = 0;
//        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
//        {
//            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
//            {
//                lb_temp.col(initial_col + col_j) = elem.second->lower_bounds.col(col_i);
//                ub_temp.col(initial_col + col_j) = elem.second->upper_bounds.col(col_i);
//            }
//            initial_col += stretch_map[old_nodes[col_i]].size();
//        }

//        elem.second->lower_bounds = lb_temp;
//        elem.second->upper_bounds = ub_temp;
//    }

//    for (auto& elem : _info_parameters)
//    {
//        auto old_nodes = elem.second->nodes;
//        elem.second->nodes = _extract_stretch_nodes(stretch_map, elem.second->nodes);

//        // set new values with copies of the values at the old nodes
//        // for each new values, set it with the expanded old node
//        Eigen::MatrixXd val_temp(elem.second->values.rows(), elem.second->nodes.size());

//        int initial_col = 0;
//        for (int col_i = 0; col_i < old_nodes.size(); col_i++)
//        {
//            for (int col_j = 0; col_j < stretch_map[old_nodes[col_i]].size(); col_j++)
//            {
//                val_temp.col(initial_col + col_j) = elem.second->values.col(col_i);
//            }
//            initial_col += stretch_map[old_nodes[col_i]].size();
//        }

//        elem.second->values = val_temp;

////        std::cout << "old nodes: " << std::endl;
////        for (auto node : old_nodes)
////        {
////            std::cout << node << " ";
////        }
////        std::cout << std::endl;

////        std::cout << "nodes: " << std::endl;
////        for (auto node : elem.second->nodes)
////        {
////            std::cout << node << " ";
////        }
////        std::cout << std::endl;

////        std::cout << "values: " << std::endl;
////        std::cout << elem.second->values << std::endl;

//    }

//    return true;
//}

//InfoContainer::Ptr Phase::_get_info_element(std::string elem_name)
//{
//    auto it = _elem_map.find(elem_name);
//    if (it == _elem_map.end())
//    {
//        return nullptr;
//    }


//    auto elem = it->second;
//    auto itt = _info_elements.find(elem);


//    if (itt == _info_elements.end())
//    {
//        throw std::runtime_error(std::string("There is a problem here! Contact the mantainer. Element exists in map but there are no info about it? "));
//    }

//    return itt->second;
//}


//bool Phase::setElemNodes(std::string elem_name, std::vector<int> nodes, const Eigen::MatrixXd& value_1, const Eigen::MatrixXd& value_2)
//{
//    ///
//    /// \brief set new nodes, values or bounds of element, overwriting everything.
//    /// if element only has nodes, value_1 and value_2 are ignored
//    /// if element has bounds, value_1 is lower bounds and value_2 is upper bounds (if no value_2 is specified, lower bounds == upper bounds)
//    /// if element has values, value_1 is value

//    auto info = _get_info_element(elem_name);

//    if (!info)
//    {
//        throw std::runtime_error(std::string("Element does not exist in phase."));
//    }

//    auto active_nodes = _check_active_nodes(nodes);

//    info->nodes = active_nodes;

//    if (std::dynamic_pointer_cast<BoundsContainer>(info))
//    {
//        auto bounds_info = std::dynamic_pointer_cast<BoundsContainer>(info);

//        if (value_1.rows() == 0 && value_1.cols() == 0)
//        {
//            throw std::runtime_error(std::string("No value provided for derived element."));
//        }

//        if ((value_1.cols() != nodes.size()) || (value_1.rows() != bounds_info->lower_bounds.rows()))
//        {
//            throw std::runtime_error(std::string("Wrong dimension of lower bounds inserted."));
//        }

//        bounds_info->lower_bounds = value_1;

//        if (value_2.rows() == 0 && value_2.cols() == 0)
//        {
//            bounds_info->upper_bounds = value_1;
//        }
//        else
//        {
//            if ((value_2.cols() != nodes.size()) || (value_2.rows() != bounds_info->upper_bounds.rows()))
//            {
//                throw std::runtime_error(std::string("Wrong dimension of upper bounds inserted."));
//            }

//            bounds_info->upper_bounds = value_2;
//        }
//    }
//    else if (std::dynamic_pointer_cast<ValuesContainer>(info))
//    {

//        auto value_info = std::dynamic_pointer_cast<ValuesContainer>(info);

//        if (value_1.rows() == 0 && value_1.cols() == 0)
//        {
//            throw std::runtime_error(std::string("No value provided for derived element."));
//        }

//        if (value_2.rows() != 0 && value_2.cols() != 0)
//        {
//            std::cout << "Second value has no meaning for a parameter. Ignoring." << std::endl;
//        }

//        if ((value_1.cols() != nodes.size()) || (value_1.rows() != value_info->values.rows()))
//        {
//            throw std::runtime_error(std::string("Wrong dimension of values inserted."));
//        }

//        value_info->values = value_1;

//    }
//    else
//    {
//        std::cout << "it's a base container. Changing nodes only" << std::endl;
//    }

//    return true;

//}

bool Phase::addItem(ItemBase::Ptr item, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    auto item_to_add = _add_element(item);
//    if (!_timeline.addElement(item))
//    {
//        item_to_add = _timeline.getElement(item->getName());
//    }

    // add item to timeline database

    auto it = std::make_shared<ItemManager>(item_to_add, _n_nodes, active_nodes);
    _add_element_manager(it);

    _items_base.push_back(item_to_add);

    return true;

}

bool Phase::addItemReference(ItemWithValuesBase::Ptr item_with_ref, Eigen::MatrixXd values, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    auto item_to_add = std::static_pointer_cast<ItemWithValuesBase>(_add_element(item_with_ref));


    auto it = std::make_shared<ItemReferenceManager>(item_to_add, _n_nodes, active_nodes, values);
    _add_element_manager(it);

    _items_ref.push_back(item_to_add);

    return true;

}

bool Phase::addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);

    auto item_to_add = std::static_pointer_cast<ItemWithBoundsBase>(_add_element(constraint));


    auto it = std::make_shared<ConstraintManager>(item_to_add, _n_nodes, active_nodes);
    _add_element_manager(it);

    _constraints.push_back(item_to_add);

    return true;
}


bool Phase::addCost(ItemBase::Ptr cost, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    auto item_to_add = _add_element(cost);


    auto it = std::make_shared<CostManager>(item_to_add, _n_nodes, active_nodes);
    _add_element_manager(it);

    _costs.push_back(item_to_add);

    return true;
}

bool Phase::addVariableBounds(ItemWithBoundsBase::Ptr variable, Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);
    // bounds are updated only on the active nodes

    auto item_to_add = std::static_pointer_cast<ItemWithBoundsBase>(_add_element(variable));

    auto it = std::make_shared<VariableManager>(item_to_add, _n_nodes, active_nodes, lower_bounds, upper_bounds);
    _add_element_manager(it);

    _variables.push_back(item_to_add);

    return true;
}

bool Phase::addParameterValues(ItemWithValuesBase::Ptr parameter, Eigen::MatrixXd values, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);
    // bounds are updated only on the active nodes

    auto item_to_add = std::static_pointer_cast<ItemWithValuesBase>(_add_element(parameter));

    auto it = std::make_shared<ParameterManager>(item_to_add, _n_nodes, active_nodes, values);
    _add_element_manager(it);

    _parameters.push_back(item_to_add);
    return true;
}

std::unordered_set<int> Phase::getNodesAsSet()
{
    return _nodes_as_set;
}

bool Phase::_add_element_manager(NodesManager::Ptr element)
{
    _elements.push_back(element);
    return true;
}

//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getItemLocalNodes()
//{
//    return _items_local_nodes;
//}


std::unordered_map<int, std::vector<int>> Phase::_stretch(std::vector<int> nodes, double stretch_factor)
{
    // output a map with old_node = vector of stretch_nodes
    // stretches the duration of the phase, assigning to each old node a list of new node
    // (some of these list will be empty if the stretch_factor < 1)

    std::unordered_map<int, std::vector<int>> stretch_map;

    int initial_node = 0;
    for (int node : nodes)
    {

        int new_node = static_cast<int>( (node + 1) * stretch_factor);
        std::vector<int> stretch_nodes;
        for (int it = initial_node; it < new_node; it++)
        {
            stretch_nodes.push_back(it);
            initial_node = it + 1;
        }
        stretch_map[node] = stretch_nodes;
    }

    return stretch_map;
}

std::vector<int> Phase::_extract_stretch_nodes(std::unordered_map<int, std::vector<int>> stretch_map, std::vector<int> nodes)
{
    // get vector of stretched nodes
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

std::vector<int> Phase::_check_active_nodes(std::vector<int> nodes)
{
    // check if added nodes are correct w.r.t. the nodes of the phase
    // if nodes is empty, assume all the nodes are active

    for (int num : nodes) {
        if (_nodes_as_set.find(num) == _nodes_as_set.end()) {
            throw std::invalid_argument("Node inserted ("
                                        + std::to_string(num)
                                        + ") is outside of phase nodes.");
        }
    }

    std::vector<int> active_nodes = (nodes.empty()) ? _vec_nodes : nodes;

    return active_nodes;
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

std::vector<NodesManager::Ptr> Phase::getElements()
{
    return _elements;
}

ItemBase::Ptr Phase::_add_element(ItemBase::Ptr elem)
{
    if (!_timeline.addElement(elem))
    {
        return _timeline.getElement(elem->getName());
    }
    return elem;
}

//std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getItemsInfo()
//{
//    return _info_items_base;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getItemsReferenceInfo()
//{
//    return _info_items_ref;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> Phase::getConstraintsInfo()
//{
//    return _info_constraints;
//}

//std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getCostsInfo()
//{
//    return _info_costs;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> Phase::getVariablesInfo()
//{
//    return _info_variables;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getParametersInfo()
//{
//    return _info_parameters;
//}

bool Phase::_init_nodes(int n_nodes)
{
    _nodes_as_set.clear();
    // create unordered set for phase nodes
    for (int i = 0; i < n_nodes; i++) {
        _nodes_as_set.insert(i);
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
    for (auto element : _abstract_phase->getElements())
    {
        if (auto item_ref_manager = std::dynamic_pointer_cast<ItemReferenceManager>(element))
        {

            ItemReferenceManager::Ptr item_ref_copy = std::make_unique<ItemReferenceManager>(*item_ref_manager);
            _cloned_elements[item_ref_copy->getItem()->getName()] = std::move(item_ref_copy);
        }

        if (auto parameter_manager = std::dynamic_pointer_cast<ParameterManager>(element))
        {
            auto item_ref_copy = std::make_unique<ParameterManager>(*parameter_manager);
            _cloned_elements[item_ref_copy->getItem()->getName()] = std::move(item_ref_copy);
        }

    }
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
    for (auto [name, item] : _cloned_elements)
    {
        if (name == item_name)
        {
            std::static_pointer_cast<ItemReferenceManager>(item)->setValues(values);
            return true;
        }
    }

    return false;
}

std::vector<int>& PhaseToken::_get_active_nodes()
{
    return _active_nodes;
}

Phase::Ptr PhaseToken::get_phase()
{
    return _abstract_phase;
}

std::pair<std::vector<int>, std::vector<int>> PhaseToken::_compute_horizon_nodes(std::vector<int> nodes, int initial_node)
{
    /***
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

    ***/

    std::vector<int> active_item_nodes; // Vector to store positions of elements in 'nodes' that are in '_active_nodes'

    // check which node of the item (constraint, cost, var...) is active inside the phase, given the active nodes of the phase
    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_item_nodes));



    std::vector<int> horizon_nodes(active_item_nodes.size());
    // active phase nodes             : [2 3 4 5]
    // active nodes of item in phase  : [3 4]
    // phase position in horizon      : 7
    // item node position in horizon  : 7 + 3 (node 0 of 'active nodes') = 10
    // item node position in horizon  : 7 + 4 (node 1 of 'active nodes') = 11
    for (int node_i = 0; node_i < horizon_nodes.size(); node_i++)
    {
        horizon_nodes[node_i] = initial_node + active_item_nodes[node_i];
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

        for (auto element : _abstract_phase->getElements())
        {
            auto it = element;
            if (_cloned_elements.find(element->getName()) != _cloned_elements.end())
            {
                 it = _cloned_elements[element->getName()];
            }

            auto pair_nodes = _compute_horizon_nodes(element->getSelectedNodes(), _initial_node);

//            std::cout << "updating element " << it->getName() << " (" << it->getItem() << ")" << std::endl;
            it->update(pair_nodes.first, pair_nodes.second);
        }

//        std::cout << "===========================" << std::endl;
    }
    return true;

}
