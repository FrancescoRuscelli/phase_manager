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

bool PhaseToken::_update_constraints(int initial_node)
{
    for (auto cnstr_map : _abstract_phase->getConstraints())
    {
        std::vector<int> active_nodes;

        std::set_intersection(cnstr_map.second.begin(), cnstr_map.second.end(),
                              _active_nodes.begin(), _active_nodes.end(),
                              std::back_inserter(active_nodes));

        std::cout << "active nodes of constraint: ";
        for (auto elem : active_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;


        std::vector<int> horizon_nodes(active_nodes.size());
        // active phase nodes   : [1 int2 3 4]
        // active fun nodes     : [2 3]
        // phase pos in horizon : 7
        // constr pos in horizon: 7 + 2 - 1 = 8
        std::iota(std::begin(horizon_nodes), std::end(horizon_nodes), initial_node + active_nodes[0] - _active_nodes[0]);

        cnstr_map.first->addNodes(horizon_nodes);
        std::cout << " adding horizon nodes: ";
        for (auto elem : horizon_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }

    return true;
}

bool PhaseToken::_update_variables(int initial_node)
{
    for (auto var_map : _abstract_phase->getVariables())
    {
        std::vector<int> active_nodes;

        std::set_intersection(var_map.second.nodes.begin(), var_map.second.nodes.end(),
                              _active_nodes.begin(), _active_nodes.end(),
                              std::back_inserter(active_nodes));

        std::cout << "active nodes of variable: ";
        for (auto elem : active_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;


        std::vector<int> horizon_nodes(active_nodes.size());
        std::iota(std::begin(horizon_nodes), std::end(horizon_nodes), initial_node + active_nodes[0] - _active_nodes[0]);

        var_map.first->addBounds(horizon_nodes,
                                 var_map.second.lower_bounds(Eigen::indexing::all, active_nodes),
                                 var_map.second.upper_bounds(Eigen::indexing::all, active_nodes));

    }

    return true;
}

bool PhaseToken::_update_costs(int initial_node)
{
    for (auto cost_map : _abstract_phase->getConstraints())
    {
        std::vector<int> active_nodes;

        std::set_intersection(cost_map.second.begin(), cost_map.second.end(),
                              _active_nodes.begin(), _active_nodes.end(),
                              std::back_inserter(active_nodes));

        std::cout << "active nodes of cost: ";
        for (auto elem : active_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;


        std::vector<int> horizon_nodes(active_nodes.size());
        // active phase nodes   : [1 2 3 4]
        // active fun nodes     : [2 3]
        // phase pos in horizon : 7
        // constr pos in horizon: 7 + 2 - 1 = 8
        std::iota(std::begin(horizon_nodes), std::end(horizon_nodes), initial_node + active_nodes[0] - _active_nodes[0]);

        cost_map.first->addNodes(horizon_nodes);
        std::cout << " adding horizon nodes: ";
        for (auto elem : horizon_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
    }

    return true;
}

bool PhaseToken::_update_parameters(int initial_node)
{
    for (auto par_map : _abstract_phase->getParameters())
    {
        std::vector<int> active_nodes;

        std::set_intersection(par_map.second.nodes.begin(), par_map.second.nodes.end(),
                              _active_nodes.begin(), _active_nodes.end(),
                              std::back_inserter(active_nodes));

        std::cout << "active nodes of parameter: ";
        for (auto elem : active_nodes)
        {
            std::cout << elem << " ";
        }
        std::cout << std::endl;


        std::vector<int> horizon_nodes(active_nodes.size());
        std::iota(std::begin(horizon_nodes), std::end(horizon_nodes), initial_node + active_nodes[0] - _active_nodes[0]);

        par_map.first->addValues(horizon_nodes, par_map.second.values(Eigen::indexing::all, active_nodes));

    }

    return true;
}

bool PhaseToken::_update(int initial_node)
{
    _update_constraints(initial_node);
    _update_variables(initial_node);
    _update_costs(initial_node);
    _update_parameters(initial_node);
    return true;
}
