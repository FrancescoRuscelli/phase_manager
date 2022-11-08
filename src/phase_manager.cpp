#include <phase_manager/phase_manager.h>

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

template <typename HorizonFunction>
bool Phase::addConstraint(HorizonFunction constraint, int nodes)
{
    int active_nodes = (nodes == -1) ? _n_nodes : nodes;
    _constraints[constraint] = active_nodes;

    return 1;
}

//def addCost(self, cost, nodes=None):
//    active_nodes = range(self.n_nodes) if nodes is None else nodes
//    self.costs[cost] = active_nodes

//def addVariableBounds(self, var, lower_bounds, upper_bounds, nodes=None):
//    # todo: this is not very nice, find another way? would be nice to copy the structure of addCost and addConstraint
//    active_nodes = range(self.n_nodes) if nodes is None else nodes
//    self.vars[var.getName()] = var
//    self.vars_node[var.getName()] = active_nodes
//    self.var_bounds[var.getName()] = (lower_bounds, upper_bounds)

//def addParameterValues(self, par, values, nodes=None):
//    active_nodes = range(self.n_nodes) if nodes is None else nodes
//    self.pars[par.getName()] = par
//    self.pars_node[par.getName()] = active_nodes
//    self.par_values[par.getName()] = values


PhaseToken::PhaseToken(Phase::PhasePtr phase):
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

SinglePhaseManager::SinglePhaseManager(int n_nodes, std::string name)
{

    _name = name;
    _n_nodes = n_nodes;
    _trailing_empty_nodes = _n_nodes;

}

bool SinglePhaseManager::registerPhase(Phase::PhasePtr phase)
{
    _registered_phases.push_back(phase);
}

bool SinglePhaseManager::_add_phase(Phase::PhasePtr phase, int pos)
{
//    TODO: cannot add a phase if it is not registered
    struct PhaseTokenGate : PhaseToken
    {
        PhaseTokenGate(Phase::PhasePtr phase):
            PhaseToken(phase)
        {}
    };

    PhaseToken::PhaseTokenPtr phase_token;
    phase_token = std::make_shared<PhaseTokenGate>(phase);

    if (pos == -1)
    {
        _phases.push_back(phase_token);
    }
    else
    {
//      if pos is beyond the horizon (outside of the active_phases), skip useless computation:
        if (0 <= pos <= _active_phases.size())
        {
    //          recompute empty nodes in horizon, clear active nodes of all the phases AFTER the one inserted
    //          insert phase + everything AFTER it back
            for (int i=0; i<pos; i++)
            {
                std::shared_ptr<PhaseToken> s = _phases[i];
                _trailing_empty_nodes = _n_nodes - s->_get_active_nodes().size();
                // if I'm removing phases from the current stack, i need to remove current nodes from the phases

    //        //        for (auto i = pos; i <= _phases.size(); ++i) _phases[i].reset(); // reset all the phases after the desired position
    //        //        horizon_manager.reset(); // reset the horizon_manager
    //        //        for (auto i = pos; i <= _phases.size(); ++i) _horizon_manager.update_phase(_phases[i]); // update all the "new" phases added
            }
        }
    }


    if (_trailing_empty_nodes > 0)
    {
        // set active node for each added phase
        int pos_in_horizon = _n_nodes - _trailing_empty_nodes;
        _trailing_empty_nodes -= phase->getNNodes();
        std::cout << "pos_in_horizon: " << pos_in_horizon << std::endl;


        if (_trailing_empty_nodes <= 0)
        {
            int remaining_nodes = phase_token->_get_n_nodes() + _trailing_empty_nodes;

            for (int i = 0; i<remaining_nodes; i++)
            {
                phase_token->_get_active_nodes().push_back(i);
            }
            _trailing_empty_nodes = 0;
        }
        else
        {
            for (int i = 0; i<phase_token->_get_n_nodes(); i++)
            {
                phase_token->_get_active_nodes().push_back(i);
            }
        }


        _active_phases.push_back(phase_token);



        std::cout << "number of free nodes: " << _trailing_empty_nodes << std::endl;
        std::cout << "number of node in added phase: " << phase_token->_get_n_nodes() << std::endl;
        std::cout << "number of phases: " << _phases.size() << std::endl;
        std::cout << "active nodes in added phase: ";
        for (auto j : phase_token->_get_active_nodes())
        {
            std::cout << j << " ";
        }
        std::cout << std::endl;

        for (int i=0; i<_phases.size(); i++)
        {
            std::cout << "active nodes of phase at pos '" << i << "': ";

            for (auto j : _phases[i]->_get_active_nodes())
            {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "==============" << std::endl;

//         phase_token->_update()

    }

//     update only if phase_to_add is inside horizon ( --> :current_phase)
//     [self.horizon_manager.update_phase(phase) for phase in phases_to_add[:current_phase]]
//     self.horizon_manager.set_horizon_nodes()


}

int SinglePhaseManager::_shift_phases()
{
//  if phases is empty, skip everything
    if (_phases.size() > 0)
    {
        // update active_phases with all the phases that have active nodes
        _active_phases.clear();

        for (int i=0; i<_phases.size(); i++)
        {
            if (_phases[i]->_get_active_nodes().size() > 0)
            {
                _active_phases.push_back(_phases[i]);
            }
        }


        auto last_active_phase = _active_phases.back();
//      if 'last active node' of 'last active phase' is the last of the phase, add new phase, otherwise continue to fill the phase
        int n_nodes_last_active_phase = last_active_phase->_get_n_nodes() - 1;
        bool flag_found = false;

        for (int i=0; i < last_active_phase->_get_active_nodes().size(); i++)
        {
            if (n_nodes_last_active_phase == last_active_phase->_get_active_nodes()[i])
            {
                flag_found = true;
                break;
            }
        }

        if (flag_found == true)
        {

            int n_active_phases = _active_phases.size();
////              add new active phase from list of added phases
            if (n_active_phases < _phases.size())
            {
                int elem = (_phases[n_active_phases]->_get_active_nodes().size() > 0) ? _phases[n_active_phases]->_get_active_nodes().back() + 1 : 0;
                _phases[n_active_phases]->_get_active_nodes().push_back(elem);
            }
        }
            else
            {
                // add new node to last active phase
                int elem = (last_active_phase->_get_active_nodes().size() > 0) ? last_active_phase->_get_active_nodes().back() + 1 : 0;
                last_active_phase->_get_active_nodes().push_back(elem);
            }

////    remove first element in phases
         _active_phases[0]->_get_active_nodes().erase(_active_phases[0]->_get_active_nodes().begin());

    //        burn depleted phases
        if (_phases[0]->_get_active_nodes().size() == 0)
        {
            _phases.erase(_phases.begin());
        }


        for (int i=0; i<_phases.size(); i++)
        {
            std::cout << "active nodes of phase at pos '" << i << "': ";

            for (auto j : _phases[i]->_get_active_nodes())
            {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }


////    self.horizon_manager.reset()
////   i = 0
////    for phase in self.active_phases:
////         phase.update(i)
////         i += len(phase.active_nodes)

////      [self.horizon_manager.update_phase(phase) for phase in self.active_phases]
    }
    int num_nodes = 0;
    for (int i=0; i<_active_phases.size(); i++)
    {
        num_nodes += _active_phases[i]->_get_active_nodes().size();
    }
    _trailing_empty_nodes = _n_nodes - num_nodes;

    std::cout << "current number of free nodes: " << _trailing_empty_nodes << std::endl;

}

bool SinglePhaseManager::addPhase(std::vector<Phase::PhasePtr> phases)
{
    for (int i=0; i<phases.size(); i++)
    {
        SinglePhaseManager::_add_phase(phases[i]);
        _n_phases += 1;
    }
}

bool SinglePhaseManager::addPhase(Phase::PhasePtr phase)
{
    SinglePhaseManager::_add_phase(phase);
}

Phase::PhasePtr SinglePhaseManager:: getRegisteredPhase(std::string name)
{
//    std::cout << "checking name: '" << phase_name << "'" <<std::endl;
    for (int i=0; i<_registered_phases.size(); i++)
    {
        std::string phase_name = _registered_phases[i]->getName();

        if (phase_name == name)
        {
            std::cout << "Found registered phase with name '" << name << "'" <<std::endl;
            return _registered_phases[i];
        }
    }

    std::cout << "NOT found registered phase with name '" << name << "'" <<std::endl;
    return NULL;
}

SinglePhaseManager::~SinglePhaseManager()
{

}
