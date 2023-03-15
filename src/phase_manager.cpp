#include <phase_manager/phase_manager.h>
#include <chrono>

SinglePhaseManager::SinglePhaseManager(int n_nodes, std::string name):
    _name(name),
    _n_nodes(n_nodes),
    _trailing_empty_nodes(_n_nodes)
{
}

bool SinglePhaseManager::registerPhase(Phase::Ptr phase)
{
    // HORRIBLE
    // TODO also this should be automatic, everything that i get from phase get added

    // this is correct, I want to add horizon items (items, constraints, costs...) only one time.

    // registering phases
    std::cout << "registering phase " << phase->getName() << std::endl;
    _registered_phases.push_back(phase);

    // registering the items inside the phase, if not already registered
    for (auto item : phase->getItems())
    {
        for (auto it : _items)
        {
            if (item.first->getName() == it->getName())
            {
                return false;
            }
        }

        _items.push_back(item.first);
        return true;
    }

    for (auto item_ref : phase->getItemsReference())
    {
        for (auto it : _items_ref)
        {
            if (item_ref.first->getName() == it->getName())
            {
                return false;
            }
        }

        _items_ref.push_back(item_ref.first);
        return true;
    }

    for (auto constraint : phase->getConstraints())
    {
        for (auto it : _constraints)
        {
            if (constraint.first->getName() == it->getName())
            {
                return false;
            }
        }

        _constraints.push_back(constraint.first);
        return true;
    }

    for (auto variable : phase->getVariables())
    {
        for (auto it : _variables)
        {
            if (variable.first->getName() == it->getName())
            {
                return false;
            }
        }

        _variables.push_back(variable.first);
        return true;
    }

    for (auto cost : phase->getCosts())
    {
        for (auto it : _costs)
        {
            if (cost.first->getName() == it->getName())
            {
                return false;
            }
        }

        _costs.push_back(cost.first);
        return true;
    }

    for (auto parameter : phase->getParameters())
    {
        for (auto it : _parameters)
        {
            if (parameter.first->getName() == it->getName())
            {
                return false;
            }
        }

        _parameters.push_back(parameter.first);
        return true;
    }
    return true;
}

bool SinglePhaseManager::_add_phases(int pos)
{
    //  TODO: cannot add a phase if it is not registered (done with nullptr)

//    for (auto phase : _phases_to_add)
//    {
//        std::cout << "phase: (" << phase << ") ";
//        std::cout << phase->get_phase()->getName() << std::endl;
//        for (auto item : phase->get_phase()->getConstraints())
//        {
//            std::cout << item.first->getName() << ": ";

//            for (auto node : item.second)
//            {
//                std::cout << node << " ";
//            }

//            std::cout << std::endl;
//        }
//    }
//    std::cout << std::endl;
//    std::cout << "= = = = = = = = = = = = = =adding phase: < ";
//    for (auto phase : _phases_to_add)
//    {
//        std::cout << phase->get_phase()->getName() << " ";
//    }
//    std::cout << "> = = = = = = = = = = = = = =" << std::endl;

    if (pos == -1 || pos > _phases.size())
    {
        // add phase_tokens to stack
//        std::cout << "...adding at tail: " << "(" << _phases.size() << ")";
        _phases.insert(_phases.end(), _phases_to_add.begin(), _phases_to_add.end());
//        std::cout << " ...done." << " (" << _phases.size() << ")" << std::endl;
    }
    else
    {
        // insert phase_tokens in stack at pos
//        std::cout << "...inserting at pos: " << pos << " (" << _phases.size() << ")";
        _phases.insert(_phases.begin() + pos, _phases_to_add.begin(), _phases_to_add.end());
//        std::cout << "...done." << " (" << _phases.size() << ")" << std::endl;

        // if pos is beyond the horizon (outside of the active_phases), skip useless computation
        if (0 <= pos && pos <= _active_phases.size())
        {
            // remove all the active_phases after the position
            _active_phases.resize(pos);

            // reset the the items (holding all the active nodes)
            reset();

            // recompute empty nodes in horizon until pos
            _trailing_empty_nodes = _n_nodes;
            for (int i=0; i<pos; i++)
            {
                _trailing_empty_nodes -= _phases[i]->_get_active_nodes().size();
            }

            // update again phases before the position (before i resetted)
            int pos_in_horizon = 0;


            for (auto phase_token_i : _active_phases)
            {
                phase_token_i->_update(pos_in_horizon);
                pos_in_horizon += phase_token_i->_get_active_nodes().size();

            }

            // add tail of phases after the one added
            _phases_to_add.insert(_phases_to_add.end(), _phases.begin() + pos + 1, _phases.end());

            // remove active nodes from phases to add
            for (auto phase_to_add : _phases_to_add)
            {
                phase_to_add->_get_active_nodes().clear();
            }

        }
    }

    if (_trailing_empty_nodes > 0)
    {
        // set active node for each added phase
        for (auto phase_token_i : _phases_to_add)
        {
            int pos_in_horizon = _n_nodes - _trailing_empty_nodes;
            _trailing_empty_nodes -= phase_token_i->get_phase()->getNNodes();

            int remaining_nodes = phase_token_i->_get_n_nodes();
            if (_trailing_empty_nodes <= 0)
            {
                remaining_nodes += _trailing_empty_nodes;

                for (int i = 0; i<remaining_nodes; i++)
                {
                    phase_token_i->_get_active_nodes().push_back(i);
                }

                phase_token_i->_update(pos_in_horizon);
                _trailing_empty_nodes = 0;
                break;
            }
    
            for (int i = 0; i<remaining_nodes; i++)
            {
                phase_token_i->_get_active_nodes().push_back(i);
            }

            std::cout << "pos_in_horizon: " << pos_in_horizon << std::endl;
            std::cout << "updating phase: " << phase_token_i->get_phase()->getName() << std::endl;
            // important bit: this is where i update the phase
            phase_token_i->_update(pos_in_horizon);
//            std::cout << "adding phaseToken " << phase_token_i->getName() << " to active phases" << std::endl;
            _active_phases.push_back(phase_token_i);
        }

    }


//    std::cout << "number of free nodes: " << _trailing_empty_nodes << std::endl;

//    for (auto phase : _phases)
//    {
//        std::cout << "active nodes of phase '" << phase->get_phase()->getName() << "': ";

//        for (auto j : phase->_get_active_nodes())
//        {
//            std::cout << j << " ";
//        }
//        std::cout << std::endl;
//    }

//    std::cout << "current phases in vector: << ";
//    for (auto phase : _phases)
//    {
//       std::cout << phase->get_phase()->getName() << " ";
//    }
//    std::cout << ">> "<< std::endl;

//    std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = =" << std::endl;

    return true;


}

PhaseToken::Ptr SinglePhaseManager::_generate_phase_token(Phase::Ptr phase)
{
    // magic trick because PhaseToken construction is protected (only friends can use it)
    // in particular, the method make_shared cannot acces to the construction of PhaseToken
    struct PhaseTokenGate : PhaseToken
    {
        PhaseTokenGate(Phase::Ptr phase):
            PhaseToken(phase)
        {}
    };

    return std::make_shared<PhaseTokenGate>(phase);
}

bool SinglePhaseManager::_shift_phases()
{
    // reset all nodes
    reset();

//    auto start_time = std::chrono::high_resolution_clock::now();
//    std::cout << " ============ shifting phases ============ : " << std::endl;

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
            // add new active phase from list of added phases
            if (n_active_phases < _phases.size())
            {
                int elem = (_phases[n_active_phases]->_get_active_nodes().size() > 0) ? _phases[n_active_phases]->_get_active_nodes().back() + 1 : 0;
                _phases[n_active_phases]->_get_active_nodes().push_back(elem);
                _active_phases.push_back(_phases[n_active_phases]);
            }
        }
            else
            {
                // add new node to last active phase
                int elem = (last_active_phase->_get_active_nodes().size() > 0) ? last_active_phase->_get_active_nodes().back() + 1 : 0;
                last_active_phase->_get_active_nodes().push_back(elem);
            }


//        std::cout << "active nodes in active phases:" << std::endl;
//        for (auto phase : _active_phases)
//        {
//            std::cout << "phase: " << phase->get_phase()->getName() << " ";
//            for (auto node : phase->_active_nodes)
//            {
//                std::cout << node << " ";
//            }
//            std::cout << std::endl;
//        }
        // remove first element in phases
         _active_phases[0]->_get_active_nodes().erase(_active_phases[0]->_get_active_nodes().begin());
         // remove phase from active_phases if active nodes is empty
         if (_active_phases[0]->_get_active_nodes().empty())
         {
             _active_phases.erase(_active_phases.begin());
         }


        // burn depleted phases
        if (_phases[0]->_get_active_nodes().empty())
        {
            _phases.erase(_phases.begin());
        }


//        for (int i=0; i<_phases.size(); i++)
//        {
//            std::cout << "active nodes of phase '" << _phases[i]->get_phase()->getName() << "' at pos '" << i << "': ";

//            for (auto j : _phases[i]->_get_active_nodes())
//            {
//                std::cout << j << " ";
//            }
//            std::cout << std::endl;
//        }

        // update every active phase
        int i = 0;
        for (auto phase : _active_phases)
        {
            phase->_update(i);
            i += phase->_active_nodes.size();
        }

    }
    int num_nodes = 0;
    for (int i=0; i<_active_phases.size(); i++)
    {
        num_nodes += _active_phases[i]->_get_active_nodes().size();
    }
    _trailing_empty_nodes = _n_nodes - num_nodes;

//    std::cout << "active nodes: << ";
//    for (int i=0; i<_active_phases.size(); i++)
//    {
//        std::cout << _active_phases[i]->get_phase()->getName() <<" ";
//    }
//    std::cout << " >>" << std::endl;
//    std::cout << "current number of free nodes: " << _trailing_empty_nodes << std::endl;

//    std::chrono::duration<double> elapsed_time = std::chrono::system_clock::now() - start_time;
//    std::cout << "elapsed time: " << elapsed_time.count() << std::endl;
    return true;
}

bool SinglePhaseManager::reset()
{
    for (auto item : _items)
    {
        std::vector<int> empty_nodes;
        bool erasing = true;

        item->setNodes(empty_nodes, erasing);
    }

//    for (auto item_ref : _items_ref)
//    {
//        item_ref->clearNodes();
//    }

    for (auto constraint : _constraints)
    {
        std::vector<int> empty_nodes;
        bool erasing = true;

        constraint->setNodes(empty_nodes, erasing);
//        constraint->clearBounds(); // this cleared bounds here
    }

    for (auto cost : _costs)
    {
        std::vector<int> empty_nodes;
        bool erasing = true;
        cost->setNodes(empty_nodes, erasing);
//        cost->clearNodes();
    }

    for (auto variable : _variables)
    {
//        variable->clearNodes();
        variable->clearBounds();
    }

    for (auto parameter : _parameters)
    {
        parameter->clearValues();
    }
    return true;
}

bool SinglePhaseManager::addPhase(std::vector<Phase::Ptr> phases, int pos)
{

    for (auto phase : phases)
    {
        _phases_to_add.push_back(_generate_phase_token(phase));
    }

    _add_phases(pos);

    _phases_to_add.clear();

    return true;

}

bool SinglePhaseManager::addPhase(Phase::Ptr phase, int pos)
{
    if (!phase)
    {
        return false;
    }

    _phases_to_add.push_back(_generate_phase_token(phase));


    _add_phases(pos);
    _phases_to_add.clear();

    return true;


}

Phase::Ptr SinglePhaseManager:: getRegisteredPhase(std::string name)
{
//    std::cout << "checking name: '" << phase_name << "'" <<std::endl;
    for (int i=0; i<_registered_phases.size(); i++)
    {
        std::string phase_name = _registered_phases[i]->getName();

        if (phase_name == name)
        {
//            std::cout << "Found registered phase with name '" << name << "'" <<std::endl;
            return _registered_phases[i];
        }
    }

//    std::cout << "NOT found registered phase with name '" << name << "'" <<std::endl;
    return nullptr;
}

std::vector<Phase::Ptr> SinglePhaseManager::getRegisteredPhases()
{
    return _registered_phases;
}

int SinglePhaseManager::getEmptyNodes()
{
    return _trailing_empty_nodes;
}

std::vector<PhaseToken::Ptr> SinglePhaseManager::getActivePhases()
{
    return _active_phases;
}

SinglePhaseManager::~SinglePhaseManager()
{

}

PhaseManager::PhaseManager(int n_nodes):
    _n_nodes(n_nodes)
{

}

SinglePhaseManager::Ptr PhaseManager::addTimeline(std::string name)
{
    SinglePhaseManager::Ptr timeline = std::make_shared<SinglePhaseManager>(_n_nodes, name);
    _timelines[name]= timeline;
    return timeline;
}

std::unordered_map<std::string, SinglePhaseManager::Ptr> PhaseManager::getTimelines()
{
    return _timelines;
}

SinglePhaseManager::Ptr PhaseManager::getTimelines(std::string name)
{
    // add guards
    auto it = _timelines[name];
    return it;
}

bool PhaseManager::registerPhase(std::string name, Phase::Ptr phase)
{
    return _timelines[name]->registerPhase(phase);
}

bool PhaseManager::addPhase(std::string name, Phase::Ptr phase)
{
    return _timelines[name]->addPhase(phase);
}

int PhaseManager::getNodes()
{
    return _n_nodes;
}

bool PhaseManager::_shift_phases()
{
    for (auto timeline : _timelines)
    {
        timeline.second->_shift_phases();
    }

    return true;
}
