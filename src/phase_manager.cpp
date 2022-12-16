#include <phase_manager/phase_manager.h>
#include <phase_manager/horizon_manager.h>

#include <chrono>

SinglePhaseManager::SinglePhaseManager(int n_nodes, std::string name):
    _name(name),
    _n_nodes(n_nodes),
    _trailing_empty_nodes(_n_nodes)
{
    _horizon_manager = std::make_unique<HorizonManager>();
}

bool SinglePhaseManager::registerPhase(Phase::Ptr phase)
{
    _registered_phases.push_back(phase);

    for (auto constraint : phase->getConstraints())
    {
        _horizon_manager->addConstraint(constraint.first);
    }

    for (auto variable : phase->getVariables())
    {
        _horizon_manager->addVariable(variable.first);
    }

    for (auto cost : phase->getCosts())
    {
        _horizon_manager->addCost(cost.first);
    }

    for (auto parameter : phase->getParameters())
    {
        _horizon_manager->addParameter(parameter.first);
    }
    return true;
}

bool SinglePhaseManager::_add_phases(int pos)
{
    //  TODO: cannot add a phase if it is not registered

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

    if (pos == -1)
    {
        // add phase_tokens to stack
        _phases.insert(_phases.end(), _phases_to_add.begin(), _phases_to_add.end());
    }
    else
    {
        // insert phase_tokens in stack at pos
        _phases.insert(_phases.begin() + pos, _phases_to_add.begin(), _phases_to_add.end());

        // if pos is beyond the horizon (outside of the active_phases), skip useless computation
        if (0 <= pos && pos <= _active_phases.size())
        {

            // remove all the active_phases after the position
            _active_phases.resize(pos);

            // reset the horizon_manager (holding all the active nodes)
            _horizon_manager->reset();

            // recompute empty nodes in horizon until pos
            _trailing_empty_nodes = _n_nodes;
            for (int i=0; i<pos; i++)
            {
                _trailing_empty_nodes -= _phases[i]->_get_active_nodes().size();
            }

            // update again phases before the position (before i resetted horizon_manager)
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
    
//            std::cout << "pos_in_horizon: " << pos_in_horizon << std::endl;

//            std::cout << "updating phase: " << phase_token_i->get_phase()->getName() << std::endl;
            // important bit: this is where i update the phase
            phase_token_i->_update(pos_in_horizon);
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


    _horizon_manager->flush();
//    std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = =" << std::endl;

    return true;

//     update only if phase_to_add is inside horizon ( --> :current_phase)
//     [self.horizon_manager.update_phase(phase) for phase in phases_to_add[:current_phase]]
//     self.horizon_manager.set_horizon_nodes()


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

    _horizon_manager->reset();

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

        int i = 0;
        for (auto phase : _active_phases)
        {
            phase->_update(i);
            i += phase->_active_nodes.size();
        }

        _horizon_manager->flush();

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
    return NULL;
}

//std::vector<PhaseToken::Ptr> SinglePhaseManager::getActivePhase()
//{
//    std::vector<Phase::Ptr> registered_active_phases;
//    for (auto active_phase : _active_phases)
//    {

//        registered_active_phases.back(_pha)
//    }
//}

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

bool PhaseManager::registerPhase(std::string name, Phase::Ptr phase)
{
    return _timelines[name]->registerPhase(phase);
}

bool PhaseManager::addPhase(std::string name, Phase::Ptr phase)
{
    return _timelines[name]->addPhase(phase);
}

bool PhaseManager::_shift_phases()
{
    for (auto timeline : _timelines)
    {
        timeline.second->_shift_phases();
    }

    return true;
}
