#include <phase_manager/phase_manager.h>
#include <chrono>

SinglePhaseManager::SinglePhaseManager(int n_nodes, std::string name):
    _name(name),
    _n_nodes(n_nodes),
    _trailing_empty_nodes(_n_nodes),
    _last_node(0)
{
}

template <typename T, typename U>
bool _register_items_from_phase(std::vector<T>& container, std::vector<U> items)
{
    bool flag_repetition = false;
    // registering the items inside the phase, if not already registered
    for (auto item : items)
    {
        for (auto it : container)
        {
            if (item->getName() == it->getName())
            {
                flag_repetition = true;
                break;
            }
        }

        if (flag_repetition)
        {
            flag_repetition = false;
            continue;
        }

        container.push_back(item);
    }
    return true;
}

bool SinglePhaseManager::registerPhase(Phase::Ptr phase)
{
    // registering phases
    // todo: make it faster

//    std::cout << "registering phase " << phase->getName() << std::endl;
    _registered_phases.push_back(phase);

    _register_items_from_phase(_items,  phase->getItems());
    _register_items_from_phase(_items_ref,  phase->getItemsReference());
    _register_items_from_phase(_constraints,  phase->getConstraints());
    _register_items_from_phase(_variables,  phase->getVariables());
    _register_items_from_phase(_costs,  phase->getCosts());
    _register_items_from_phase(_parameters,  phase->getParameters());

    return true;
}

bool SinglePhaseManager::_add_phases(int pos, bool absolute_position)
{
    //  TODO: cannot add a phase if it is not registered (done with nullptr)

//    for (auto phase : _phases_to_add)
//    {
//        std::cout << "adding phase: (" << phase << ") ";
//        std::cout << phase->get_phase()->getName() << std::endl;
//        for (auto item : phase->get_phase()->getConstraints())
//        {
//            std::cout << "constraint: ";
//            std::cout << item.first->getName() << ": ";

//            for (auto node : item.second)
//            {
//                std::cout << node << " ";
//            }

//            std::cout << std::endl;
//        }

//        for (auto item : phase->get_phase()->getItems())
//        {
//            std::cout << "item: ";
//            std::cout << item.first->getName() << ": ";

//            for (auto node : item.second)
//            {
//                std::cout << node << " ";
//            }

//            std::cout << std::endl;
//        }

//        for (auto item : phase->get_phase()->getVariables())
//        {
//            std::cout << "variable: ";
//            std::cout << item.first->getName() << ": ";

//            for (auto node : item.second.nodes)
//            {
//                std::cout << node << " ";
//            }

//            std::cout << std::endl;
//        }
//    }
//    std::cout << "=================================== "<< std::endl;

      // SHORT VERSION LOGGER
//    std::cout << "= = = = = = = = = = = = = =adding phase: < ";
//    for (auto phase : _phases_to_add)
//    {
//        std::cout << phase->get_phase()->getName() << " ";
//    }
//    std::cout << "> = = = = = at node: ";
//    std::cout << _last_node;
//    std::cout << " = = = = = = = = =" << std::endl;
    if (absolute_position)
    {
        pos = _check_absolute_position(pos);
    }

    // fill _phases_to_add, a vector of phases to add to the horizon
    if (pos == -1 || pos > _phases.size())
    {
        // add phase_tokens in temporary container (_phase_to_add) to stack
        std::cout << "...adding at tail: " << "(" << _phases.size() << ")";
        _phases.insert(_phases.end(), _phases_to_add.begin(), _phases_to_add.end());
        std::cout << " ...done." << " (" << _phases.size() << ")" << std::endl;
    }
    else
    {   
        _insert_phases(pos);
    }


    // compute active nodes inside the added phases
    for (auto phase_token_i : _phases_to_add)
    {
        int active_nodes = phase_token_i->getNNodes();

        // set active node for each added phase
//        if (_trailing_empty_nodes > 0)
        if (_last_node < _n_nodes)
        {
            // if _trailing_empty_nodes is bigger than zero, the phase is active (even if its tail falls outside the horizon)
            std::cout << "adding phaseToken " << phase_token_i->getName() << " to active phases" << std::endl;
            _active_phases.push_back(phase_token_i);

            std::cout << "active_nodes: " << active_nodes << std::endl;

            if (_last_node + active_nodes >= _n_nodes)
            {
                active_nodes -= _last_node + active_nodes - _n_nodes;
            }

            for (int i = 0; i<active_nodes; i++)
            {
                phase_token_i->_get_active_nodes().push_back(i);
            }
//
        }

        std::cout << "updating phase: '" << phase_token_i->get_phase()->getName() << "' at node: " << _last_node << std::endl;
//        important bit: this is where i update the phase
        phase_token_i->_update(_last_node);
        _last_node += phase_token_i->getNNodes();
        std::cout << "last node: " << _last_node << std::endl;

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

bool SinglePhaseManager::_insert_phases(int pos)
{
    // insert phase_tokens of temporary container (_phase_to_add) in stack at posistion 'pos'
//        std::cout << "...inserting at pos: " << pos << " (" << _phases.size() << ")";
    _phases.insert(_phases.begin() + pos, _phases_to_add.begin(), _phases_to_add.end());
//        std::cout << " ...done." << " (" << _phases.size() << ")" << std::endl;


    if (pos >= 0) //        if (0 <= pos && pos <= _active_phases.size())
    {
        // if 'pos' is beyond the horizon (outside of the active_phases), skip useless computation
        if (pos <= _active_phases.size())
        {
            // remove all the active_phases after the position 'pos'
            _active_phases.resize(pos);

            // reset the items (holding all the active nodes)
            // TODO: should I do it only for the items before pos?
            _reset();

            // recompute empty nodes in horizon until position 'pos'
//            _trailing_empty_nodes = _n_nodes;
//            for (int i=0; i<pos; i++)
//            {
//                _trailing_empty_nodes -= _phases[i]->_get_active_nodes().size();

//            }
            // update again phases before the position 'pos' (before i resetted)
            int last_active_node = 0;

            for (auto phase_token_i : _active_phases)
            {
                phase_token_i->_update(last_active_node);
                last_active_node += phase_token_i->_get_active_nodes().size();
            }
        }

        // reset the last_node at the last element position before the inserted.
        _last_node = 0;
        for (int it=0; it < pos; it++)
        {
            _last_node += _phases[it]->getNNodes();
        }

        // add to _phases_to_add the tail of all the phases after the one inserted (nodes need to be recomputed)
        _phases_to_add.insert(_phases_to_add.end(), _phases.begin() + pos + 1, _phases.end());

        // remove active nodes from phases to add, needs to be recomputed (all the phases that were active may not be active anymore after being pushed back)
        for (auto phase_to_add : _phases_to_add)
        {
            phase_to_add->_get_active_nodes().clear();
        }
    }

    return true;
}

int SinglePhaseManager::_check_absolute_position(int pos)
{
    // search for the position 'pos', which is to be intended as the absolute position in horizon
    if (pos > _last_node)
    {
        // 'pos' is after the last phase
        _last_node = pos;
//        _trailing_empty_nodes = _n_nodes - _last_node;
        std::cout << "adding phase to pos: " << _last_node << std::endl;
        pos = -1;
        return pos;
    }

    std::cout << "inserting phase at absolute position: " << pos << std::endl;

    int phase_num = 0;
    // search it in the vector '_phases'
    for (int phase_pos=0; phase_pos < _phases.size(); phase_pos++)
    {
        if (pos >= _phases[phase_pos]->getPosition() && pos < _phases[phase_pos+1]->getPosition())
        {
            std::cout << "absolute position requested is between phase at positions: " << phase_pos << " and " << phase_pos+1 << std::endl;
            phase_num = phase_pos;

            if (pos < _phases[phase_pos]->getPosition() + _phases[phase_pos]->getNNodes())
            {
                throw std::runtime_error("absolute position requested is occupied by phase at position: " + std::to_string(phase_pos));
            }
        }
    }

    int total_duration = 0;
    for (auto phase : _phases_to_add)
    {
        total_duration += phase->getNNodes();
    }

    if ((pos + total_duration) < _phases[phase_num + 1]->getPosition())
    {
        std::cout << "You can insert it." << std::endl;
        _last_node = pos;
//        _trailing_empty_nodes = _n_nodes - _last_node;
        return pos;

    }
    else
    {
        throw std::runtime_error("There is no space left to insert phase.");
    }
        //(pos +  && pos < it->getPosition() + it->getNNodes())
    phase_num++;


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

bool SinglePhaseManager::shift()
{
    // reset all nodes
    _reset();

//    auto start_time = std::chrono::high_resolution_clock::now();
//    std::cout << " ============ shifting phases ============ : " << std::endl;

    //  if phases vector is empty, skip everything
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
        int n_nodes_last_active_phase = last_active_phase->getNNodes() - 1;
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

        // update every phase
        // there must be a more elegant way than this mess
        int i = 0;
        if (!_phases.empty())
        {
            // update the first phase given its position 0 (next phase will be at a node n depending on the active node of the first phase)
            _phases[0]->_update(i);
            i += _phases[0]->getActiveNodes().size();

            // update all the other phases
            for (int it = 1; it < _phases.size(); it++)
            {
                _phases[it]->_update(i);
                i += _phases[it]->getNNodes();
            }
        }

    }

    int num_nodes = 0;
    for (int i=0; i<_active_phases.size(); i++)
    {
        num_nodes += _active_phases[i]->_get_active_nodes().size();
    }

//    _trailing_empty_nodes = _n_nodes - num_nodes;

    _last_node -= 1;

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

bool SinglePhaseManager::clear()
{
    // this shows how the code sucks, there should be a function centralizing the update of all this info, as they are connected together.
    _reset();
    _phases.clear();
    _active_phases.clear();
    _last_node = 0;
//    _trailing_empty_nodes = _n_nodes;
    return true;
}

bool SinglePhaseManager::_reset()
{
    // todo: could be made faster?
    // todo: reset(pos) --> reset from a specific position?

    bool erasing = true;
    std::vector<int> empty_nodes;


    for (auto item : _items)
    {
//        std::cout << item->getName() << " (" << item << "): ";
        if (item->isChanged())
        {
//            std::cout << "resetting item..." << std::endl;
            item->setNodes(empty_nodes, erasing);
        }
    }

    for (auto item_ref : _items_ref)
    {
        if (item_ref->isChanged())
        {
//            std::cout << "resetting items ref..." << std::endl;
            item_ref->setNodes(empty_nodes, erasing);
            item_ref->clearValues();
        }
    }

    for (auto constraint : _constraints)
    {
        if (constraint->isChanged())
        {
//            std::cout << "resetting constraint..." << std::endl;
            constraint->setNodes(empty_nodes, erasing);
//        constraint->clearBounds(); // this cleared bounds here
        }
    }

    for (auto cost : _costs)
    {
        if (cost->isChanged())
        {
//            std::cout << "resetting cost..." << std::endl;
            cost->setNodes(empty_nodes, erasing);
        }
    }

    for (auto variable : _variables)
    {

        if (variable->isChanged())
        {
//            std::cout << "resetting variable..." << std::endl;
//          variable->clearNodes();
            variable->clearBounds();
        }
    }

    for (auto parameter : _parameters)
    {
        if (parameter->isChanged())
        {
//            std::cout << "resetting parameter..." << std::endl;
            parameter->clearValues();
        }
    }
    return true;
}

bool SinglePhaseManager::addPhase(std::vector<Phase::Ptr> phases, int pos, bool absolute_position)
{

    // add phase in temporary container (_phase_to_add), do computation and clean it
    for (auto phase : phases)
    {
        _phases_to_add.push_back(_generate_phase_token(phase));
    }

    _add_phases(pos, absolute_position);

    _phases_to_add.clear();

    return true;

}

bool SinglePhaseManager::addPhase(Phase::Ptr phase, int pos, bool absolute_position)
{
    if (!phase)
    {
        return false;
    }

    _phases_to_add.push_back(_generate_phase_token(phase));


    _add_phases(pos, absolute_position);
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
    return _n_nodes - _last_node;
//    return _trailing_empty_nodes;
}

std::vector<PhaseToken::Ptr> SinglePhaseManager::getActivePhases()
{
    // return the active phases on the horizon
    return _active_phases;
}

std::vector<PhaseToken::Ptr> SinglePhaseManager::getPhases()
{
    // return all the phases added (regardless of the phase being active on the horizon)
    return _phases;
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

// TODO: remove registerphase, is very tricky. Find a way to embed it in addPhase
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

bool PhaseManager::shift()
{
    for (auto timeline : _timelines)
    {
        timeline.second->shift();
    }

    return true;
}

bool PhaseManager::clear()
{
    for (auto timeline : _timelines)
    {
        timeline.second->clear();
    }

    return true;
}
