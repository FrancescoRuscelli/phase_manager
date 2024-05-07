#include <phase_manager/timeline.h>
#include <phase_manager/phase_manager.h>
#include <phase_manager/phase.h>

Timeline::Timeline(PhaseManager& phase_manager, int n_nodes, std::string name):
    _name(name),
    _n_nodes(n_nodes),
    _phase_manager(phase_manager)
{

}

std::string Timeline::getName()
{
    return _name;
}

Phase::Ptr Timeline::createPhase(int n_nodes, std::string name)
{

    Phase::Ptr phase = std::make_shared<Phase>(*this, n_nodes, name);

    if (_registered_phases.find(name) != _registered_phases.end())
    {
        throw std::runtime_error(std::string("Failed to create phase: '") + name + std::string("'. Name already registered."));
    }
    _registered_phases[name]= phase;
    return phase;

}

bool Timeline::addElement(std::shared_ptr<ItemBase> element)
{
    // registering the items inside the phase_manager, if not already registered
//    std::cout << "registering element: " << element->getName() << " (" << element << ")" << std::endl;
    if (_elements.find(element->getName()) != _elements.end())
    {
//        std::cout << " already present (" << _elements[element->getName()] << ")" << std::endl;
//        std::cout << " ========== " << std::endl;
        return false;
    }

//    std::cout << " ========== " << std::endl;

    _elements[element->getName()] = element;

    return true;
}

std::shared_ptr<ItemBase> Timeline::getElement(std::string name)
{
    auto it = _elements.find(name);

    if (it != _elements.end())
    {
        return _elements[name];
    }
    else
    {
        return nullptr;
    }
}

int Timeline::_pos_to_absolute(int pos)
{
    int absolute_position;

    if (pos < _phases.size())
    {
        absolute_position = _phases[pos]->getPosition() + _phases[pos]->getNNodes();
    }
    else
    {
        absolute_position = _phases.back()->getPosition() + _phases.back()->getNNodes();
    }

    return absolute_position;
}
bool Timeline::_add_phases(int pos, bool absolute_position_flag)
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
//    std::cout << " = = = = = = = = =" << std::endl;
    int last_node = 0;

    // compute absolute position of phase
    int absolute_position = 0;
    if (absolute_position_flag)
    {
        std::tie(absolute_position, pos) = _check_absolute_position(pos);

        if ((absolute_position == -1) && (pos == -1))
        {
            return 0;
        }
    }
    else
    {
        if (!_phases.empty())
        {
            absolute_position = _phases.back()->getPosition() + _phases.back()->getNNodes();
        }
    }

    // fill _phases_to_add, a vector of phases to add to the horizon

    last_node = _insert_phases(pos, absolute_position);


    // compute active nodes inside the added phases
    for (auto phase_token_i : _phases_to_add)
    {
        int initial_node = phase_token_i->getPosition();
        int phase_nodes = phase_token_i->getNNodes();

        // std::cout << "initial_node: " << initial_node << std::endl;
        // set active node for each added phase
        if (initial_node <= _n_nodes)
        {
            int active_nodes = phase_nodes;
            // phase is active (even if its tail falls outside the horizon)
//            std::cout << "   --> Adding phase token (" << phase_token_i << ") '" << phase_token_i->getName() << "' to active phases" << std::endl;;
            _active_phases.push_back(phase_token_i);

            if (initial_node + active_nodes >= _n_nodes)
            {
                active_nodes -= initial_node + phase_nodes - _n_nodes;
            }

            for (int i = 0; i<active_nodes; i++)
            {
                phase_token_i->_get_active_nodes().push_back(i);
            }

//            std::cout << "        starting position: " << phase_token_i->getPosition() << std::endl;
//            std::cout << "        active_nodes: " << active_nodes << "/" << phase_token_i->getNNodes() << std::endl;
        }

        // update the phase tokens
        phase_token_i->update();
//        std::cout << "============================" << std::endl;
    }

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

int Timeline::_insert_phases(int pos, int absolute_position)
{
//    std::cout << "inserting phases at position: " << pos << std::endl;
//    std::cout << "inserting phases at absolute position: " << absolute_position << std::endl;

    // if 'pos' is beyond the horizon (outside of the active_phases), skip useless computation
    if (pos <= _active_phases.size())
    {
        // remove all the active_phases after the position 'pos'
        _active_phases.resize(pos);

        // reset the items (holding all the active nodes)
        // TODO: should I do it only for the items before pos?
        _reset(); // there is way, resetting phase by phase not the whole thing
        // otherwise I have to update like this
        for (auto phase_i : _active_phases)
        {
            phase_i->update();
        }
    }

    // update position of phases before the position 'pos' (before i resetted)
    int last_current_node = 0;
    if (!_active_phases.empty())
    {
        last_current_node = _active_phases.back()->getPosition() + _active_phases.back()->getNNodes();

        // if the absolute node is greater than the last current node, add in absolute node (if there are no gaps between phases, last_current_node=absolute_position)
        if (last_current_node < absolute_position)
        {
            last_current_node = absolute_position;
        }
    }

    // set initial nodes of phases inserted
    int added_nodes = 0;
    for (auto phase_token_i : _phases_to_add)
    {
        phase_token_i->_set_position(last_current_node);
        added_nodes += phase_token_i->getNNodes();
//        std::cout << "setting position of " << phase_token_i->getName() << ": " << last_current_node << std::endl;
    }

//    std::cout << "number of nodes inserted: " << added_nodes << std::endl;

    // shift all the phases in the tail (after the phase inserted)


    if (pos == -1 || pos > _phases.size())
    {
        // add phase_tokens in temporary container (_phase_to_add) to stack
//        std::cout << "Adding '" << _phases_to_add.size() << "' phase/s at tail: " << "(pos: " << _phases.size() << ")";
        _phases.insert(_phases.end(), _phases_to_add.begin(), _phases_to_add.end());
    }
    else
    {
        //  std::cout << " ...done." << " (total n. of phases: " << _phases.size() << ")" << std::endl;
        // insert phase_tokens of temporary container (_phase_to_add) in stack at posistion 'pos'
        //    std::cout << "Inserting phase at pos: " << pos << " (total n. of phases: " << _phases.size() << ")";
            _phases.insert(_phases.begin() + pos, _phases_to_add.begin(), _phases_to_add.end());
        //    std::cout << " ...done." << std::endl;

        for (int pos_i = pos + 1; pos_i < _phases.size(); pos_i++)
        {
            _phases[pos_i]->_set_position(_phases[pos_i]->getPosition() + added_nodes);
//            std::cout << "setting position of " << _phases[pos_i]->getName() << ": " << _phases[pos_i]->getPosition() + added_nodes << std::endl;
        }


    // add to _phases_to_add the tail of all the phases after the one inserted (nodes need to be recomputed)
        _phases_to_add.insert(_phases_to_add.end(), _phases.begin() + pos + 1, _phases.end());
    }
    //    std::cout << "Inserting phases to be recomputed: " << std::endl;
    //    for (auto phase_token_i : _phases_to_add)
    //    {
    //        std::cout << phase_token_i->getName() << ", ";
    //    }
    //    std::cout << std::endl;

    // remove active nodes from phases to add, needs to be recomputed (all the phases that were active may not be active anymore after being pushed back)
    for (auto phase_token_i : _phases_to_add)
    {
        phase_token_i->_get_active_nodes().clear();
    }

//    std::cout << "------------------" << std::endl;
    return last_current_node;
}

std::pair<int, int> Timeline::_check_absolute_position(int pos)
{
    // search for the position 'pos', which is to be intended as the absolute position in horizon
    // if the absolute position is free, add the phase at that position, pushing the other phases forward
    // if not free, do not add the phase
    int absolute_position = pos;

    // if there are no phases, add to specified position
    if (_phases.size() == 0)
    {
        int phase_position = -1;
        return std::make_pair(absolute_position, phase_position);
    }

    // if position is after the last occupied node, add to specified position
    int last_current_node = _phases.back()->getPosition() + _phases.back()->getNNodes();
    if (pos >= last_current_node)
    {
        int phase_position = -1;
        return std::make_pair(absolute_position, phase_position);
    }

    // otherwise, search it in the vector '_phases'
    int phase_num = 0;
    for (phase_num; phase_num < _phases.size(); phase_num++)
    {
        // if the absolute phase is in a position already occupied, throw error
        if ((pos > _phases[phase_num]->getPosition()) && (pos < _phases[phase_num]->getPosition() + _phases[phase_num]->getNNodes()))
        {
            std::string text = "absolute position requested (" + std::to_string(pos) + ") is occupied by phase at position: " + std::to_string(phase_num) + ".";
            std::cout << "WARNING: " << text << " Phase NOT added." << std::endl;
            return std::make_pair(-1, -1);
//            throw std::runtime_error(text);
        }

        if (pos <= _phases[phase_num]->getPosition())
        {
            break;
        }
    }

    int total_duration = 0;
    for (auto phase : _phases_to_add)
    {
        total_duration += phase->getNNodes();
    }

    /// todo
    // once inserted, checking if the phase fits (is not overlapping with the next)
    /// changing behaviour: now if the it can be added, it is added. All the others gets pushed further in the timeline

//    if (phase_num < _phases.size() && pos + total_duration > _phases[phase_num]->getPosition())
//    {
//        throw std::runtime_error("There is no space left to insert phase. Another phase is starting at node: " + std::to_string(_phases[phase_num]->getPosition()));
//    }

    int phase_position = phase_num;
//    std::cout << "absolute_position: " << absolute_position << std::endl;
//    std::cout << "phase_position: " << phase_position << std::endl;

    return std::make_pair(absolute_position, phase_position);


}


PhaseToken::Ptr Timeline::_generate_phase_token(Phase::Ptr phase)
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

bool Timeline::shift()
{
    // reset all nodes
    _reset();

//    auto start_time = std::chrono::high_resolution_clock::now();
//    std::cout << " ============ shifting phases ============ : " << std::endl;

    //  if phases vector is empty, skip everything

    if (_phases.size() > 0)
    {
        // update active_phases with all the phases that have active nodes

        bool erase_node_flag = false;
//        _active_phases.clear();

//        for (int i=0; i<_phases.size(); i++)
//        {
//            if (_phases[i]->_get_active_nodes().size() > 0)
//            {
//                _active_phases.push_back(_phases[i]);
//            }
//        }
        // roll down all phases by 1
        for (auto phase : _phases)
        {
            int phase_pos = phase->getPosition();

            if (phase_pos > 0)
            {
                phase->_set_position(phase_pos - 1);
            }
            else
            {
                // if phase is in position 0, erase node or full phase
                erase_node_flag = true;
            }
        }

        auto n_active_phases = _active_phases.size();

        if (n_active_phases > 0)
        {
            auto last_active_phase = _active_phases.back();
            auto pos_last_node = last_active_phase->getPosition() + last_active_phase->getNNodes();

            // if there is a phase entering the horizon window, activate its nodes
            if (pos_last_node >= _n_nodes)
            {
                int elem = (last_active_phase->_get_active_nodes().size() > 0) ? last_active_phase->_get_active_nodes().back() + 1 : 0;
                last_active_phase->_get_active_nodes().push_back(elem);
            }
        }

        // as long as there are non-active phases in the stack, check if the next non-active phase can become active
        if (n_active_phases < _phases.size())
        {
            // add new phase in active phase if its position is inside horizon nodes (less than _n_nodes)
            if (_phases[n_active_phases]->getPosition() < _n_nodes)
            {
                int elem = 0;
                _phases[n_active_phases]->_get_active_nodes().push_back(elem);
                _active_phases.push_back(_phases[n_active_phases]);
            }
        }

        // remove first element in phases
        // todo: should not erase, but shift below 0. Nodes with negative position are not considered
        if (erase_node_flag)
        {
            _active_phases[0]->_get_active_nodes().erase(_active_phases[0]->_get_active_nodes().begin());

            if (_active_phases[0]->_get_active_nodes().empty())
            {
                _active_phases.erase(_active_phases.begin());
            }

            if (_phases[0]->_get_active_nodes().empty())
            {
                _phases.erase(_phases.begin());
            }
        }

        // update every phase
        for (auto phase : _phases)
        {
            phase->update();
        }
    }

    return true;
}

bool Timeline::clear()
{
    // this shows how the code sucks, there should be a function centralizing the update of all this info, as they are connected together.
    _reset();
    _phases.clear();
    _active_phases.clear();
    return true;
}

bool Timeline::_reset()
{

    // windows to items stored in phase_manager
//    std::cout << "--------- resetting phases: ----------" << std::endl;
    for (auto const & [name, item] : _elements)
    {
//        std::cout << name << " (" << item << ") changed? " << item->isChanged() << std::endl;
        if (item->isChanged())
        {
//            std::cout << "resetting item..." << std::endl;
            item->reset();
//             std::cout << "done" << std::endl;
        }
    }
    return true;
}

bool Timeline::addPhase(std::vector<Phase::Ptr> phases, int pos, bool absolute_position_flag)
{

    /*
     * Add phase to timeline.
     * pos: position where to insert the phase. If not specified, the phase is added as last. The position is w.r.t. the order of phases.
     * absolute_position_flag: if true, the position is the specified absolute node in the timeline. Default is false.
    */

    // add phase in temporary container (_phase_to_add), do computation and clean it
    for (auto phase : phases)
    {
        _phases_to_add.push_back(_generate_phase_token(phase));
    }

    _add_phases(pos, absolute_position_flag);

    _phases_to_add.clear();

    return true;

}

bool Timeline::addPhase(Phase::Ptr phase, int pos, bool absolute_position_flag)
{
    if (!phase)
    {
        return false;
    }

    _phases_to_add.push_back(_generate_phase_token(phase));


    _add_phases(pos, absolute_position_flag);
    _phases_to_add.clear();

    return true;


}

Phase::Ptr Timeline:: getRegisteredPhase(std::string name)
{
//    std::cout << "checking name: '" << phase_name << "'" <<std::endl;
    if (_registered_phases.find(name) == _registered_phases.end())
    {
      return nullptr;
    }

    return _registered_phases[name];
}

//std::vector<Phase::Ptr> Timeline::getRegisteredPhases()
//{
//    return _registered_phases;
//}

int Timeline::getEmptyNodes()
{
    int empty_nodes = _n_nodes;
    if (_phases.size() > 0)
    {
        empty_nodes -= _phases.back()->getPosition() + _phases.back()->getNNodes();
    }
    return empty_nodes;
}

std::vector<PhaseToken::Ptr> Timeline::getActivePhases()
{
    // return the active phases on the horizon
    return _active_phases;
}

std::vector<PhaseToken::Ptr> Timeline::getPhases()
{
    // return all the phases added (regardless of the phase being active on the horizon)
    return _phases;
}

Timeline::~Timeline()
{

}
