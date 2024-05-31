#include <phase_manager/timeline.h>
#include <phase_manager/phase_manager.h>
#include <phase_manager/phase.h>

Timeline::Timeline(PhaseManager& phase_manager, int n_nodes, std::string name, bool debug):
    _name(name),
    _n_nodes(n_nodes),
    _phase_manager(phase_manager),
    _debug(debug)
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

    if (_phases.empty())
    {
        std::cout << "pos to absolute: " << 0 << std::endl;
        return 0;
    }

    if (pos == -1 || pos >= _phases.size())
    {
        std::cout << "pos to absolute: " << _phases.back()->getPosition() + _phases.back()->getNNodes() << std::endl;
        return _phases.back()->getPosition() + _phases.back()->getNNodes();
    }

    std::cout << "pos to absolute: " << _phases[pos]->getPosition() << std::endl;
    return _phases[pos]->getPosition();
}

bool Timeline::_update_active_phases(std::vector<std::shared_ptr<PhaseToken>> phases)
{
    // compute active nodes inside the added phases
    for (auto phase_token_i : phases)
    {
        int initial_node = phase_token_i->getPosition();
        int phase_nodes = phase_token_i->getNNodes();

        // std::cout << "initial_node: " << initial_node << std::endl;
        // set active node for each added phase
        if (initial_node < _n_nodes)
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

    return true;
}
bool Timeline::_add_phase(std::shared_ptr<PhaseToken> phase_to_add, int pos, bool absolute_position_flag)
{
    int absolute_position;
    int phase_position;
    bool success = false;

    if (!absolute_position_flag)
    {
        absolute_position = _pos_to_absolute(pos);
    }
    else
    {
        absolute_position = pos;
    }

    // compute absolute position of phase
    success = _check_absolute_pos(absolute_position, phase_position);

    if (!success)
    {
        if (_debug)
        {
            std::cout << "Failed to add phase '" << phase_to_add->getName() << "' at absolute position: " << pos << std::endl;
        }
        return false;
    }

    phase_to_add->_set_position(absolute_position);


    success = _insert_phase(phase_to_add, phase_position);

    if (!success)
    {
        return false;
    }


//    std::cout << "current phases in active phases vector: << ";
//    for (auto phase : _active_phases)
//    {
//       std::cout << phase->getName() << " ";
//    }
//    std::cout << ">> (" << _active_phases.size() << ")" << std::endl;


//    int counter_i = 1;
//    std::cout << "current phases: " << std::endl;
//    for (auto phase : _phases)
//    {

//        std::cout << counter_i << "-> " << phase->getName() << " (initial position = " << phase->getPosition() << ")" << std::endl;

//        std::cout << "  active nodes: ";


//        for (auto j : phase->getActiveNodes())
//        {
//            std::cout << j << " ";
//        }
//        std::cout << std::endl;
//        counter_i++;
//    }


//    std::cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = =" << std::endl;


    return true;


}

bool Timeline::_insert_phase(std::shared_ptr<PhaseToken> phase_to_add, int phase_pos)
{

    // if 'phase_pos' is beyond the horizon (outside of the active_phases), skip useless computation
    if (phase_pos < _active_phases.size())
    {

        // remove all the active_phases after the position 'pos'
//        std::cout << "position is " << phase_pos << ". Removing all the phases after pos " << phase_pos << " (number of removed phases: " << _active_phases.size() - phase_pos << ")" << std::endl;
        _active_phases.resize(phase_pos);

//        std::cout << "Remaining phases:" << std::endl;
//        for (auto phase: _active_phases)
//        {
//            std::cout << phase << " ";
//        }
//        std::cout << std::endl;

        // reset the items (holding all the active nodes)
        // TODO: should I do it only for the items before pos?
        _reset(); // there is way, resetting phase by phase not the whole thing
        // otherwise I have to update like this
        for (auto phase_i : _active_phases)
        {
            phase_i->update();
        }
    }

    int phase_to_add_duration = phase_to_add->getNNodes();
    std::vector<PhaseToken::Ptr> phases_to_add;
    phases_to_add.push_back(phase_to_add);

    if (phase_pos == -1 || phase_pos > _phases.size())
    {
        _phases.push_back(phase_to_add);
    }
    else
    {

        // shift all the phases in the tail (after the phase inserted)
        for (auto i = phase_pos; i < _phases.size(); i++)
        {
            int phase_token_i_pos = _phases[i]->getPosition();
            _phases[i]->_set_position(phase_token_i_pos + phase_to_add_duration);
            phases_to_add.push_back(_phases[i]);

        }
        _phases.resize(phase_pos);
        _phases.insert(_phases.begin() + phase_pos, phases_to_add.begin(), phases_to_add.end());
    }

    // remove active nodes from phases to add, needs to be recomputed (all the phases that were active may not be active anymore after being pushed back)
    for (auto phase_token_i : phases_to_add)
    {
        phase_token_i->_get_active_nodes().clear();
    }

    _update_active_phases(phases_to_add);

//    std::cout << "------------------" << std::endl;
    return true;
}

bool Timeline::_check_absolute_pos(int absolute_position, int& phase_position)
{
    // if the absolute position is free, add the phase at that position, pushing the other phases forward
    // if not free, do not add the phase

    // if phases is empty, can add everywhere
    if (_phases.empty())
    {
        phase_position = 0;
        return true;
    }

    // if position is after the last occupied node, add to specified position
    if (absolute_position >= _phases.back()->getPosition() + _phases.back()->getNNodes())
    {
        phase_position = _phases.size() + 1;
        return true;
    }

    // otherwise, search it in the vector '_phases'
    int phase_num = 0;
    for (phase_num; phase_num <= _phases.size() - 1; phase_num++)
    {
        // if the absolute phase is in a position already occupied, throw error
        if (absolute_position >= _phases[phase_num]->getPosition())
        {
            if (absolute_position == _phases[phase_num]->getPosition())
            {
                phase_position = _phases[phase_num]->getPosition();
                return true;
            }

            if (absolute_position < _phases[phase_num]->getPosition() + _phases[phase_num]->getNNodes())
            {
                return false;
            }
        }

        if (absolute_position <= _phases[phase_num + 1]->getPosition())
        {
            phase_position = phase_num + 1;
            return true;
        }
    }

//    std::cout << _phases[phase_num]->getPosition() << std::endl;
    return false;

}

//std::pair<int, int> Timeline::_check_absolute_position(int pos)
//{
//    // search for the position 'pos', which is to be intended as the absolute position in horizon
//    // if the absolute position is free, add the phase at that position, pushing the other phases forward
//    // if not free, do not add the phase
//    int absolute_position = pos;

//    // if there are no phases, add to specified position
//    if (_phases.size() == 0)
//    {
//        int phase_position = -1;
//        return std::make_pair(absolute_position, phase_position);
//    }

//    // if position is after the last occupied node, add to specified position
//    int last_current_node = _phases.back()->getPosition() + _phases.back()->getNNodes();
//    if (pos >= last_current_node)
//    {
//        int phase_position = -1;
//        return std::make_pair(absolute_position, phase_position);
//    }

//    // otherwise, search it in the vector '_phases'
//    int phase_num = 0;
//    for (phase_num; phase_num < _phases.size(); phase_num++)
//    {
//        // if the absolute phase is in a position already occupied, throw error
//        if ((pos > _phases[phase_num]->getPosition()) && (pos < _phases[phase_num]->getPosition() + _phases[phase_num]->getNNodes()))
//        {
//            if (_debug)
//            {
//                std::string text = "absolute position requested (" + std::to_string(pos) + ") is occupied by phase at position: " + std::to_string(phase_num) + ".";
//                std::cout << "WARNING: " << text << " Phase NOT added." << std::endl;
//            }
//            return std::make_pair(-1, -1);
////            throw std::runtime_error(text);
//        }

//        if (pos <= _phases[phase_num]->getPosition())
//        {
//            break;
//        }
//    }

//    /// todo
//    // once inserted, checking if the phase fits (is not overlapping with the next)
//    /// changing behaviour: now if the it can be added, it is added. All the others gets pushed further in the timeline

////    if (phase_num < _phases.size() && pos + total_duration > _phases[phase_num]->getPosition())
////    {
////        throw std::runtime_error("There is no space left to insert phase. Another phase is starting at node: " + std::to_string(_phases[phase_num]->getPosition()));
////    }

//    int phase_position = phase_num;
////    std::cout << "absolute_position: " << absolute_position << std::endl;
////    std::cout << "phase_position: " << phase_position << std::endl;

//    return std::make_pair(absolute_position, phase_position);


//}


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
//    std::cout << " ============ shifting phases =========== " << std::endl;

    //  if phases vector is empty, skip everything

    if (_phases.size() > 0)
    {
        // update active_phases with all the phases that have active nodes

        bool erase_node_flag = false;

        // roll down all phases by 1
        for (auto phase : _phases)
        {
            int phase_pos = phase->getPosition();
            phase->_set_position(phase_pos - 1);

            if (phase_pos <= 0)
            {
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
            // remove depleted phase from _active_phases
            if (_active_phases[0]->_get_active_nodes().empty())
            {
                _active_phases.erase(_active_phases.begin());
            }

            // remove depleted phase from _phases
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

bool Timeline::addPhase(Phase::Ptr phase, int pos, bool absolute_position_flag)
{
    if (!phase)
    {
        return false;
    }

    if (absolute_position_flag && pos == -1)
    {
        throw std::runtime_error("Can't add absolute position at position -1.");
    }

    bool res = _add_phase(_generate_phase_token(phase), pos, absolute_position_flag);


    return res;


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
