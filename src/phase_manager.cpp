#include <phase_manager/phase_manager.h>

PhaseManager::PhaseManager(int n_nodes):
    _n_nodes(n_nodes)
{

}

Timeline::Ptr PhaseManager::addTimeline(std::string name)
{
    Timeline::Ptr timeline = std::make_shared<Timeline>(_n_nodes, name);
    _timelines[name]= timeline;
    return timeline;
}

std::unordered_map<std::string, Timeline::Ptr> PhaseManager::getTimelines()
{
    return _timelines;
}

Timeline::Ptr PhaseManager::getTimelines(std::string name)
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
