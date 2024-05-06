#include <phase_manager/phase_manager.h>
#include <phase_manager/timeline.h>
//#include <phase_manager/phase.h>

PhaseManager::PhaseManager(int n_nodes):
    _n_nodes(n_nodes)
{

}

Timeline::Ptr PhaseManager::createTimeline(std::string name)
{
    Timeline::Ptr timeline = std::make_shared<Timeline>(*this, _n_nodes, name);
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
