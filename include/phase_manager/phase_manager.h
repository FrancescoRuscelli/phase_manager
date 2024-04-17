#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#include <phase_manager/timeline.h>

#include <map>
#include <set>


class PhaseManager
{

public:

    typedef std::shared_ptr<PhaseManager> Ptr;

    PhaseManager(int n_nodes);

    Timeline::Ptr addTimeline(std::string name);

    Timeline::Ptr getTimelines(std::string name);

    std::unordered_map<std::string, Timeline::Ptr> getTimelines();

    bool registerPhase(std::string name, Phase::Ptr phase);
    bool addPhase(std::string name, Phase::Ptr phase);
    int getNodes();
    bool shift();
    bool clear();

private:

    std::unordered_map<std::string, Timeline::Ptr> _timelines;
    int _n_nodes;

};

#endif
