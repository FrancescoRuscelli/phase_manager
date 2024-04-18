#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#include <phase_manager/horizon_interface.h>

#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_set>

class Timeline;
class Phase;
//typedef std::shared_ptr<Timeline> Timeline::Ptr;
//typedef std::shared_ptr<Phase> Phase::Ptr;

class PhaseManager
{

public:

    typedef std::shared_ptr<PhaseManager> Ptr;

    PhaseManager(int n_nodes);

    std::shared_ptr<Timeline> createTimeline(std::string name);

    std::shared_ptr<Timeline> getTimelines(std::string name);

    std::unordered_map<std::string, std::shared_ptr<Timeline>> getTimelines();

    std::shared_ptr<Phase> createPhase(int n_nodes, std::string name);

//    bool registerPhase(std::string name, std::shared_ptr<Phase> phase);
//    bool addPhase(std::string name, std::shared_ptr<Phase> phase);
    int getNodes();
    bool shift();
    bool clear();

protected:

//    bool addItem(ItemBase::Ptr item);
//    std::unordered_map<std::string, ItemBase::Ptr> getItems();

private:

    std::unordered_map<std::string, std::shared_ptr<Timeline>> _timelines;

    int _n_nodes;

//    std::unordered_map<std::string, ItemBase::Ptr> _items;

};

#endif
