#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#include <phase_manager/horizon_interface.h>
#include <phase_manager/phase.h>

#include <vector>
#include <iostream>
#include <map>
#include <memory>
#include <set>

class HorizonManager;

class SinglePhaseManager
{

public:

    typedef std::shared_ptr<SinglePhaseManager> Ptr;

    SinglePhaseManager(int n_nodes, std::string name="");

    bool registerPhase(Phase::Ptr phase);
    bool addPhase(std::vector<Phase::Ptr> phases, int pos=-1);
    bool addPhase(Phase::Ptr phase, int pos=-1);
    Phase::Ptr getRegisteredPhase(std::string name);
//    std::vector<PhaseToken::Ptr> getActivePhase();
    bool _shift_phases();

    ~SinglePhaseManager();

private:

    bool _add_phases(int pos=-1); // TODO substitute with pointer
    PhaseToken::Ptr _generate_phase_token(Phase::Ptr phase);

    // horizon manager manages nodes for Horizon
    std::unique_ptr<HorizonManager> _horizon_manager;

    std::string _name;
    std::vector<Phase::Ptr> _registered_phases; // container of all the registered phases
    int _n_nodes;

    std::vector<PhaseToken::Ptr> _phases_to_add;
    std::vector<PhaseToken::Ptr> _phases; // list of all the phases
    std::vector<PhaseToken::Ptr> _active_phases; // list of all active phases
    int _trailing_empty_nodes; // empty nodes in horizon --> at the very beginning, all


//  self.default_action = Phase('default', 1)
//    float cioa = 1;
};


class PhaseManager
{

public:

    PhaseManager(int n_nodes);

    SinglePhaseManager::Ptr addTimeline(std::string name);
    SinglePhaseManager::Ptr getTimeline(std::string name);

    bool registerPhase(std::string name, Phase::Ptr phase);
    bool addPhase(std::string name, Phase::Ptr phase);
    bool _shift_phases();

private:

    std::unordered_map<std::string, SinglePhaseManager::Ptr> _timelines;
    int _n_nodes;

};

#endif
