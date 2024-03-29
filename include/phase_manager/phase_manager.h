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
    bool addPhase(std::vector<Phase::Ptr> phases, int pos=-1, bool absolute_position_flag=false);
    bool addPhase(Phase::Ptr phase, int pos=-1, bool absolute_position_flag=false);

    Phase::Ptr getRegisteredPhase(std::string name);
    std::vector<Phase::Ptr> getRegisteredPhases();
    int getEmptyNodes();
    std::vector<PhaseToken::Ptr> getActivePhases();
    std::vector<PhaseToken::Ptr> getPhases();
    bool shift();
    bool clear();
    ~SinglePhaseManager();

private:

    bool _reset();
    bool _add_phases(int pos=-1, bool absolute_position_flag=false);
    int _insert_phases(int pos);
    int _update_phases_to_add(int pos);
    bool _update_active_phases();
    std::pair<int, int> _check_absolute_position(int pos);
    PhaseToken::Ptr _generate_phase_token(Phase::Ptr phase);

    std::string _name;
    std::vector<Phase::Ptr> _registered_phases; // container of all the registered phases
    int _n_nodes;

    // todo: find a way of exposing this in python (should give back "images" of the Phase class, which cannot be used)
    std::vector<PhaseToken::Ptr> _phases_to_add; // temporary vector of phases that are added to timeline
    std::vector<PhaseToken::Ptr> _phases; // list of all the phases
    std::vector<PhaseToken::Ptr> _active_phases; // list of all active phases

    // keep all the items from horizon
    std::vector<ItemBase::Ptr> _items;
    std::vector<ItemWithValuesBase::Ptr> _items_ref;

    std::vector<ItemWithBoundsBase::Ptr> _constraints;
    std::vector<ItemBase::Ptr> _costs;
    std::vector<ItemWithBoundsBase::Ptr> _variables;
    std::vector<ItemWithValuesBase::Ptr> _parameters;


//  self.default_action = Phase('default', 1)
//    float cioa = 1;
};


class PhaseManager
{

public:

    PhaseManager(int n_nodes);

    SinglePhaseManager::Ptr addTimeline(std::string name);

    SinglePhaseManager::Ptr getTimelines(std::string name);
    std::unordered_map<std::string, SinglePhaseManager::Ptr> getTimelines();

    bool registerPhase(std::string name, Phase::Ptr phase);
    bool addPhase(std::string name, Phase::Ptr phase);
    int getNodes();
    bool shift();
    bool clear();

private:

    std::unordered_map<std::string, SinglePhaseManager::Ptr> _timelines;
    int _n_nodes;

};

#endif
