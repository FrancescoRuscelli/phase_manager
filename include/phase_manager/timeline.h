#ifndef TIMELINE_H
#define TIMELINE_H

#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <set>

class PhaseManager;
class Phase;
class PhaseToken;

class ItemBase;
class ItemWithValuesBase;
class ItemWithBoundsBase;


class Timeline
{
    friend class Phase;

public:

    typedef std::shared_ptr<Timeline> Ptr;

    Timeline(PhaseManager& phase_manager, int n_nodes, std::string name="", bool debug=false);
    // , PhaseManager::Ptr phase_manager

    std::string getName();

//    bool registerPhase(std::shared_ptr<Phase> phase);
    std::shared_ptr<Phase> createPhase(int n_nodes, std::string name);

//    bool addPhase(std::vector<std::shared_ptr<Phase>> phases, int pos=-1, bool absolute_position_flag=false);
    bool addPhase(std::shared_ptr<Phase> phase, int pos=-1, bool absolute_position_flag=false);


    std::shared_ptr<Phase> getRegisteredPhase(std::string name);
//    std::vector<std::shared_ptr<Phase>> getRegisteredPhases();
    int getEmptyNodes();
    std::vector<std::shared_ptr<PhaseToken>> getActivePhases();
    std::vector<std::shared_ptr<PhaseToken>> getPhases();
    bool shift();
    bool clear();
    ~Timeline();

protected:

    bool addElement(std::shared_ptr<ItemBase> element);
    std::shared_ptr<ItemBase> getElement(std::string name);

private:

    bool _reset();
    bool _add_phase(std::shared_ptr<PhaseToken> phase_to_add, int pos=-1, bool absolute_position_flag=false);
    bool _insert_phase(std::shared_ptr<PhaseToken> phase, int pos);
    bool _update_active_phases(std::vector<std::shared_ptr<PhaseToken>> phases);
    int _pos_to_absolute(int pos);
    bool _check_absolute_pos(int absolute_position, int& phase_position);
//    std::pair<int, int> _check_absolute_position(int pos);
    std::shared_ptr<PhaseToken> _generate_phase_token(std::shared_ptr<Phase> phase);

    std::string _name;
    std::unordered_map<std::string, std::shared_ptr<Phase>> _registered_phases; // container of all the registered phases
    int _n_nodes;

    // todo: find a way of exposing this in python (should give back "images" of the Phase class, which cannot be used)
    std::vector<std::shared_ptr<PhaseToken>> _phases; // list of all the phases
    std::vector<std::shared_ptr<PhaseToken>> _active_phases; // list of all active phases

    // keep all the items from horizon
    std::unordered_map<std::string, std::shared_ptr<ItemBase>> _elements;
//    std::vector<std::shared_ptr<ItemWithValuesBase>> _items_ref;
//    std::vector<std::shared_ptr<ItemWithBoundsBase>> _constraints;
//    std::vector<std::shared_ptr<ItemBase>> _costs;
//    std::vector<std::shared_ptr<ItemWithBoundsBase>> _variables;
//    std::vector<std::shared_ptr<ItemWithValuesBase>> _parameters;

    PhaseManager& _phase_manager;

    bool _debug;

};

#endif
