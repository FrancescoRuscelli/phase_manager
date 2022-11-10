#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

//#include "Eigen/Dense"
#include <phase_manager/horizon_interface.h>

#include <vector>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <numeric>

class HorizonManager;

class Phase
{
public:

    typedef std::shared_ptr<Phase> PhasePtr;

    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();
    template <typename T>
    bool addConstraint(std::shared_ptr<T> constraint, std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        ItemWithBoundsBase::ItemWithBoundsBasePtr converted_constraint = std::make_shared<WrapperWithBounds<T>>(constraint);

        _constraints[converted_constraint] = active_nodes;

        return 1;
    }

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>> getConstraints();

    bool update();


//    std::map<ItemWithBoundsConcept::ItemWithBoundsConceptPtr, std::vector<int>> getConstraints();


private:

    std::string _name;
    int _n_nodes;

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>>_constraints;
//    std::map<ItemBase::ItemBasePtr, std::vector<int>> _costs;

//    std::map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>> _vars;

//    std::map<std::string, std::string> _vars_node;
//    std::map<std::string, std::string> _var_bounds;

//    std::map<std::string, std::string> _pars;
//    std::map<std::string, std::string> _pars_node;
//    std::map<std::string, std::string> _par_bounds;


//    def addCost(self, cost, nodes=None);

//    def addVariableBounds(self, var, lower_bounds, upper_bounds, nodes=None):

//    def addParameterValues(self, par, values, nodes=None):
};

class SinglePhaseManager;
class PhaseToken
{
    friend class SinglePhaseManager;
    friend class HorizonManager;
//    friend class std::forward<SinglePhaseManager>;
//    friend class std::shared_ptr<PhaseToken>;

public:

    typedef std::shared_ptr<PhaseToken> PhaseTokenPtr;

protected:

    PhaseToken(Phase::PhasePtr phase);

private:

    Phase::PhasePtr _abstract_phase;
    std::vector<int> _active_nodes;

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>> _constraints_in_horizon;
//    std::vector<std::string, std::set<int>> _costs_in_horizon;
//    std::vector<std::string, std::set<int>> _vars_in_horizon;
//    std::vector<std::string, std::set<int>> _pars_in_horizon;

    int _function_update(int initial_node, std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container);
    int _update(int initial_node);
    int _reset();

    int _get_n_nodes();
    std::vector<int>& _get_active_nodes();
    Phase::PhasePtr get_phase();

};

class SinglePhaseManager
{

public:

    SinglePhaseManager(int n_nodes, std::string name="how_to_use_default_arguments");

    bool registerPhase(Phase::PhasePtr phase);
    bool addPhase(std::vector<Phase::PhasePtr> phases);
    bool addPhase(Phase::PhasePtr phase);
    Phase::PhasePtr getRegisteredPhase(std::string name);
    Phase getActivePhase();
    int _shift_phases();

    ~SinglePhaseManager();
//    Eigen::VectorXd diocane;

private:
    bool _add_phase(Phase::PhasePtr phase, int pos=-1); // TODO substitute with pointer


    std::unique_ptr<HorizonManager> _horizon_manager;

    std::string _name;
    std::vector<Phase::PhasePtr> _registered_phases; // container of all the registered phases
    int _n_nodes;

//    PhaseContainer phase_container; //PhaseContainer

    std::vector<PhaseToken::PhaseTokenPtr> _phases; // list of all the phases
    int _n_phases;
    std::vector<PhaseToken::PhaseTokenPtr> _active_phases; // list of all active phases
    int _n_active_phases;
    std::vector<int*> _activated_nodes; //
    int _horizon_nodes[100]; // np.nan * np.ones(self.n_tot)
    int _trailing_empty_nodes; // empty nodes in horizon --> at the very beginning, all


//  self.default_action = Phase('default', 1)
//    float cioa = 1;
};

#endif
