#ifndef PHASE_MANAGER_H
#define PHASE_MANAGER_H

#include "Eigen/Dense"
#include <iostream>
#include <map>
#include <memory>
#include <set>

class Phase
{
public:

    typedef std::shared_ptr<Phase> PhasePtr;

    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();
    template <typename HorizonFunction>
    bool addConstraint(HorizonFunction constraint, int nodes=-1);

private:

    std::string _name;
    int _n_nodes;

    std::map<std::string, int> _constraints;
    std::map<std::string, int> _costs;

//    std::map<std::string, std::string> _vars;
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
//    friend class std::forward<SinglePhaseManager>;
//    friend class std::shared_ptr<PhaseToken>;

public:

    typedef std::shared_ptr<PhaseToken> PhaseTokenPtr;

protected:

    PhaseToken(Phase::PhasePtr phase);

private:

    Phase::PhasePtr _abstract_phase;

    std::vector<int> _active_nodes;
//    std::vector<std::string, std::set<int>> _constraints_in_horizon;
//    std::vector<std::string, std::set<int>> _costs_in_horizon;
//    std::vector<std::string, std::set<int>> _vars_in_horizon;
//    std::vector<std::string, std::set<int>> _pars_in_horizon;

    int _function_update(int initial_node, std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container);
    int _update(int initial_node);
    int _reset();

    int _get_n_nodes();
    std::vector<int>& _get_active_nodes();

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
