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

    struct BoundsContainer
    {
        BoundsContainer() {}

        // must be of same dimension
        std::vector<int> nodes;
        Eigen::MatrixXd lower_bounds;
        Eigen::MatrixXd upper_bounds;
    };


    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();

//    template <typename T>
//    bool addConstraint(std::shared_ptr<T> constraint, std::vector<int> nodes = {})
    bool addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint, std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        _constraints[constraint] = active_nodes;

        return 1;
    }
//    template <typename T>
//    bool addVariableBounds(std::shared_ptr<T> variable,
    bool addVariableBounds(ItemWithBoundsBase::ItemWithBoundsBasePtr variable,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        BoundsContainer val_container;

        val_container.nodes = active_nodes;
        val_container.lower_bounds = lower_bounds;
        val_container.upper_bounds = upper_bounds;

        _variables[variable] = val_container;

        return 1;
    }

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>> getConstraints();
    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, BoundsContainer> getVariables();

private:



    std::string _name;
    int _n_nodes;

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, std::vector<int>>_constraints;
//    std::map<ItemBase::ItemBasePtr, std::vector<int>> _costs;

    std::unordered_map<ItemWithBoundsBase::ItemWithBoundsBasePtr, BoundsContainer> _variables;

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

    int _update_constraints(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    int _update_variables(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
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

    std::vector<PhaseToken::PhaseTokenPtr> _phases; // list of all the phases
    std::vector<PhaseToken::PhaseTokenPtr> _active_phases; // list of all active phases
    int _trailing_empty_nodes; // empty nodes in horizon --> at the very beginning, all


//  self.default_action = Phase('default', 1)
//    float cioa = 1;
};

#endif
