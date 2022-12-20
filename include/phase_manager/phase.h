#ifndef PHASE_H
#define PHASE_H

#include <phase_manager/horizon_interface.h>

#include <eigen3/Eigen/Dense>
#include <numeric>
#include <vector>
#include <iostream>
#include <memory>


class Phase
{
public:

    typedef std::shared_ptr<Phase> Ptr;

    struct BoundsContainer
    {
        BoundsContainer() {}

        // must be of same dimension
        std::vector<int> nodes;
        Eigen::MatrixXd lower_bounds;
        Eigen::MatrixXd upper_bounds;
    };

    struct ValuesContainer
    {
        ValuesContainer() {}

        // must be of same dimension
        std::vector<int> nodes;
        Eigen::MatrixXd values;
    };

    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();


    bool addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        _constraints[constraint] = active_nodes;

        return 1;
    }

    bool addCost(ItemBase::Ptr cost, std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        _costs[cost] = active_nodes;

        return 1;
    }


    bool addVariableBounds(ItemWithBoundsBase::Ptr variable,
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

    bool addParameterValues(ItemWithValuesBase::Ptr parameter,
                           Eigen::MatrixXd values,
                           std::vector<int> nodes = {})
    {

        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        ValuesContainer val_container;

        val_container.nodes = active_nodes;
        val_container.values = values;

        _parameters[parameter] = val_container;

        return 1;
    }

    std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> getConstraints();
    std::unordered_map<ItemBase::Ptr, std::vector<int>> getCosts();
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer> getVariables();
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> getParameters();

private:

    std::string _name;
    int _n_nodes;

    std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> _constraints;
    std::unordered_map<ItemBase::Ptr, std::vector<int>> _costs;
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer> _variables;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> _parameters;


};


class PhaseToken
{
    friend class SinglePhaseManager;
    friend class HorizonManager;

public:

    typedef std::shared_ptr<PhaseToken> Ptr;

protected:

    PhaseToken(Phase::Ptr phase);

private:

    Phase::Ptr _abstract_phase;
    std::vector<int> _active_nodes;

    bool _update_constraints(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_variables(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_costs(int initial_node);
    bool _update_parameters(int initial_node);

    std::pair<std::vector<int>, std::vector<int>> _compute_horizon_nodes(std::vector<int> nodes, int initial_node);

    bool _update(int initial_node);

    int _get_n_nodes();
    std::vector<int>& _get_active_nodes();
    Phase::Ptr get_phase();

};

#endif
