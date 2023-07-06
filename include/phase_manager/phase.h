#ifndef PHASE_H
#define PHASE_H

#include <phase_manager/horizon_interface.h>

#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include <iostream>
#include <memory>
#include <stdexcept>


class Phase
{
    /*
     * container of any object compatible (i.e. has a setNodes() at least).
     * It represent an abstract phase that can be used to define constraints, costs, bounds active on certain segments of nodes.
     */

public:

    typedef std::shared_ptr<Phase> Ptr;

    struct BoundsContainer
    {
        BoundsContainer() {}

        // must be of same dimension
        // TODO: what happen if not same dimension?
        std::vector<int> nodes;
        Eigen::MatrixXd lower_bounds;
        Eigen::MatrixXd upper_bounds;
    };

    struct ValuesContainer
    {
        ValuesContainer() {}

        // must be of same dimension
        // TODO: what happen if not same dimension?
        std::vector<int> nodes;
        Eigen::MatrixXd values;
    };

    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();



    std::vector<int> _check_active_nodes(std::vector<int> nodes)
    {
        // if nodes inserted is empty, get all the nodes in horizon
        std::vector<int> range_nodes(_n_nodes);
        std::iota(std::begin(range_nodes), std::end(range_nodes), 0); //0 is the starting number

        std::vector<int> active_nodes = (nodes.empty()) ? range_nodes : nodes;

        return active_nodes;
    }

    bool addItem(ItemBase::Ptr item, std::vector<int> nodes = {})
    {
        auto active_nodes = _check_active_nodes(nodes);
//        std::cout << "adding item:" << item->getName() << " to phase. " << std::endl;
        _items_base[item] = active_nodes;

        return true;

    }

    bool addItemReference(ItemWithValuesBase::Ptr item_with_ref,
                          Eigen::MatrixXd values,
                          std::vector<int> nodes = {})
    {
        auto active_nodes = _check_active_nodes(nodes);

        ValuesContainer val_container;

        if (values.cols() != active_nodes.size())
        {
            throw std::invalid_argument("Row dimension of values inserted ("
                                  + std::to_string(values.cols())
                                  + ") does not match number of nodes specified ("
                                  + std::to_string(active_nodes.size()) + ")");
        }

        val_container.nodes = active_nodes;
        val_container.values = values;

        _items_ref[item_with_ref] = val_container;

        return 1;

    }

    bool addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes = {})
    {

        auto active_nodes = _check_active_nodes(nodes);
        _constraints[constraint] = active_nodes;

        return 1;
    }

    bool addCost(ItemBase::Ptr cost, std::vector<int> nodes = {})
    {
        // check if added item is actually a cost?
        auto active_nodes = _check_active_nodes(nodes);
        _costs[cost] = active_nodes;

        return 1;
    }


    bool addVariableBounds(ItemWithBoundsBase::Ptr variable,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes = {})
    {

        auto active_nodes = _check_active_nodes(nodes);

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

        /*
         * add to horizon parameter a desired value
         */
        auto active_nodes = _check_active_nodes(nodes);

        ValuesContainer val_container;

        val_container.nodes = active_nodes;
        val_container.values = values;

        _parameters[parameter] = val_container;

        return 1;
    }


    std::unordered_map<ItemBase::Ptr, std::vector<int>> getItems();
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> getItemsReference();

    std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> getConstraints();
    std::unordered_map<ItemBase::Ptr, std::vector<int>> getCosts();
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer> getVariables();
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> getParameters();


private:

    std::string _name;
    int _n_nodes;

    // generic item that must have a method setNodes()
    std::unordered_map<ItemBase::Ptr, std::vector<int>> _items_base;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> _items_ref;

    std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> _constraints;
    std::unordered_map<ItemBase::Ptr, std::vector<int>> _costs;
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer> _variables;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> _parameters;


};


class PhaseToken
{
    /*
     * generated given the abstract class Phase.
     * Keeps track of the active nodes set nodes to the objects inside it (constraints, costs, tasks ...)
     */

    friend class SinglePhaseManager;
    friend class HorizonManager;

public:

    typedef std::shared_ptr<PhaseToken> Ptr;
    std::string getName();
    std::vector<int> getActiveNodes();
    std::vector<int> getNodes(); // todo: get the absolute position of nodes (even if they are not active)
    // get name, get nodes...

protected:

    PhaseToken(Phase::Ptr phase);

private:

    Phase::Ptr _abstract_phase;
    std::vector<int> _active_nodes;
    std::vector<int> _nodes;

    // all these updates gets called by the SinglePhaseManager
    bool _update_items(int initial_node);
    bool _update_item_reference(int initial_node);
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
