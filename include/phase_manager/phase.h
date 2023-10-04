#ifndef PHASE_H
#define PHASE_H

#include <phase_manager/horizon_interface.h>

#include <Eigen/Dense>
#include <numeric>
#include <vector>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_set>
//#include <any>
//#include <variant>
//#include <functional>


class Phase
{
    /*
     * container of any object compatible (i.e. has a setNodes() at least).
     * It represent an abstract phase that can be used to define constraints, costs, bounds active on certain segments of nodes.
     */

public:

    typedef std::shared_ptr<Phase> Ptr;

    class InfoContainer
    {

    public:

        typedef std::shared_ptr<InfoContainer> Ptr;
        InfoContainer() {}
        virtual ~InfoContainer() = default;
        virtual std::string getType() {return "base";}

        std::vector<int> nodes;
    };

    class BoundsContainer : virtual public InfoContainer
    {

    public:

        typedef std::shared_ptr<BoundsContainer> Ptr;
        BoundsContainer() {}

        // must be of same dimension
        std::string getType() {return "bounds";}
        // TODO: what happen if not same dimension?
        Eigen::MatrixXd lower_bounds;
        Eigen::MatrixXd upper_bounds;
    };

    class ValuesContainer : virtual public InfoContainer
    {

    public:

        typedef std::shared_ptr<ValuesContainer> Ptr;
        ValuesContainer() {}

        // must be of same dimension
        std::string getType() {return "values";}
        // TODO: what happen if not same dimension?
        Eigen::MatrixXd values;
    };

    Phase(int n_nodes, std::string name);
    std::string getName();
    int getNNodes();
    bool setDuration(int new_n_nodes);
    bool setElemNodes(std::string elem_name, std::vector<int> nodes,
                      const Eigen::MatrixXd& value_1 = Eigen::MatrixXd(),
                      const Eigen::MatrixXd& value_2 = Eigen::MatrixXd());

    bool addItem(ItemBase::Ptr item, std::vector<int> nodes = {})
    {
        auto active_nodes = _check_active_nodes(nodes);
//        std::cout << "adding item:" << item->getName() << " to phase. " << std::endl;

        InfoContainer::Ptr nodes_container = std::make_unique<InfoContainer>();
        nodes_container->nodes = active_nodes;

        // add to vector the element pointer
        // add to map name-element
        // add to map element-info
        _items_base.push_back(item);
        _info_items_base[item] = nodes_container;
        _elem_map[item->getName()] = item;

        return true;

    }

    bool addItemReference(ItemWithValuesBase::Ptr item_with_ref,
                          Eigen::MatrixXd values,
                          std::vector<int> nodes = {})
    {
        auto active_nodes = _check_active_nodes(nodes);

        ValuesContainer::Ptr val_container = std::make_unique<ValuesContainer>();

//        std::cout << "nodes: ";

//        for (auto elem : nodes)
//        {
//            std::cout << elem << " ";
//        }
//        std::cout << std::endl;

        if (values.cols() != active_nodes.size())
        {
            throw std::invalid_argument("Row dimension of values inserted ("
                                  + std::to_string(values.cols())
                                  + ") does not match number of nodes specified ("
                                  + std::to_string(active_nodes.size()) + ")");
        }

        val_container->nodes = active_nodes;
        val_container->values = values;

        // add to vector the element pointer
        // add to map name-element
        // add to map element-info
        _items_ref.push_back(item_with_ref);
        _info_items_ref[item_with_ref] = val_container;
        _elem_map[item_with_ref->getName()] = item_with_ref;

        return true;

    }

    bool addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes = {})
    {

        auto active_nodes = _check_active_nodes(nodes);

        InfoContainer::Ptr nodes_container = std::make_unique<InfoContainer>();
        nodes_container->nodes = active_nodes;


        // add to vector the element pointer
        // add to map name-element
        // add to map element-info
        _constraints.push_back(constraint);
        _info_constraints[constraint] = nodes_container;
        _elem_map[constraint->getName()] = constraint;
        _info_elements[constraint] = nodes_container;


        return true;
    }

    bool addCost(ItemBase::Ptr cost, std::vector<int> nodes = {})
    {
        // check if added item is actually a cost?
        auto active_nodes = _check_active_nodes(nodes);

        InfoContainer::Ptr nodes_container = std::make_unique<InfoContainer>();
        nodes_container->nodes = active_nodes;

        // add to vector the element pointer
        // add to map name-element
        // add to map element-info
        _costs.push_back(cost);
        _info_costs[cost] = nodes_container;
        _elem_map[cost->getName()] = cost;


        return true;
    }


    bool addVariableBounds(ItemWithBoundsBase::Ptr variable,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes = {})
    {

        auto active_nodes = _check_active_nodes(nodes);

        // check if lower and upper bounds have the right size
        if (lower_bounds.cols() != active_nodes.size())
        {
            throw std::invalid_argument("Dimension of lower bounds inserted ("
                                  + std::to_string(lower_bounds.cols())
                                  + ") does not match number of nodes specified ("
                                  + std::to_string(active_nodes.size()) + ")");
        }

        if (upper_bounds.cols() != active_nodes.size())
        {
            throw std::invalid_argument("Dimension of upper bounds inserted ("
                                  + std::to_string(upper_bounds.cols())
                                  + ") does not match number of nodes specified ("
                                  + std::to_string(active_nodes.size()) + ")");
        }


        BoundsContainer::Ptr val_container = std::make_unique<BoundsContainer>();

        // update nodes and bounds
        // bounds are updated only on the active nodes
        val_container->nodes = active_nodes;

//        val_container->lower_bounds = std::get<0>(variable->getBounds());
//        val_container->upper_bounds = std::get<1>(variable->getBounds());

//        for (int col_i = 0; col_i < active_nodes.size(); col_i++)
//        {
//            val_container->lower_bounds.col(active_nodes.at(col_i)) = lower_bounds.col(col_i);
//        }

//        for (int col_i = 0; col_i < active_nodes.size(); col_i++)
//        {
//            val_container->upper_bounds.col(active_nodes.at(col_i)) = upper_bounds.col(col_i);
//        }
        val_container->lower_bounds = lower_bounds;
        val_container->upper_bounds = upper_bounds;

        // add to vector the element pointer
        // add to map name-element
        // add to map element-info
        _variables.push_back(variable);
        _info_variables[variable] = val_container;
        _elem_map[variable->getName()] = variable;
        _info_elements[variable] = val_container;


        return true;
    }

    bool addParameterValues(ItemWithValuesBase::Ptr parameter,
                            Eigen::MatrixXd values,
                            std::vector<int> nodes = {})
    {

        /*
         * add to horizon parameter a desired value
         */
        auto active_nodes = _check_active_nodes(nodes);


        // check if values have the right size w.r.t. the nodes selected
        if (values.cols() != active_nodes.size())
        {
            throw std::invalid_argument("Dimension of values inserted ("
                                  + std::to_string(values.cols())
                                  + ") does not match number of nodes specified ("
                                  + std::to_string(active_nodes.size()) + ")");
        }

        ValuesContainer::Ptr val_container = std::make_unique<ValuesContainer>();

//        std::cout <<" nodes: " << std::endl;
//        for (auto node : active_nodes)
//        {
//            std::cout << node << " ";
//        }
//        std::cout << std::endl;

//        std::cout <<" values: " << std::endl;
//        std::cout << values << std::endl;

        val_container->nodes = active_nodes;
        val_container->values = values;

        _parameters.push_back(parameter);
        _info_parameters[parameter] = val_container;
        _elem_map[parameter->getName()] = parameter;
        _info_elements[parameter] = val_container;

        return true;
    }

    std::unordered_set<int> getSetNodes();
    std::vector<ItemBase::Ptr> getItems();
    std::vector<ItemWithValuesBase::Ptr> getItemsReference();

    std::vector<ItemWithBoundsBase::Ptr> getConstraints();
    std::vector<ItemBase::Ptr> getCosts();
    std::vector<ItemWithBoundsBase::Ptr> getVariables();
    std::vector<ItemWithValuesBase::Ptr> getParameters();

    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> getItemsInfo();
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> getItemsReferenceInfo();
    std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> getConstraintsInfo();
    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> getCostsInfo();
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> getVariablesInfo();
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> getParametersInfo();

private:

    bool _init_nodes(int n_nodes);
    Phase::InfoContainer::Ptr _get_info_element(std::string elem_name);
    std::unordered_map<int, std::vector<int>> _stretch(std::vector<int> nodes, double stretch_factor);

    std::vector<int> _check_active_nodes(std::vector<int> nodes)
    {
    // check if added nodes are correct w.r.t. the nodes of the phase
    // if nodes is empty, assume all the nodes are active

        for (int num : nodes) {
            if (_set_nodes.find(num) == _set_nodes.end()) {
                throw std::invalid_argument("Node inserted ("
                                      + std::to_string(num)
                                      + ") is outside of phase nodes.");
            }
        }

        std::vector<int> active_nodes = (nodes.empty()) ? _vec_nodes : nodes;

        return active_nodes;
    }

    std::string _name;
    int _n_nodes;
    std::unordered_set<int> _set_nodes;
    std::vector<int> _vec_nodes;

//    std::unordered_map<std::string, std::any> _elem_map;
//    std::unordered_map<std::string, std::variant<std::unordered_map<ItemBase::Ptr, std::vector<int>>,
//                                                 std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer>,
//                                                 std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>>,
//                                                 std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer>>> _elem_map;

    std::vector<ItemBase::Ptr> _items_base;
    std::vector<ItemWithValuesBase::Ptr> _items_ref;

    std::vector<ItemWithBoundsBase::Ptr> _constraints;
    std::vector<ItemBase::Ptr> _costs;
    std::vector<ItemWithBoundsBase::Ptr> _variables;
    std::vector<ItemWithValuesBase::Ptr> _parameters;


    // generic item that must have a method setNodes()
    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_items_base;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_items_ref;

    std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> _info_constraints;
    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_costs;
    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> _info_variables;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_parameters;


    std::unordered_map<std::string, ItemBase::Ptr> _elem_map;
    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_elements;

//    std::unordered_map<ItemBase::Ptr, std::vector<int>> _items_base;
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> _items_ref;

//    std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> _constraints;
//    std::unordered_map<ItemBase::Ptr, std::vector<int>> _costs;
//    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer> _variables;
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer> _parameters;


//    std::unordered_map<std::string, std::pair<ItemBase::Ptr, std::vector<int>>> _items_base;
//    std::unordered_map<std::string, std::pair<ItemWithValuesBase::Ptr, ValuesContainer>> _items_ref;

//    std::unordered_map<std::string, std::pair<ItemWithBoundsBase::Ptr, std::vector<int>>> _constraints;
//    std::unordered_map<std::string, std::pair<ItemBase::Ptr, std::vector<int>>> _costs;
//    std::unordered_map<std::string, std::pair<ItemWithBoundsBase::Ptr, BoundsContainer>> _variables;
//    std::unordered_map<std::string, std::pair<ItemWithValuesBase::Ptr, ValuesContainer>> _parameters;





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
    const std::vector<int>& getActiveNodes();
    const int getPosition();
    const int getNNodes();
    // get name, get nodes...

protected:

    PhaseToken(Phase::Ptr phase);

private:

    Phase::Ptr _abstract_phase;
    std::vector<int> _active_nodes;
    std::vector<int> _nodes;
    int _initial_node;

    // all these updates gets called by the SinglePhaseManager
    bool _update_items(int initial_node);
    bool _update_item_reference(int initial_node);
    bool _update_constraints(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_variables(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_costs(int initial_node);
    bool _update_parameters(int initial_node);

    std::pair<std::vector<int>, std::vector<int>> _compute_horizon_nodes(std::vector<int> nodes, int initial_node);

    bool _set_position(int initial_node);
    bool _update(int initial_node);

    std::vector<int>& _get_active_nodes();
    Phase::Ptr get_phase();

};

#endif
