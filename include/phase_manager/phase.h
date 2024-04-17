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


class InfoContainer
{

public:

    typedef std::shared_ptr<InfoContainer> Ptr;
    InfoContainer() {}

//    InfoContainer(const InfoContainer::Ptr other) :
//        nodes(other->nodes) {}

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

//  copy constructor
    ValuesContainer(const ValuesContainer::Ptr other) :
        values(other->values)
        {
            nodes = other->nodes;
        }

    // must be of same dimension
    std::string getType() {return "values";}
    // TODO: what happen if not same dimension?
    Eigen::MatrixXd values;
};


class Phase
{
    /*
     * container of any object compatible (i.e. has a setNodes() at least).
     * It represent an abstract phase that can be used to define constraints, costs, bounds active on certain segments of nodes.
     */

public:

    typedef std::shared_ptr<Phase> Ptr;


    Phase(int n_nodes, std::string name);

    std::string getName();
    int getNNodes();
    bool setDuration(int new_n_nodes);
    bool setElemNodes(std::string elem_name, std::vector<int> nodes,
                      const Eigen::MatrixXd& value_1 = Eigen::MatrixXd(),
                      const Eigen::MatrixXd& value_2 = Eigen::MatrixXd());

    bool addItem(ItemBase::Ptr item, std::vector<int> nodes = {});

    bool addItemReference(ItemWithValuesBase::Ptr item_with_ref,
                          Eigen::MatrixXd values,
                          std::vector<int> nodes = {});

    bool addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes = {});

    bool addCost(ItemBase::Ptr cost, std::vector<int> nodes = {});


    bool addVariableBounds(ItemWithBoundsBase::Ptr variable,
                           Eigen::MatrixXd lower_bounds,
                           Eigen::MatrixXd upper_bounds,
                           std::vector<int> nodes = {});

    bool addParameterValues(ItemWithValuesBase::Ptr parameter,
                            Eigen::MatrixXd values,
                            std::vector<int> nodes = {});

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
    InfoContainer::Ptr _get_info_element(std::string elem_name);
    std::unordered_map<int, std::vector<int>> _stretch(std::vector<int> nodes, double stretch_factor);
    std::vector<int> _extract_stretch_nodes(std::unordered_map<int, std::vector<int>> stretch_map, std::vector<int> nodes);


    std::vector<int> _check_active_nodes(std::vector<int> nodes);

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

//    PhaseManager::Ptr _phase_manager;

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

    friend class Timeline;
    friend class HorizonManager;

public:

    typedef std::shared_ptr<PhaseToken> Ptr;
    std::string getName();
    const std::vector<int>& getActiveNodes();
    const int getPosition();
    const int getNNodes();

    bool setItemReference(std::string item_name,
                          Eigen::MatrixXd values);

    bool update();

protected:

    PhaseToken(Phase::Ptr phase);

private:

    Phase::Ptr _abstract_phase;
    std::vector<int> _active_nodes;
//    std::vector<int> _nodes;
    int _initial_node;

    // change ref of single phasetoken
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_items_ref_token;
    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_parameters_token;

    // all these updates gets called by the SinglePhaseManager // or can be called manually
    bool _update_items(int initial_node);
    bool _update_item_reference(int initial_node);
    bool _update_constraints(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_variables(int initial_node); //std::vector<std::string, std::set<int>>* input_container, std::vector<std::string, std::set<int>>* output_container
    bool _update_costs(int initial_node);
    bool _update_parameters(int initial_node);

    std::pair<std::vector<int>, std::vector<int>> _compute_horizon_nodes(std::vector<int> nodes, int initial_node);

    bool _set_position(int initial_node);

//    Eigen::MatrixXd _item_reference_values_to_assign;

    std::vector<int>& _get_active_nodes();
    Phase::Ptr get_phase();


};

#endif
