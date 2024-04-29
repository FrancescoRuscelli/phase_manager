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


class Timeline;

// constraints .setNodes()
// costs .setNodes()
// item .setNodes()
class NodesManager
{
public:

    typedef std::shared_ptr<NodesManager> Ptr;

    NodesManager(ItemBase::Ptr item,
                 int n_phase_nodes,
                 std::vector<int> selected_nodes) : _item(item), _n_phase_nodes(n_phase_nodes), _selected_nodes(selected_nodes)
    {
        _name = _item->getName();
    }

    std::string getName() {return _name; }
    int getNNodes() {return _n_phase_nodes;}
    std::vector<int> getSelectedNodes() {return _selected_nodes;}
    ItemBase::Ptr getItem() {return _item;}
    virtual bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes) = 0;

private:

    std::string _name;
    ItemBase::Ptr _item;
    int _n_phase_nodes;
    std::vector<int> _selected_nodes;

};

// variables  .setBounds()
class BoundsManager : public NodesManager
{
public:

    typedef std::shared_ptr<BoundsManager> Ptr;

     BoundsManager(ItemWithBoundsBase::Ptr item_with_bounds,
                   int n_phase_nodes,
                   std::vector<int> active_nodes,
                   Eigen::MatrixXd lower_bounds,
                   Eigen::MatrixXd upper_bounds) : NodesManager(item_with_bounds, n_phase_nodes, active_nodes), _lower_bounds(lower_bounds), _upper_bounds(upper_bounds)
     {

        std::cout << "creating bounds manager with: " << std::endl;
        std::cout << "lower bounds: " << _lower_bounds << std::endl;
        std::cout << "upper bounds: " << _upper_bounds << std::endl;

        std::cout << "n nodes: " << getNNodes() << std::endl;
        std::cout << "selected nodes: " << std::endl;
        for (auto node : getSelectedNodes())
        {
            std::cout << node << " ";

        }
        std::cout << std::endl;
        std::cout << " ========= " << std::endl;
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
     }

protected:

    Eigen::MatrixXd _lower_bounds;
    Eigen::MatrixXd _upper_bounds;

};

// parameter  .assign()
// itemref    .setNodes() .assign()
class ValuesManager : public NodesManager
{
public:

    typedef std::shared_ptr<ValuesManager> Ptr;

     ValuesManager(ItemWithValuesBase::Ptr item_with_values,
                   int n_phase_nodes,
                   std::vector<int> selected_nodes,
                   Eigen::MatrixXd values) : NodesManager(item_with_values, n_phase_nodes, selected_nodes)
     {

         if (values.cols() != selected_nodes.size())
         {
             throw std::invalid_argument("Dimension of values inserted ("
                                         + std::to_string(values.cols())
                                         + ") does not match number of nodes specified ("
                                         + std::to_string(selected_nodes.size()) + ")");
         }

         // initialize values to zero
         _values = Eigen::MatrixXd::Zero(item_with_values->getDim(), getNNodes());

         //        std::cout << "active nodes:" << std::endl;
         //        for (auto node: active_nodes)
         //        {
         //            std::cout << node << " ";
         //        }
         //        std::cout << std::endl;
         // set desired values
        _set_values(values);

     }

     bool setValues(Eigen::MatrixXd values)
     {
         return _set_values(values);
     }

protected:

    Eigen::MatrixXd _values;

private:

    bool _set_values(Eigen::MatrixXd values)
    {
        int col_values = 0;
        for (auto node : getSelectedNodes())
        {
            _values.col(node) = values.col(col_values);
            col_values++;
        }
        return true;

    }

};

class ConstraintManager : public NodesManager
{

public:

    typedef std::shared_ptr<ConstraintManager> Ptr;

    ConstraintManager(ItemBase::Ptr item,
                      int n_phase_nodes,
                      std::vector<int> active_nodes) : NodesManager(item, n_phase_nodes, active_nodes) {}

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {
        getItem()->setNodes(absolute_nodes, false);
        return true;
    }

};

class CostManager : public NodesManager
{
public:

    typedef std::shared_ptr<CostManager> Ptr;

    CostManager(ItemBase::Ptr item,
                int n_phase_nodes,
                std::vector<int> active_nodes) : NodesManager(item, n_phase_nodes, active_nodes) {}

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {
        getItem()->setNodes(absolute_nodes, false);
        return true;
    }

};


class ItemManager : public NodesManager
{
public:

    typedef std::shared_ptr<ItemManager> Ptr;

    ItemManager(ItemBase::Ptr item,
                int n_phase_nodes,
                std::vector<int> active_nodes) : NodesManager(item, n_phase_nodes, active_nodes) {}

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {
        getItem()->setNodes(absolute_nodes, false);
        return true;
    }

};

class ItemReferenceManager : public ValuesManager
{
public:
    typedef std::shared_ptr<ItemReferenceManager> Ptr;

    ItemReferenceManager(ItemWithValuesBase::Ptr item_with_values,
                         int n_phase_nodes,
                         std::vector<int> active_nodes,
                         Eigen::MatrixXd values) : ValuesManager(item_with_values, n_phase_nodes, active_nodes, values)
    {

    }

    ItemReferenceManager(const ItemReferenceManager &other) : ValuesManager(other) {}

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {
        Eigen::MatrixXd vals_sliced;
        vals_sliced.resize(_values.rows(), active_phase_nodes.size());

        for (int col_i = 0; col_i < vals_sliced.cols(); col_i++)
        {
            vals_sliced.col(col_i) = _values.col(active_phase_nodes[col_i]);
        }

        std::dynamic_pointer_cast<ItemWithValuesBase>(getItem())->setNodes(absolute_nodes, false);
        std::dynamic_pointer_cast<ItemWithValuesBase>(getItem())->assign(vals_sliced, absolute_nodes);
        return true;
    }

};

class ParameterManager : public ValuesManager
{
public:

    typedef std::shared_ptr<ParameterManager> Ptr;

    ParameterManager(ItemWithValuesBase::Ptr item_with_values,
                     int n_phase_nodes,
                     std::vector<int> active_nodes,
                     Eigen::MatrixXd values) : ValuesManager(item_with_values, n_phase_nodes, active_nodes, values)
    {

    }

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {

        Eigen::MatrixXd vals;
        vals.resize(_values.rows(), active_phase_nodes.size());

        for (int col_i = 0; col_i < vals.cols(); col_i++)
        {
            vals.col(col_i) = _values.col(active_phase_nodes[col_i]);
        }

        std::dynamic_pointer_cast<ItemWithValuesBase>(getItem())->assign(vals, absolute_nodes);

        return true;

    }

};

class VariableManager : public BoundsManager
{
public:

    typedef std::shared_ptr<VariableManager> Ptr;

    VariableManager(ItemWithBoundsBase::Ptr item_with_bounds,
                    int n_phase_nodes,
                    std::vector<int> active_nodes,
                    Eigen::MatrixXd lower_bounds,
                    Eigen::MatrixXd upper_bounds) : BoundsManager(item_with_bounds, n_phase_nodes, active_nodes, lower_bounds, upper_bounds)
    {
    }

    bool update(std::vector<int> active_phase_nodes, std::vector<int> absolute_nodes)
    {
        Eigen::MatrixXd lb(_lower_bounds.rows(), active_phase_nodes.size());
        Eigen::MatrixXd ub(_upper_bounds.rows(), active_phase_nodes.size());

        std::vector<int> selected_nodes = getSelectedNodes();

        for (int col_i = 0; col_i < active_phase_nodes.size(); col_i++)
        {
            // find the column of the values matrix corresponding to the active nodes
            auto itr = std::find(selected_nodes.begin(), selected_nodes.end(), active_phase_nodes[col_i]);
            if (itr == selected_nodes.end())
            {
                throw std::runtime_error(std::string("SOMETHING WRONG IN UPDATE of VARIABLE MANAGER") + getItem()->getName() + std::string("element NOT FOUND"));
            }
            int index = std::distance(selected_nodes.begin(), itr);
            lb.col(col_i) = _lower_bounds.col(index);
            ub.col(col_i) = _upper_bounds.col(index);
        }
        std::dynamic_pointer_cast<ItemWithBoundsBase>(getItem())->setBounds(lb, ub, absolute_nodes);
        return true;
    }

};


class Phase
{
    /*
     * container of any object compatible (i.e. has a setNodes() at least).
     * It represent an abstract phase that can be used to define constraints, costs, bounds active on certain segments of nodes.
     */

public:

    typedef std::shared_ptr<Phase> Ptr;


    Phase(Timeline& timeline, int n_nodes, std::string name);

    std::string getName();
    int getNNodes();
    bool setDuration(int new_n_nodes);
//    bool setElemNodes(std::string elem_name, std::vector<int> nodes,
//                      const Eigen::MatrixXd& value_1 = Eigen::MatrixXd(),
//                      const Eigen::MatrixXd& value_2 = Eigen::MatrixXd());

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

    std::unordered_set<int> getNodesAsSet();

    std::vector<ItemBase::Ptr> getItems();
    std::vector<ItemWithValuesBase::Ptr> getItemsReference();

    std::vector<ItemWithBoundsBase::Ptr> getConstraints();
    std::vector<ItemBase::Ptr> getCosts();
    std::vector<ItemWithBoundsBase::Ptr> getVariables();
    std::vector<ItemWithValuesBase::Ptr> getParameters();

    std::vector<NodesManager::Ptr> getElements();

//    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> getItemsInfo();
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> getItemsReferenceInfo();
//    std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> getConstraintsInfo();
//    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> getCostsInfo();
//    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> getVariablesInfo();
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> getParametersInfo();

private:

    ItemBase::Ptr _add_element(ItemBase::Ptr elem);
    bool _add_element_manager(NodesManager::Ptr element);
    bool _init_nodes(int n_nodes);
//    InfoContainer::Ptr _get_info_element(std::string elem_name);
    std::unordered_map<int, std::vector<int>> _stretch(std::vector<int> nodes, double stretch_factor);
    std::vector<int> _extract_stretch_nodes(std::unordered_map<int, std::vector<int>> stretch_map, std::vector<int> nodes);


    std::vector<int> _check_active_nodes(std::vector<int> nodes);

    std::string _name;
    int _n_nodes;
    std::unordered_set<int> _nodes_as_set;
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
    std::vector<std::shared_ptr<NodesManager>> _elements;


//    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_items_base;
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_items_ref;

//    std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> _info_constraints;
//    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_costs;
//    std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> _info_variables;
//    std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> _info_parameters;


//    std::unordered_map<std::string, ItemBase::Ptr> _elem_map;
//    std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> _info_elements;

    Timeline& _timeline;

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
    int _initial_node;

    // change ref of single phasetoken
//    std::vector<ItemReferenceManager::Ptr> _item_refs_token;
//    std::vector<ParameterManager::Ptr> _parameters_token;

    std::unordered_map<std::string, NodesManager::Ptr> _cloned_elements;

    std::pair<std::vector<int>, std::vector<int>> _compute_horizon_nodes(std::vector<int> nodes, int initial_node);

    bool _set_position(int initial_node);

    std::vector<int>& _get_active_nodes();
    Phase::Ptr get_phase();


};

#endif
