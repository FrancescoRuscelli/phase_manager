#ifndef HORIZON_MANAGER_H
#define HORIZON_MANAGER_H

#include <phase_manager/horizon_interface.h>
#include <phase_manager/phase_manager.h>

#include <Eigen/Dense>
#include <vector>
#include <unordered_map> // use unordered map?

class HorizonManager
{
public:

    bool addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint);

    bool flush();
    bool reset();


private:

    std::vector<ItemWithBoundsBase::ItemWithBoundsBasePtr> _constraints;
//    std::vector<ItemBase::ItemBasePtr> _costs;
    std::vector<ItemWithBoundsBase::ItemWithBoundsBasePtr> _vars;
//    std::vector<HorizonFunction> _pars;


//    class VariableUpdater
//    {
//    public:

//        VariableUpdater(HorizonVariable var);
//        bool update(int nodes, Eigen::VectorXd bounds, std::vector<int> phase_nodes);

//    private:

//        bool _set_nodes_to_horizon();

//        std::string _name;
//        HorizonVariable _var;
//        int _var_nodes;
//        int _var_dim;
//        int _active_nodes;
//        int _horizon_nodes;

//        Eigen::VectorXd _lower_bounds;
//        Eigen::VectorXd _upper_bounds;

//    };

//    class ParameterUpdater
//    {
//    public:

//        ParameterUpdater(HorizonParameter par);
//        bool update(int nodes, Eigen::VectorXd values, std::vector<int> phase_nodes);

//    private:

//        bool _set_nodes_to_horizon();

//        std::string _name;
//        HorizonVariable _par;
//        int _par_nodes;
//        int _par_dim;
//        int _active_nodes;
//        int _horizon_nodes;

//        Eigen::VectorXd _lower_bounds;
//        Eigen::VectorXd _upper_bounds;

//    };


};

#endif
