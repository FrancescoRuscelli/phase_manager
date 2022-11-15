#ifndef HORIZON_MANAGER_H
#define HORIZON_MANAGER_H

#include <phase_manager/horizon_interface.h>
#include <phase_manager/phase_manager.h>

#include <Eigen/Dense>
#include <vector>

class HorizonManager
{
public:


    bool addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint);
    bool addCost(ItemBase::ItemBasePtr cost);
    bool addVariable(ItemWithBoundsBase::ItemWithBoundsBasePtr variable);
    bool addParameter(ItemWithValuesBase::ItemWithValuesBasePtr parameter);

    bool flush();
    bool reset();


private:

    std::vector<ItemWithBoundsBase::ItemWithBoundsBasePtr> _constraints;
    std::vector<ItemBase::ItemBasePtr> _costs;
    std::vector<ItemWithBoundsBase::ItemWithBoundsBasePtr> _variables;
    std::vector<ItemWithValuesBase::ItemWithValuesBasePtr> _parameters;



};

#endif
