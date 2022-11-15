#ifndef HORIZON_MANAGER_H
#define HORIZON_MANAGER_H

#include <phase_manager/horizon_interface.h>
#include <phase_manager/phase_manager.h>

#include <Eigen/Dense>
#include <vector>

class HorizonManager
{
public:


    bool addConstraint(ItemWithBoundsBase::Ptr constraint);
    bool addCost(ItemBase::Ptr cost);
    bool addVariable(ItemWithBoundsBase::Ptr variable);
    bool addParameter(ItemWithValuesBase::Ptr parameter);

    bool flush();
    bool reset();


private:

    std::vector<ItemWithBoundsBase::Ptr> _constraints;
    std::vector<ItemBase::Ptr> _costs;
    std::vector<ItemWithBoundsBase::Ptr> _variables;
    std::vector<ItemWithValuesBase::Ptr> _parameters;



};

#endif
