#include <phase_manager/horizon_manager.h>

bool HorizonManager::addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint)
{
    bool duplicate_flag = false;
    for (auto item : _constraints)
    {
        if (constraint->getName() == item->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _constraints.push_back(constraint);
    }

}

bool HorizonManager::addCost(ItemBase::ItemBasePtr cost)
{
    bool duplicate_flag = false;
    for (auto item : _costs)
    {
        if (cost->getName() == item->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _costs.push_back(cost);
    }

}

bool HorizonManager::addVariable(ItemWithBoundsBase::ItemWithBoundsBasePtr variable)
{
    bool duplicate_flag = false;
    for (auto item : _variables)
    {
        if (variable->getName() == item->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _variables.push_back(variable);
    }

}

bool HorizonManager::addParameter(ItemWithValuesBase::ItemWithValuesBasePtr parameter)
{
    bool duplicate_flag = false;
    for (auto item : _parameters)
    {
        if (parameter->getName() == item->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _parameters.push_back(parameter);
    }

}

bool HorizonManager::flush()
{
    for (auto constraint : _constraints)
    {
        constraint->flush();
    }

    for (auto cost : _costs)
    {
        cost->flush();
    }

    for (auto variable : _variables)
    {
        variable->flush();
    }

    for (auto parameter : _parameters)
    {
        parameter->flush();
    }


}

bool HorizonManager::reset()
{
    for (auto constraint : _constraints)
    {
        constraint->clearNodes();
        constraint->clearBounds();
    }

    for (auto variable : _variables)
    {
        variable->clearNodes();
        variable->clearBounds();
    }

    for (auto parameter : _parameters)
    {
        parameter->clearNodes();
        parameter->clearBounds();
    }

}
