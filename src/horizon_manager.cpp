#include <phase_manager/horizon_manager.h>

bool HorizonManager::addConstraint(ItemWithBoundsBase::Ptr constraint)
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
    return true;
}

bool HorizonManager::addCost(ItemBase::Ptr cost)
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
    return true;
}

bool HorizonManager::addVariable(ItemWithBoundsBase::Ptr variable)
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
    return true;
}

bool HorizonManager::addParameter(ItemWithValuesBase::Ptr parameter)
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
    return true;
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

    return true;

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
    return true;
}
