#include <phase_manager/horizon_manager.h>

// TODO: this is an horrible implementation: if I add new items in phase, I should add them here
// implement automatic detection.
//      given a list of different elements?
// this class
HorizonManager::HorizonManager()
{
}

bool HorizonManager::addItem(ItemBase::Ptr item)
{
    bool duplicate_flag = false;

    for (auto it : _items)
    {
        if (item->getName() == it->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _items.push_back(item);
    }
    return true;
}

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

    for (auto item : _items)
    {
        item->flushNodes();
    }

    for (auto constraint : _constraints)
    {
        constraint->flushNodes();
    }

    for (auto constraint : _constraints)
    {
        constraint->flushNodes();
    }

    for (auto cost : _costs)
    {
        cost->flushNodes();
    }

    for (auto variable : _variables)
    {
//        variable->flushNodes();
        variable->flushBounds();

    }

    for (auto parameter : _parameters)
    {
//        parameter->flushNodes();
        parameter->flushValues();
    }

    return true;

}

bool HorizonManager::reset()
{

    for (auto item : _items)
    {
        item->clearNodes();
    }


    for (auto constraint : _constraints)
    {
        constraint->clearNodes();
        constraint->clearBounds();
    }

    for (auto cost : _costs)
    {
        cost->clearNodes();
    }

    for (auto variable : _variables)
    {
        variable->clearNodes();
        variable->clearBounds();
    }

    for (auto parameter : _parameters)
    {
        parameter->clearNodes();
        parameter->clearValues();
    }
    return true;
}
