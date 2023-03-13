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
//        std::cout << "pushing back item: " << item->getName() << std::endl;
        _items.push_back(item);
    }
    return true;
}

bool HorizonManager::addItemReference(ItemWithValuesBase::Ptr item_ref)
{
    bool duplicate_flag = false;

    for (auto it : _items_ref)
    {
        if (item_ref->getName() == it->getName())
        {
            duplicate_flag = true;
        }

    }

    if (duplicate_flag == false)
    {
        _items_ref.push_back(item_ref);
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
    // todo: this flush all the nodes at each call, regardless of the item being updated or not

//    std::cout << "flushing all the nodes to horizon:" << std::endl;
    for (auto item : _items)
    {
//        std::cout << "flushing item: " << item->getName() << std::endl;
        item->flushNodes();
    }

    for (auto item_ref : _items_ref)
    {
//        std::cout << "flushing item: " << item_ref->getName() << std::endl;
        item_ref->flushNodes();
        item_ref->flushValues();
    }

    for (auto constraint : _constraints)
    {
//        std::cout << "flushing constraints: " << constraint->getName() << std::endl;
        constraint->flushNodes();
    }

    for (auto cost : _costs)
    {
//        std::cout << "flushing costs: " << cost->getName() << std::endl;
        cost->flushNodes();
    }

    for (auto variable : _variables)
    {
//        std::cout << "flushing variables: " << variable->getName() << std::endl;
//        variable->flushNodes();
        variable->flushBounds();

    }

    for (auto parameter : _parameters)
    {
//        std::cout << "flushing parameters: " << parameter->getName() << std::endl;
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

    for (auto item_ref : _items_ref)
    {
        item_ref->clearNodes();
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
