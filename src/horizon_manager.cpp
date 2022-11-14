#include <phase_manager/horizon_manager.h>

bool HorizonManager::addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint)
{
    // should push pointer
    _constraints.insert(constraint);
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
        _variables.insert(variable);
    }

}

bool HorizonManager::flush()
{
    for (auto constraint : _constraints)
    {
        constraint->flush();
    }

    for (auto variables : _variables)
    {
        variables->flush();
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

}
