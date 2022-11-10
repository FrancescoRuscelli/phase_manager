#include <phase_manager/horizon_manager.h>

bool HorizonManager::addConstraint(ItemWithBoundsBase::ItemWithBoundsBasePtr constraint)
{
    // should push pointer
    _constraints.push_back(constraint);
}

bool HorizonManager::flush()
{
    for (auto constraint : _constraints)
    {
        constraint->flushNodes();
    }
}

bool HorizonManager::reset()
{
    for (auto constraint : _constraints)
    {
        constraint->clearNodes();
    }
}
