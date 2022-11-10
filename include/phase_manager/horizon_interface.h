#ifndef HORIZON_INTERFACE_H
#define HORIZON_INTERFACE_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <set>


// synthesis of what I need from the items in Horizon
class ItemBase
{
public:
    using ItemBasePtr = std::shared_ptr<ItemBase>;
    virtual bool setNodes(std::vector<int> nodes) = 0;
    bool addNodes(std::vector<int> nodes)
    {
        _nodes.insert(_nodes.end(), nodes.begin(), nodes.end());
        return true;
    }
    bool flushNodes()
    {
        _nodes.erase(std::unique(_nodes.begin(), _nodes.end()), _nodes.end());
        setNodes(_nodes);

        return true;
    }
    bool clearNodes()
    {
        _nodes.clear();
    }
//    std::vector<int> getNodes()
//    {
//        return _nodes;
//    }

private:
    std::vector<int> _nodes;
};

//// + bounds
class ItemWithBoundsBase : public ItemBase
{
public:
    using ItemWithBoundsBasePtr = std::shared_ptr<ItemWithBoundsBase>;
    virtual bool setBounds(Eigen::VectorXd bounds) = 0;
};


// model that calls the desired function from the original objects
template <typename T>
class Wrapper : public ItemBase
{
public:
    Wrapper(std::shared_ptr<T> item):
        m_item(item) {}

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }

private:

    std::shared_ptr<T> m_item;
};


template <typename T>
class WrapperWithBounds : public ItemWithBoundsBase
{
public:
    WrapperWithBounds(std::shared_ptr<T> item):
        m_item(item) {}

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
    bool setBounds(Eigen::VectorXd bounds) { return m_item->setBounds(bounds); }

private:

    std::shared_ptr<T> m_item;
};


#endif
