#ifndef HORIZON_INTERFACE_H
#define HORIZON_INTERFACE_H

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <set>
#include <iostream>

// synthesis of what I need from the items in Horizon
class ItemBase
{
public:
    using ItemBasePtr = std::shared_ptr<ItemBase>;
    virtual bool setNodes(std::vector<int> nodes) = 0;
//    virtual std::string getName() = 0;
    bool addNodes(std::vector<int> nodes)
    {
        _nodes.insert(_nodes.end(), nodes.begin(), nodes.end());
        return true;
    }
    bool flush()
    {
        _nodes.erase(std::unique(_nodes.begin(), _nodes.end()), _nodes.end());
        setNodes(_nodes);

        return true;
    }
    bool clearNodes()
    {
        _nodes.clear();
    }

protected:
    std::vector<int> _nodes;
};

//// + bounds
class ItemWithBoundsBase : public ItemBase
{
public:

    using ItemWithBoundsBasePtr = std::shared_ptr<ItemWithBoundsBase>;
    virtual bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds) = 0;

    bool addBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds)
    {
        std::cout << "setting bounds to nodes: ";
        for (auto node : _nodes)
        {
            std::cout << node << " ";
        }
        std::cout << std::endl;
        _lower_bounds(Eigen::placeholders::all, _nodes) << lower_bounds;
        _upper_bounds(Eigen::placeholders::all, _nodes) << upper_bounds;
    }

    bool flush()
    {
        ItemBase::flush();
        std::cout << _upper_bounds << std::endl;
        setBounds(_lower_bounds, _upper_bounds);
        return true;
    }

protected:
    // do I need to initialize these?
    Eigen::MatrixXd _lower_bounds;
    Eigen::MatrixXd _upper_bounds;
};


// model that calls the desired function from the original objects
template <typename T>
class Wrapper : public ItemBase
{
public:
    Wrapper(std::shared_ptr<T> item):
        m_item(item) {}

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
//    std::string getName() { return m_item->getName(); }

private:

    std::shared_ptr<T> m_item;
};


template <typename T>
class WrapperWithBounds : public ItemWithBoundsBase
{
public:
    WrapperWithBounds(std::shared_ptr<T> item):
        m_item(item)
        {
            // what if empty?
            _lower_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(m_item->getDim(), m_item->getNodes().size());
            _upper_bounds = std::numeric_limits<double>::infinity() * Eigen::MatrixXd::Ones(m_item->getDim(), m_item->getNodes().size());
        }

    bool setNodes(std::vector<int> nodes) { return m_item->setNodes(nodes); }
    bool setBounds(Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds) { return m_item->setBounds(lower_bounds, upper_bounds); }
//    std::string getName() { return m_item->getName(); }

private:

    std::shared_ptr<T> m_item;
};


#endif
