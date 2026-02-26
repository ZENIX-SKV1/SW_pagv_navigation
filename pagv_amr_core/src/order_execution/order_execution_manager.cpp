#include "pagv_amr_core/order_execution/order_execution_manager.hpp"
#include <stdexcept>
#include <iostream>

namespace pagv_amr_core 
{
void OrderExecutionManager::set_order(const Order& order)
{
    current_order_ = order;
    current_node_index_ = 0;
    state_ = OrderState::EXECUTING;
    
    std::cout << "[OrderExecutionManager] Order set: " << order.order_id << std::endl;
    std::cout << "  - Nodes: " << order.nodes.size() << std::endl;
}

bool OrderExecutionManager::has_next_node() const
{
    return current_node_index_ < current_order_.nodes.size();
}

const NodeInfo& OrderExecutionManager::get_current_node() const
{
    if (current_node_index_ >= current_order_.nodes.size()) {
        throw std::out_of_range("No current node available");
    }
    return current_order_.nodes[current_node_index_];
}

const NodeInfo& OrderExecutionManager::get_next_node()
{
    if (!has_next_node()) {
        throw std::out_of_range("No next node available");
    }
    return current_order_.nodes[current_node_index_];
}

void OrderExecutionManager::advance_to_next_node()
{
    if (has_next_node()) {
        current_node_index_++;
        std::cout << "[OrderExecutionManager] Advanced to node " 
                  << current_node_index_ << "/" << current_order_.nodes.size() << std::endl;
        
        if (!has_next_node()) {
            std::cout << "[OrderExecutionManager] All nodes completed" << std::endl;
            state_ = OrderState::COMPLETED;
        }
    }
}

} // namespace pagv_amr_core
