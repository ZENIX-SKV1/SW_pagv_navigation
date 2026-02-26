#ifndef PAGV_AMR_CORE__ORDER_EXECUTION__ORDER_EXECUTION_MANAGER_HPP_
#define PAGV_AMR_CORE__ORDER_EXECUTION__ORDER_EXECUTION_MANAGER_HPP_

#include <pagv_amr_core/node_edge_info.h>
#include <vector>
#include <string>

namespace pagv_amr_core 
{

enum class OrderState 
{
    IDLE, VALIDATING, EXECUTING, PAUSED, COMPLETED, CANCELLED, FAILED
};

struct Order 
{
    std::string order_id;
    std::vector<NodeInfo> nodes;
    std::vector<EdgeInfo> edges;
};

class OrderExecutionManager 
{
public:
    OrderExecutionManager() = default;
    
    void set_order(const Order& order);
    bool has_order() const { return state_ != OrderState::IDLE; }
    const Order& get_current_order() const { return current_order_; }
    
    bool has_next_node() const;
    const NodeInfo& get_current_node() const;
    const NodeInfo& get_next_node();
    void advance_to_next_node();
    
    size_t get_current_node_index() const { return current_node_index_; }
    size_t get_total_nodes() const { return current_order_.nodes.size(); }
    
    void complete() { state_ = OrderState::COMPLETED; }
    
private:
    Order current_order_;
    OrderState state_{OrderState::IDLE};
    size_t current_node_index_{0};
};

} 

#endif
