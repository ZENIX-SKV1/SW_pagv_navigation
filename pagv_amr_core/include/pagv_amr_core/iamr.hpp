#pragma once
#include <vector>
#include <string>
#include <iostream>  
#include "node_edge_info.h"

struct AmrNode 
{
    std::string id;
    double x;
    double y;
};

struct AmrEdge 
{
    std::string id;
    std::string from;
    std::string to;
};

class IVcu;  // 전방 선언

class IAmr 
{
public:
    virtual ~IAmr() = default;
    
    virtual void step(double dt) = 0;
    virtual std::string getState() const = 0;
    virtual std::vector<NodeInfo> getNodes() const = 0;
    virtual std::size_t getCurIdx() const = 0;
    virtual IVcu* getVcu() = 0;
    
    // Order 관리
    virtual void setOrder(const std::vector<NodeInfo>& nodes, 
                         const std::vector<EdgeInfo>& edges, 
                         const std::vector<NodeInfo>& all_nodes) = 0;
    virtual void cancelOrder() = 0;
    
    // Node/Edge 완료 추적
    virtual void markNodeAsCompleted(const NodeInfo& node) = 0;
    virtual std::vector<NodeInfo> getCurrentNodes() const = 0;
    virtual std::vector<EdgeInfo> getCurrentEdges() const = 0;
    virtual std::vector<NodeInfo> getCompletedNodes() const = 0;
    virtual std::vector<EdgeInfo> getCompletedEdges() const = 0;
    virtual std::string getLastNodeId() const = 0;
    virtual int getLastNodeSequenceId() const = 0;
    
    // 배터리 관리
    virtual void updateBattery(double dt, bool is_charging) = 0;
    virtual double getBatteryPercent() const = 0;
    
    // VDA5050 표준에서 필요한 배터리 전압
    virtual double getBatteryVoltage() const 
    { 
        return 0.0;  
    }
    virtual void getEstimatedPose(double& x, double& y, double& theta) const = 0;
    virtual double getLinearVelocity() const = 0;
    virtual double getAngularVelocity() const = 0;

    virtual std::string getId() const = 0;

    virtual void updateOrder(const std::vector<NodeInfo>& nodes, 
                            const std::vector<EdgeInfo>& edges,
                            const std::vector<NodeInfo>& all_nodes) 
    {
        setOrder(nodes, edges, all_nodes);
    }
    
    virtual void executeAction(const ActionInfo& action) 
    {
        std::cout << "[IAmr] Action requested: " << action.actionType 
                  << " (ID: " << action.actionId << ")" << std::endl;
    }
    
    virtual bool isPaused() const 
    { 
        return false;  
    }
    
    virtual void pause() 
    {
        std::cout << "[IAmr] Pause requested" << std::endl;
    }
    
    virtual void resume() 
    {
        std::cout << "[IAmr] Resume requested" << std::endl;
    }
    
    virtual bool isDriving() const 
    {
        return false;  
    }
    
    virtual void setMaxSpeed(double speed) 
    {
        std::cout << "[IAmr] Max speed set to: " << speed << " m/s" << std::endl;
    }
    
    virtual std::string getOperatingMode() const 
    {
        return "AUTOMATIC";  
    }
    
    virtual bool needsImmediateStatePublish() const 
    {
        return false;  
    }
    
    virtual void resetImmediateStatePublishFlag() 
    {
    }
};