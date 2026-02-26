// iprotocol.h - VDA5050 Protocol Interface
#pragma once
#include <string>

class IAmr;

class IProtocol 
{
public:
    virtual ~IProtocol() = default;
    
    // AMR 설정
    virtual void setAmr(IAmr* amr) = 0;
    virtual void setAgvId(const std::string& agv_id) = 0;
    
    // 프로토콜 연결 및 설정
    virtual void useDefaultConfig(const std::string& server_address) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    
    // 메시지 발행 (FMS로 송신)
    virtual void publishStateMessage(IAmr* amr) = 0;
    virtual void publishVisualizationMessage(IAmr* amr) = 0;
    
    // 메시지 생성 (내부용 - 테스트/디버깅용으로 public)
    virtual std::string makeStateMessage(IAmr* amr) = 0;
    
    // FMS에서 들어온 메시지 처리 (수신)
    virtual void handleMessage(const std::string& msg, IAmr* amr) = 0;
    
    // 프로토콜 타입 반환
    virtual std::string getProtocolType() const = 0;
    
    // 오더 완료 확인 (주기적 호출용)
    virtual void checkOrderCompletion(IAmr* amr) = 0;
};

