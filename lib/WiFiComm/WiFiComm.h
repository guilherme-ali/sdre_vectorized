/**
 * WiFiComm.h
 * 
 * Biblioteca para comunicação WiFi/UDP com protocolo Crazyflie
 * Compatível com LiteWing app
 */

#ifndef WIFICOMM_H
#define WIFICOMM_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Estrutura para comandos do joystick (valores típicos do Crazyflie)
struct CommanderPacket {
    float roll;      // -1.0 a 1.0
    float pitch;     // -1.0 a 1.0
    float yaw;       // -1.0 a 1.0
    uint16_t thrust; // 0 a 65535
};

class WiFiComm {
public:
    WiFiComm(const char* ssid = "ESP-DRONE", 
             const char* password = "12345678",
             int udpPort = 2390);
    
    // Métodos principais
    bool begin();
    void update();
    bool isClientConnected();
    
    // Getters para comandos recebidos
    CommanderPacket getLastCommand();
    bool hasNewCommand();
    
    // Configuração de IP
    void setIP(IPAddress local, IPAddress gateway, IPAddress subnet);
    
    // Configuração de callbacks
    void onCommandReceived(void (*callback)(CommanderPacket));
    void onClientConnected(void (*callback)());
    void onClientDisconnected(void (*callback)());
    
    // Getters de status
    IPAddress getClientIP();
    int getClientPort();
    unsigned long getPacketCount();
    unsigned long getLastPacketTime();
    
    // Configurações de debug
    void enableDebug(bool enable);
    void enableVerbose(bool enable);

private:
    // Configurações WiFi
    const char* _ssid;
    const char* _password;
    IPAddress _local_IP;
    IPAddress _gateway;
    IPAddress _subnet;
    
    // Configurações UDP
    int _udpPort;
    WiFiUDP _udp;
    
    // Buffer para pacotes
    uint8_t _packetBuffer[256];
    
    // Status da conexão
    IPAddress _clientIP;
    int _clientPort;
    bool _clientConnected;
    unsigned long _lastPacketTime;
    unsigned long _packetCount;
    int _lastNumClients;
    unsigned long _lastStatusCheck;
    
    // Comandos
    CommanderPacket _lastCommand;
    bool _newCommandAvailable;
    
    // Callbacks
    void (*_onCommandCallback)(CommanderPacket);
    void (*_onClientConnectedCallback)();
    void (*_onClientDisconnectedCallback)();
    
    // Flags de debug
    bool _debugEnabled;
    bool _verboseEnabled;
    
    // Métodos internos
    void processPacket(uint8_t* buffer, int length, IPAddress remoteIP, int remotePort);
    void processCommanderPacket(uint8_t* buffer, int length);
    
    // Respostas CRTP
    void sendPongResponse(IPAddress ip, int port);
    void sendParameterResponse(IPAddress ip, int port, uint8_t* request, int len);
    void sendLoggingResponse(IPAddress ip, int port, uint8_t* request, int len);
    void sendMemoryResponse(IPAddress ip, int port, uint8_t* request, int len);
    void sendPlatformResponse(IPAddress ip, int port, uint8_t* request, int len);
    
    // Monitoramento de status
    void checkConnectionStatus();
    void updateClientConnection(IPAddress remoteIP, int remotePort);
};

#endif // WIFICOMM_H
