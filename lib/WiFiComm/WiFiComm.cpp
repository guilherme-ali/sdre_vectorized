/**
 * WiFiComm.cpp
 * 
 * Implementação da biblioteca de comunicação WiFi/UDP com protocolo Crazyflie
 */

#include "WiFiComm.h"

WiFiComm::WiFiComm(const char* ssid, const char* password, int udpPort) 
    : _ssid(ssid), 
      _password(password),
      _udpPort(udpPort),
      _clientConnected(false),
      _lastPacketTime(0),
      _packetCount(0),
      _lastNumClients(-1),
      _lastStatusCheck(0),
      _newCommandAvailable(false),
      _onCommandCallback(nullptr),
      _onClientConnectedCallback(nullptr),
      _onClientDisconnectedCallback(nullptr),
      _debugEnabled(false),
      _verboseEnabled(false)
{
    // IP padrão do LiteWing
    _local_IP = IPAddress(192, 168, 43, 42);
    _gateway = IPAddress(192, 168, 43, 1);
    _subnet = IPAddress(255, 255, 255, 0);
    
    // Inicializa comando com zeros
    _lastCommand.roll = 0;
    _lastCommand.pitch = 0;
    _lastCommand.yaw = 0;
    _lastCommand.thrust = 0;
}

void WiFiComm::setIP(IPAddress local, IPAddress gateway, IPAddress subnet) {
    _local_IP = local;
    _gateway = gateway;
    _subnet = subnet;
}

bool WiFiComm::begin() {
    if (_debugEnabled) {
        Serial.println("\n╔════════════════════════════════════╗");
        Serial.println("║  ESP32 DRONE - LiteWing Compatible ║");
        Serial.println("╚════════════════════════════════════╝\n");
        Serial.println("📡 Configurando WiFi Access Point...");
        Serial.print("   SSID: ");
        Serial.println(_ssid);
        Serial.print("   Password: ");
        Serial.println(_password);
    }
    
    // Configura IP fixo
    if (!WiFi.softAPConfig(_local_IP, _gateway, _subnet)) {
        if (_debugEnabled) {
            Serial.println("❌ Falha ao configurar IP fixo!");
        }
        return false;
    }
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(_ssid, _password);
    
    delay(100);
    
    IPAddress IP = WiFi.softAPIP();
    
    if (_debugEnabled) {
        Serial.print("\n✅ WiFi AP iniciado!\n");
        Serial.print("   IP: ");
        Serial.println(IP);
        Serial.print("   Porta UDP: ");
        Serial.println(_udpPort);
    }
    
    // Inicia servidor UDP
    _udp.begin(_udpPort);
    
    if (_debugEnabled) {
        Serial.println("\n✅ Servidor UDP iniciado!");
        Serial.println("\n╔════════════════════════════════════╗");
        Serial.println("║       COMO CONECTAR O APP          ║");
        Serial.println("╠════════════════════════════════════╣");
        Serial.print("║ 1. Conecte ao WiFi: ");
        Serial.print(_ssid);
        for (int i = strlen(_ssid); i < 15; i++) Serial.print(" ");
        Serial.println("║");
        Serial.print("║ 2. Senha: ");
        Serial.print(_password);
        for (int i = strlen(_password); i < 25; i++) Serial.print(" ");
        Serial.println("║");
        Serial.println("║ 3. Abra o app LiteWing             ║");
        Serial.println("║ 4. Clique no botão LINK (conectar) ║");
        Serial.println("║ 5. O drone deve conectar auto!     ║");
        Serial.println("╚════════════════════════════════════╝\n");
        Serial.println("⏳ Aguardando conexão do LiteWing app...\n");
    }
    
    return true;
}

void WiFiComm::update() {
    // Processa pacotes UDP
    int packetSize = _udp.parsePacket();
    
    if (packetSize) {
        int len = _udp.read(_packetBuffer, sizeof(_packetBuffer));
        
        if (len > 0) {
            IPAddress remoteIP = _udp.remoteIP();
            int remotePort = _udp.remotePort();
            
            processPacket(_packetBuffer, len, remoteIP, remotePort);
        }
    }
    
    // Verifica status da conexão periodicamente
    if (millis() - _lastStatusCheck > 3000) {
        checkConnectionStatus();
    }
}

void WiFiComm::processPacket(uint8_t* buffer, int length, IPAddress remoteIP, int remotePort) {
    // Atualiza informações do cliente
    updateClientConnection(remoteIP, remotePort);
    
    _packetCount++;
    _lastPacketTime = millis();
    
    if (_verboseEnabled) {
        Serial.print("📦 Pacote #");
        Serial.print(_packetCount);
        Serial.print(" (");
        Serial.print(length);
        Serial.println(" bytes)");
        
        // Imprime dados em hexadecimal
        Serial.print("   HEX: ");
        for (int i = 0; i < length && i < 32; i++) {
            Serial.printf("%02X ", buffer[i]);
        }
        if (length > 32) Serial.print("...");
        Serial.println();
    }
    
    // Interpreta pacote CRTP
    if (length >= 1) {
        uint8_t header = buffer[0];
        uint8_t port = (header >> 4) & 0x0F;
        uint8_t channel = header & 0x0F;
        
        if (_verboseEnabled) {
            Serial.print("   CRTP → Port: ");
            Serial.print(port);
            Serial.print(", Channel: ");
            Serial.println(channel);
        }
        
        // Port 0x00 = Console
        if (port == 0x00) {
            if (_verboseEnabled) {
                Serial.print("   💬 Console: ");
                for (int i = 1; i < length; i++) {
                    Serial.print((char)buffer[i]);
                }
                Serial.println();
            }
        }
        // Port 0x02 = Parameters
        else if (port == 0x02) {
            if (_verboseEnabled) Serial.println("   ⚙️  Requisição de Parâmetros");
            sendParameterResponse(remoteIP, remotePort, buffer, length);
        }
        // Port 0x03 = Commander (controle de voo)
        else if (port == 0x03 && length >= 15) {
            processCommanderPacket(buffer, length);
        }
        // Port 0x05 = Logging
        else if (port == 0x05) {
            if (_verboseEnabled) Serial.println("   📊 Requisição de Log");
            sendLoggingResponse(remoteIP, remotePort, buffer, length);
        }
        // Port 0x06 = Memory
        else if (port == 0x06) {
            if (_verboseEnabled) Serial.println("   💾 Requisição de Memória");
            sendMemoryResponse(remoteIP, remotePort, buffer, length);
        }
        // Port 0x07 = Link Control (Ping)
        else if (port == 0x07) {
            if (_verboseEnabled) Serial.println("   🏓 PING → Enviando PONG");
            sendPongResponse(remoteIP, remotePort);
        }
        // Port 0x0F = Platform
        else if (port == 0x0F) {
            if (_verboseEnabled) Serial.println("   🖥️  Platform Info");
            sendPlatformResponse(remoteIP, remotePort, buffer, length);
        }
        else {
            if (_verboseEnabled) {
                Serial.print("   ❓ Port ");
                Serial.print(port);
                Serial.println(" (desconhecido)");
            }
        }
    }
    
    if (_verboseEnabled) {
        Serial.println();
    }
}

void WiFiComm::processCommanderPacket(uint8_t* buffer, int length) {
    memcpy(&_lastCommand, &buffer[1], sizeof(CommanderPacket));
    
    // Inverte o sinal do pitch (corrige padronização frente/trás do controle para o código)
    _lastCommand.pitch = -_lastCommand.pitch;

    _newCommandAvailable = true;
    
    if (_debugEnabled || _verboseEnabled) {
        Serial.println("   🎮 CONTROLES:");
        
        // Roll
        Serial.print("      Roll:   ");
        Serial.printf("%+.3f ", _lastCommand.roll);
        if (abs(_lastCommand.roll) < 0.05) Serial.println("⬌ CENTRO");
        else if (_lastCommand.roll > 0) Serial.println("➡️  DIREITA");
        else Serial.println("⬅️  ESQUERDA");
        
        // Pitch
        Serial.print("      Pitch:  ");
        Serial.printf("%+.3f ", _lastCommand.pitch);
        if (abs(_lastCommand.pitch) < 0.05) Serial.println("⬌ CENTRO");
        else if (_lastCommand.pitch > 0) Serial.println("⬆️  FRENTE");
        else Serial.println("⬇️  TRÁS");
        
        // Yaw
        Serial.print("      Yaw:    ");
        Serial.printf("%+.3f ", _lastCommand.yaw);
        if (abs(_lastCommand.yaw) < 0.05) Serial.println("⬌ SEM ROTAÇÃO");
        else if (_lastCommand.yaw > 0) Serial.println("⟳ HORÁRIO");
        else Serial.println("⟲ ANTI-HORÁRIO");
        
        // Thrust
        Serial.print("      Thrust: ");
        Serial.print(_lastCommand.thrust);
        float percent = (_lastCommand.thrust / 60000.0f) * 100;
        Serial.printf(" (%.1f%%) ", percent);
        if (_lastCommand.thrust == 0) Serial.println("⚫ DESLIGADO");
        else if (percent < 30) Serial.println("🟢 BAIXO");
        else if (percent < 60) Serial.println("🟡 MÉDIO");
        else Serial.println("🔴 ALTO");
    }
    
    // Chama callback se configurado
    if (_onCommandCallback != nullptr) {
        _onCommandCallback(_lastCommand);
    }
}

void WiFiComm::updateClientConnection(IPAddress remoteIP, int remotePort) {
    if (!_clientConnected || _clientIP != remoteIP || _clientPort != remotePort) {
        _clientIP = remoteIP;
        _clientPort = remotePort;
        _clientConnected = true;
        
        if (_debugEnabled) {
            Serial.println("\n╔════════════════════════════════════╗");
            Serial.println("║    🎉 CLIENTE CONECTADO! 🎉        ║");
            Serial.println("╚════════════════════════════════════╝");
            Serial.print("📍 IP do Cliente: ");
            Serial.print(remoteIP);
            Serial.print(":");
            Serial.println(remotePort);
            Serial.println();
        }
        
        // Chama callback se configurado
        if (_onClientConnectedCallback != nullptr) {
            _onClientConnectedCallback();
        }
    }
}

void WiFiComm::checkConnectionStatus() {
    _lastStatusCheck = millis();
    
    int numClients = WiFi.softAPgetStationNum();
    
    if (numClients != _lastNumClients) {
        _lastNumClients = numClients;
        
        if (numClients > 0) {
            if (_debugEnabled) {
                Serial.println("📱 Cliente WiFi conectado!");
                if (!_clientConnected) {
                    Serial.println("   Aguardando pacotes UDP do app...\n");
                }
            }
        } else {
            if (_clientConnected && _debugEnabled) {
                Serial.println("\n❌ Cliente WiFi desconectado!\n");
            }
            
            if (_clientConnected && _onClientDisconnectedCallback != nullptr) {
                _onClientDisconnectedCallback();
            }
            
            _clientConnected = false;
        }
    }
    
    // Alerta se não receber pacotes
    if (_clientConnected && (millis() - _lastPacketTime > 3000) && _debugEnabled) {
        Serial.println("⚠️  Sem pacotes UDP há mais de 3 segundos");
    }
}

// ===== RESPOSTAS CRTP =====

void WiFiComm::sendPongResponse(IPAddress ip, int port) {
    uint8_t response[] = {0xF0, 0x01}; // Echo response
    _udp.beginPacket(ip, port);
    _udp.write(response, sizeof(response));
    _udp.endPacket();
}

void WiFiComm::sendParameterResponse(IPAddress ip, int port, uint8_t* request, int len) {
    if (len >= 2) {
        uint8_t cmd = request[1];
        
        // TOC Get Info (0x00)
        if (cmd == 0x00) {
            uint8_t response[] = {0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
        // TOC Get Item (0x01)
        else if (cmd == 0x01) {
            uint8_t response[] = {0x20, 0x01}; 
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
    }
}

void WiFiComm::sendLoggingResponse(IPAddress ip, int port, uint8_t* request, int len) {
    if (len >= 2) {
        uint8_t cmd = request[1];
        
        // TOC Get Info (0x00)
        if (cmd == 0x00) {
            uint8_t response[] = {0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
        // TOC Get Item (0x01)
        else if (cmd == 0x01) {
            uint8_t response[] = {0x50, 0x01};
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
    }
}

void WiFiComm::sendMemoryResponse(IPAddress ip, int port, uint8_t* request, int len) {
    if (len >= 2) {
        uint8_t cmd = request[1];
        
        if (cmd == 0x10) { // Get number of memories
            uint8_t response[] = {0x60, 0x10, 0x01, 0x00};
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
        else if (cmd == 0x11) { // Get memory info
            uint8_t response[] = {0x60, 0x11, 0x00, 0x01, 0x00, 0x00, 0x10, 0x00};
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
    }
}

void WiFiComm::sendPlatformResponse(IPAddress ip, int port, uint8_t* request, int len) {
    if (len >= 2) {
        uint8_t cmd = request[1];
        
        if (cmd == 0x00) { // Platform info
            uint8_t response[] = {0xF0, 0x00, 0x00, 0x01}; // Crazyflie 2.0
            _udp.beginPacket(ip, port);
            _udp.write(response, sizeof(response));
            _udp.endPacket();
        }
    }
}

// ===== GETTERS E SETTERS =====

CommanderPacket WiFiComm::getLastCommand() {
    _newCommandAvailable = false;
    return _lastCommand;
}

bool WiFiComm::hasNewCommand() {
    return _newCommandAvailable;
}

bool WiFiComm::isClientConnected() {
    return _clientConnected;
}

IPAddress WiFiComm::getClientIP() {
    return _clientIP;
}

int WiFiComm::getClientPort() {
    return _clientPort;
}

unsigned long WiFiComm::getPacketCount() {
    return _packetCount;
}

unsigned long WiFiComm::getLastPacketTime() {
    return _lastPacketTime;
}

void WiFiComm::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void WiFiComm::enableVerbose(bool enable) {
    _verboseEnabled = enable;
}

// ===== CALLBACKS =====

void WiFiComm::onCommandReceived(void (*callback)(CommanderPacket)) {
    _onCommandCallback = callback;
}

void WiFiComm::onClientConnected(void (*callback)()) {
    _onClientConnectedCallback = callback;
}

void WiFiComm::onClientDisconnected(void (*callback)()) {
    _onClientDisconnectedCallback = callback;
}
