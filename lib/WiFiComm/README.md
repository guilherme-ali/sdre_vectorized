# WiFiComm - Biblioteca de Comunicação WiFi/UDP

Esta biblioteca implementa comunicação WiFi/UDP compatível com o protocolo **Crazyflie** usado pelo aplicativo **LiteWing**.

## 📋 Características

- ✅ Compatível com protocolo CRTP (Crazyflie Real-Time Protocol)
- ✅ Suporta comandos de controle (roll, pitch, yaw, thrust)
- ✅ Respostas automáticas a pings, parâmetros, logging, etc.
- ✅ Sistema de callbacks para eventos
- ✅ Modos de debug configuráveis
- ✅ Detecção automática de conexão/desconexão

## 🚀 Uso Básico

```cpp
#include <WiFiComm.h>

// Criar instância
WiFiComm wifiComm("ESP-DRONE", "12345678", 2390);

// Callback para comandos recebidos
void onCommandReceived(CommanderPacket cmd) {
    Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Thrust: %d\n",
                  cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust);
}

void setup() {
    Serial.begin(115200);
    
    // Configurar debug
    wifiComm.enableDebug(true);
    
    // Inicializar WiFi
    if (wifiComm.begin()) {
        Serial.println("WiFi iniciado!");
        
        // Configurar callback
        wifiComm.onCommandReceived(onCommandReceived);
    }
}

void loop() {
    // Atualizar comunicação (OBRIGATÓRIO chamar em loop)
    wifiComm.update();
    
    // Verificar se há novo comando
    if (wifiComm.hasNewCommand()) {
        CommanderPacket cmd = wifiComm.getLastCommand();
        // Processar comando...
    }
}
```

## 📡 Configuração de Rede

Por padrão, a biblioteca usa:
- **SSID**: ESP-DRONE
- **Senha**: 12345678
- **IP**: 192.168.43.42 (padrão LiteWing)
- **Porta UDP**: 2390 (padrão Crazyflie)

### Personalizar IP

```cpp
WiFiComm wifiComm;

void setup() {
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    wifiComm.setIP(local_IP, gateway, subnet);
    wifiComm.begin();
}
```

## 📦 Estrutura do Comando

```cpp
struct CommanderPacket {
    float roll;      // -1.0 a 1.0
    float pitch;     // -1.0 a 1.0
    float yaw;       // -1.0 a 1.0
    uint16_t thrust; // 0 a 60000
};
```

## 🔔 Callbacks Disponíveis

```cpp
// Comando recebido
wifiComm.onCommandReceived([](CommanderPacket cmd) {
    // Processar comando
});

// Cliente conectado
wifiComm.onClientConnected([]() {
    Serial.println("Cliente conectado!");
});

// Cliente desconectado
wifiComm.onClientDisconnected([]() {
    Serial.println("Cliente desconectado!");
});
```

## 🔍 Métodos Úteis

```cpp
// Verificar conexão
bool connected = wifiComm.isClientConnected();

// Obter último comando
CommanderPacket cmd = wifiComm.getLastCommand();

// Verificar se há novo comando
bool hasNew = wifiComm.hasNewCommand();

// Obter informações do cliente
IPAddress ip = wifiComm.getClientIP();
int port = wifiComm.getClientPort();

// Estatísticas
unsigned long count = wifiComm.getPacketCount();
unsigned long lastTime = wifiComm.getLastPacketTime();
```

## 🐛 Debug

```cpp
// Ativar mensagens de debug (conexões, comandos)
wifiComm.enableDebug(true);

// Ativar modo verbose (todos os pacotes)
wifiComm.enableVerbose(true);
```

## 📱 Como Conectar o App LiteWing

1. Conecte o smartphone ao WiFi **ESP-DRONE** (senha: **12345678**)
2. Abra o aplicativo LiteWing
3. Clique no botão **LINK** (ícone de conexão)
4. O drone deve conectar automaticamente
5. Use os joysticks para controlar o drone

## 🔐 Protocolo CRTP

A biblioteca implementa os seguintes ports CRTP:

| Port | Função | Descrição |
|------|--------|-----------|
| 0x00 | Console | Mensagens de console |
| 0x02 | Parameters | Parâmetros do sistema |
| 0x03 | Commander | **Comandos de controle** |
| 0x05 | Logging | Sistema de logging |
| 0x06 | Memory | Acesso à memória |
| 0x07 | Link Control | Ping/pong para manter conexão |
| 0x0F | Platform | Informações da plataforma |

## ⚠️ Notas Importantes

- **SEMPRE** chame `wifiComm.update()` no loop principal
- A biblioteca responde automaticamente a pings para manter a conexão
- Em caso de desconexão, os comandos são zerados por segurança
- O modo verbose pode gerar muitos dados no Serial Monitor

## 🔗 Integração com Controle de Motores

Veja exemplo em `main.cpp` de como integrar com o sistema de controle do drone.

## 📄 Licença

Código baseado no protocolo Crazyflie (open source).
