# Integração WiFi/UDP no Projeto SDRE_VECTORIZED

## 📁 Estrutura Criada

```
lib/
└── WiFiComm/
    ├── WiFiComm.h          # Header da biblioteca
    ├── WiFiComm.cpp        # Implementação
    ├── README.md           # Documentação completa
    └── examples/
        └── simple_test.cpp # Exemplo de teste isolado
```

## 🎯 O que foi implementado

### 1. Biblioteca WiFiComm (lib/WiFiComm/)

**Características principais:**
- ✅ Protocolo CRTP (Crazyflie Real-Time Protocol) completo
- ✅ Compatível com app LiteWing
- ✅ Sistema de callbacks para eventos
- ✅ Modos debug e verbose
- ✅ Detecção automática de conexão/desconexão
- ✅ Respostas automáticas a pings e requisições

**Ports CRTP implementados:**
- `0x00` - Console
- `0x02` - Parameters
- `0x03` - **Commander (controle de voo)** ⭐
- `0x05` - Logging
- `0x06` - Memory
- `0x07` - Link Control (Ping/Pong)
- `0x0F` - Platform Info

### 2. Integração com main.cpp

**Modificações realizadas:**

1. **Inclusão da biblioteca:**
   ```cpp
   #include "WiFiComm.h"
   ```

2. **Instância global:**
   ```cpp
   WiFiComm wifiComm("ESP-DRONE", "12345678", 2390);
   ```

3. **Variáveis de controle remoto:**
   ```cpp
   bool remote_control_enabled = false;
   CommanderPacket remote_command;
   ```

4. **Callbacks implementados:**
   - `onRemoteCommandReceived()` - Processa comandos
   - `onClientConnected()` - Notifica conexão
   - `onClientDisconnected()` - Trata desconexão e zera comandos

5. **Inicialização no setup():**
   ```cpp
   wifiComm.enableDebug(true);
   wifiComm.begin();
   wifiComm.onCommandReceived(onRemoteCommandReceived);
   wifiComm.onClientConnected(onClientConnected);
   wifiComm.onClientDisconnected(onClientDisconnected);
   ```

6. **Atualização no loop():**
   ```cpp
   wifiComm.update(); // Chamado a cada iteração
   ```

7. **Lógica dual de controle:**
   - **Modo Remoto (WiFi conectado):** Usa comandos do joystick
   - **Modo Autônomo (sem WiFi):** Usa lógica original do código

## 🚀 Como usar

### Passo 1: Upload do código
Compile e faça upload do código para o ESP32.

### Passo 2: Conectar ao WiFi
No smartphone:
1. Vá em **Configurações > WiFi**
2. Conecte na rede **ESP-DRONE**
3. Senha: **12345678**

### Passo 3: Abrir app LiteWing
1. Abra o aplicativo LiteWing
2. Clique no botão **LINK** (ícone de conexão)
3. Aguarde conexão automática
4. Use os joysticks para controlar

### Passo 4: Monitorar no Serial
O Serial Monitor mostrará:
```
✅ WiFi AP iniciado!
   IP: 192.168.43.42
   Porta UDP: 2390

⏳ Aguardando conexão do LiteWing app...

📱 Cliente WiFi conectado!
   Aguardando pacotes UDP do app...

╔════════════════════════════════════╗
║    🎉 CLIENTE CONECTADO! 🎉        ║
╚════════════════════════════════════╝
📍 IP do Cliente: 192.168.43.xxx:xxxxx

   🎮 CONTROLES:
      Roll:   +0.523 ➡️  DIREITA
      Pitch:  -0.234 ⬇️  TRÁS
      Yaw:    +0.000 ⬌ SEM ROTAÇÃO
      Thrust: 32768 (50.0%) 🟡 MÉDIO
```

## 🔧 Configurações

### Alterar nome do WiFi
```cpp
WiFiComm wifiComm("MEU-DRONE", "minhasenha", 2390);
```

### Alterar IP
```cpp
void setup() {
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    
    wifiComm.setIP(local_IP, gateway, subnet);
    wifiComm.begin();
}
```

### Configurar debug
```cpp
wifiComm.enableDebug(true);   // Mensagens importantes
wifiComm.enableVerbose(true); // TODOS os pacotes (muito verbose!)
```

## 🎮 Conversão de Comandos

O código converte comandos do joystick em setpoints:

```cpp
if (remote_control_enabled && wifiComm.isClientConnected()) {
    phi_desired = remote_command.roll;      // -1.0 a 1.0
    theta_desired = remote_command.pitch;   // -1.0 a 1.0
    yaw_desired = remote_command.yaw;       // -1.0 a 1.0
    
    // Normaliza thrust (0-65535 para 0-100% da força)
    thrust = (remote_command.thrust / 65535.0f) * m * gravity * 2.0f;
}
```

## ⚠️ Segurança

1. **Desconexão automática:** Se o WiFi cair, os comandos são zerados
2. **Timeout de pacotes:** Alerta se não receber dados por 3 segundos
3. **Modo padrão:** Sem conexão = modo autônomo
4. **Flag enable_motors:** Deve ser `true` para motores funcionarem

## 🧪 Teste Isolado

Para testar apenas a conexão WiFi sem o resto do sistema:

1. Abra `lib/WiFiComm/examples/simple_test.cpp`
2. Copie o conteúdo para `src/main.cpp` (temporariamente)
3. Compile e faça upload
4. Conecte via app e observe comandos no Serial Monitor

## 📊 Fluxo de Dados

```
┌─────────────┐           ┌─────────────┐           ┌─────────────┐
│  LiteWing   │  WiFi/UDP │   ESP32     │  Process  │  Controle   │
│     App     │──────────▶│  WiFiComm   │──────────▶│   SDRE      │
└─────────────┘           └─────────────┘           └─────────────┘
     │                           │                         │
     │  Pacotes CRTP             │  Callbacks              │
     │  (roll,pitch,yaw,thrust)  │  (CommanderPacket)     │
     │                           │                         │
     └──────────Ping/Pong────────┘                         │
              (mantém conexão)                             │
                                                           │
                                                    ┌──────▼──────┐
                                                    │   Motores   │
                                                    └─────────────┘
```

## 🔍 Troubleshooting

**WiFi não aparece:**
- Verifique se o ESP32 suporta WiFi
- Reinicie o ESP32
- Verifique serial: "✅ WiFi AP iniciado!"

**App não conecta:**
- Confirme senha correta: `12345678`
- Verifique IP: deve ser `192.168.43.42`
- Tente desconectar e reconectar ao WiFi

**Comandos não chegam:**
- Ative verbose: `wifiComm.enableVerbose(true)`
- Verifique porta UDP: deve ser `2390`
- Procure mensagens "🎮 CONTROLES" no Serial

**Motores não respondem:**
- Verifique `enable_motors = true`
- Verifique `motors.isArmed()`
- Teste modo autônomo primeiro

## 📝 Próximos Passos

1. ✅ Sistema de comunicação WiFi implementado
2. ⏳ Testar comunicação com app LiteWing
3. ⏳ Calibrar conversão de comandos para setpoints
4. ⏳ Implementar failsafe avançado
5. ⏳ Adicionar telemetria de volta (enviar dados para app)

## 📚 Referências

- [Protocolo CRTP - Crazyflie](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/)
- [LiteWing App](https://play.google.com/store/apps/details?id=com.litewing)
- Código base: `testa_conexao_wifi.ino`
