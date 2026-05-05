# 🚀 Teste Rápido - Conexão WiFi LiteWing

## ⚡ Início Rápido (5 minutos)

### 1️⃣ Upload do Código
```bash
# No terminal do PlatformIO
pio run -t upload
```

### 2️⃣ Abrir Monitor Serial
```bash
pio device monitor
```

Você deve ver:
```
╔════════════════════════════════════╗
║  ESP32 DRONE - LiteWing Compatible ║
╚════════════════════════════════════╝

📡 Configurando WiFi Access Point...
   SSID: ESP-DRONE
   Password: 12345678

✅ WiFi AP iniciado!
   IP: 192.168.43.42
   Porta UDP: 2390

✅ Servidor UDP iniciado!
```

### 3️⃣ Conectar Smartphone
1. **Configurações** do smartphone
2. **WiFi** → Procurar redes
3. Conectar em **ESP-DRONE**
4. Senha: **12345678**

### 4️⃣ Abrir App LiteWing
1. Abrir app **LiteWing**
2. Clicar botão **LINK** (canto superior)
3. Aguardar mensagem "Connected"

### 5️⃣ Verificar Conexão
No Serial Monitor você verá:
```
📱 Cliente WiFi conectado!
   Aguardando pacotes UDP do app...

╔════════════════════════════════════╗
║    🎉 CLIENTE CONECTADO! 🎉        ║
╚════════════════════════════════════╝
📍 IP do Cliente: 192.168.43.xxx:xxxxx
```

### 6️⃣ Testar Joysticks
Mova os joysticks no app. Você verá:
```
   🎮 CONTROLES:
      Roll:   +0.523 ➡️  DIREITA
      Pitch:  -0.234 ⬇️  TRÁS
      Yaw:    +0.000 ⬌ SEM ROTAÇÃO
      Thrust: 32768 (50.0%) 🟡 MÉDIO
```

## ✅ Checklist de Sucesso

- [ ] ESP32 criou rede WiFi "ESP-DRONE"
- [ ] Smartphone conectou na rede
- [ ] App LiteWing mostrou "Connected"
- [ ] Serial Monitor mostra "🎉 CLIENTE CONECTADO!"
- [ ] Movendo joysticks mostra valores no Serial
- [ ] Valores de roll, pitch, yaw mudam ao mover sticks

## 🐛 Problemas Comuns

### ❌ "WiFi ESP-DRONE não aparece"
**Solução:**
1. Reinicie o ESP32
2. Verifique se é ESP32 (não ESP8266)
3. Aguarde 10 segundos após boot

### ❌ "Conecta WiFi mas app não conecta"
**Solução:**
1. Verifique IP no Serial: deve ser `192.168.43.42`
2. Force fechar e reabrir o app
3. No app, vá em Settings e verifique porta UDP: `2390`

### ❌ "Conecta mas não recebe comandos"
**Solução:**
1. No código, ative verbose:
   ```cpp
   wifiComm.enableVerbose(true);
   ```
2. Recompile e observe pacotes no Serial
3. Verifique se LED do app pisca ao mover joysticks

### ❌ "Comandos chegam mas motores não respondem"
**Solução:**
1. Verifique `enable_motors = true` no código
2. Verifique se motores estão armados: `motors.isArmed()`
3. Teste primeiro em modo autônomo

## 🎮 Valores Esperados

| Joystick | Eixo | Valor Neutro | Valor Máximo + | Valor Máximo - |
|----------|------|--------------|----------------|----------------|
| Direito  | Roll | 0.0          | +1.0 →        | -1.0 ←        |
| Direito  | Pitch| 0.0          | +1.0 ⬆        | -1.0 ⬇        |
| Esquerdo | Yaw  | 0.0          | +1.0 ⟳        | -1.0 ⟲        |
| Esquerdo | Thrust| 0           | 60000 (100%)   | 0 (0%)         |

## 📊 Teste de Performance

Execute por 1 minuto e observe:

```
Tempo_execucao: 2500 μs
Tempo_Maximo: 3200 μs
Tempo_Medio: 2600 μs

🎮 CONTROLES:
   [comandos atualizando...]

--------------------------------------------------
```

**Esperado:**
- Tempo médio < 10ms (10000 μs)
- Comandos atualizando a ~50Hz
- Sem mensagens de "Sem pacotes"

## 🔬 Teste Avançado: Latência

1. Mova joystick rapidamente
2. Observe delay entre movimento e aparecer no Serial
3. Latência esperada: **< 100ms**

## 📸 Prints de Sucesso

### Serial Monitor Correto:
```
✅ WiFi AP iniciado!
✅ Servidor UDP iniciado!
📱 Cliente WiFi conectado!
🎉 CLIENTE CONECTADO!
🎮 CONTROLES:
   Roll:   +0.523
   Pitch:  -0.234
   Yaw:    +0.000
   Thrust: 32768
```

### App LiteWing Correto:
- Status: **Connected** (verde)
- LED piscando ao mover joysticks
- Valores numéricos mudando na tela

## 🎯 Próximo Passo

Após confirmar que tudo funciona:

1. **Calibrar ganhos** de conversão:
   ```cpp
   // Ajustar multiplicadores conforme necessário
   phi_desired = remote_command.roll * GAIN_ROLL;
   ```

2. **Testar com motores desarmados:**
   ```cpp
   enable_motors = false; // Segurança
   ```

3. **Observar setpoints** no Serial:
   ```cpp
   Serial.printf("Setpoint: φ=%.2f θ=%.2f ψ=%.2f\n",
                 phi_desired, theta_desired, yaw_desired);
   ```

4. **Gradualmente ativar motores** quando tudo estiver ok!

## 📞 Suporte

Se encontrar problemas:
1. Ative modo verbose no código
2. Copie output do Serial Monitor
3. Verifique documentação: `../lib/WiFiComm/README.md`
4. Veja integração completa: `WIFI_INTEGRATION.md`

---

**✨ Boa sorte com os testes!** 🚁
