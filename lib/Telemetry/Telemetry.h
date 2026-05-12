#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <LittleFS.h>

/**
 * Buffer circular em RAM para telemetria de voo.
 * Captura amostras a cada chamada de log() sem alocacao dinamica.
 * Dump em CSV via Serial sob demanda (apos pouso, drone parado).
 *
 * Persistencia em flash (LittleFS): saveToFile() chamado ao desarmar.
 * loadFromFile() no setup() restaura buffer apos reboot/reset do ESP32
 * (ESP32 reseta quando Serial Monitor abre - sem isso o buffer some).
 *
 * Tamanho: CAPACITY * sizeof(Sample) bytes.
 * Default: 1000 amostras x 44 bytes = ~44 KB em RAM, mesmo em flash.
 *
 * Custo por log(): apenas escritas em RAM (~1 us). Zero printf, zero I/O.
 * Custo de saveToFile(): ~100-300 ms - so chamado apos pouso (motor parado).
 */
class Telemetry {
public:
    struct Sample {
        uint32_t t_ms;
        float roll, pitch, yaw;     // rad
        float p, q, r;              // rad/s (taxas no corpo)
        float u0, u1, u2;           // torques SDRE [N·m]: roll, pitch, yaw
        float w1_sq, w2_sq, w3_sq, w4_sq;  // rad^2/s^2
    };

    static constexpr size_t CAPACITY = 1000;

    Telemetry() : head(0), count(0) {}

    inline void log(uint32_t t_ms,
                    float roll, float pitch, float yaw,
                    float p, float q, float r,
                    float u0, float u1, float u2,
                    float w1_sq, float w2_sq, float w3_sq, float w4_sq) {
        Sample &s = buf[head];
        s.t_ms = t_ms;
        s.roll = roll;  s.pitch = pitch;  s.yaw = yaw;
        s.p = p;        s.q = q;          s.r = r;
        s.u0 = u0;      s.u1 = u1;        s.u2 = u2;
        s.w1_sq = w1_sq; s.w2_sq = w2_sq; s.w3_sq = w3_sq; s.w4_sq = w4_sq;
        head = (head + 1) % CAPACITY;
        if (count < CAPACITY) count++;
    }

    void reset() {
        head = 0;
        count = 0;
    }

    size_t size() const { return count; }

    /**
     * Salva buffer em LittleFS (binario, raw struct dump).
     * Chamar APENAS com motores parados (operacao lenta, ~100-300 ms).
     * Retorna true em sucesso.
     */
    bool saveToFile(const char* path = "/telem.bin") {
        File f = LittleFS.open(path, "w");
        if (!f) return false;
        uint32_t magic = 0x54454C4E; // "TELN" - v2 struct (added u0,u1,u2)
        f.write((uint8_t*)&magic, sizeof(magic));
        uint32_t cap = CAPACITY;
        f.write((uint8_t*)&cap, sizeof(cap));
        uint32_t h = (uint32_t)head;
        uint32_t c = (uint32_t)count;
        f.write((uint8_t*)&h, sizeof(h));
        f.write((uint8_t*)&c, sizeof(c));
        f.write((uint8_t*)buf, sizeof(Sample) * CAPACITY);
        f.close();
        return true;
    }

    /**
     * Carrega buffer de LittleFS. Retorna true se arquivo existe e e valido.
     * Chamar no setup() apos LittleFS.begin().
     */
    bool loadFromFile(const char* path = "/telem.bin") {
        if (!LittleFS.exists(path)) return false;
        File f = LittleFS.open(path, "r");
        if (!f) return false;
        uint32_t magic = 0, cap = 0, h = 0, c = 0;
        f.read((uint8_t*)&magic, sizeof(magic));
        f.read((uint8_t*)&cap, sizeof(cap));
        if (magic != 0x54454C4E || cap != CAPACITY) {
            f.close();
            return false;
        }
        f.read((uint8_t*)&h, sizeof(h));
        f.read((uint8_t*)&c, sizeof(c));
        f.read((uint8_t*)buf, sizeof(Sample) * CAPACITY);
        f.close();
        head = (size_t)h;
        count = (size_t)c;
        return true;
    }

    void dumpCSV(Stream &out) {
        out.println();
        out.println("=== TELEMETRY DUMP START ===");
        out.print("Samples: "); out.println((unsigned long)count);
        out.println("t_ms,roll_deg,pitch_deg,yaw_deg,p_dps,q_dps,r_dps,u_roll,u_pitch,u_yaw,w1_sq,w2_sq,w3_sq,w4_sq");

        const float RAD2DEG = 57.29578f;
        size_t start = (count < CAPACITY) ? 0 : head;
        for (size_t i = 0; i < count; i++) {
            const Sample &s = buf[(start + i) % CAPACITY];
            out.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.1f,%.1f,%.1f,%.1f\n",
                       (unsigned long)s.t_ms,
                       s.roll * RAD2DEG, s.pitch * RAD2DEG, s.yaw * RAD2DEG,
                       s.p * RAD2DEG, s.q * RAD2DEG, s.r * RAD2DEG,
                       s.u0, s.u1, s.u2,
                       s.w1_sq, s.w2_sq, s.w3_sq, s.w4_sq);
        }
        out.println("=== TELEMETRY DUMP END ===");
    }

private:
    Sample buf[CAPACITY];
    size_t head;
    size_t count;
};

#endif
