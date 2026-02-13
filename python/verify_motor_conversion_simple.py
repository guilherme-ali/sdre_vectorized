#!/usr/bin/env python3
"""
Script para verificar a correção da conversão omega² → throttle (sem dependências)
"""

import math

# Parâmetros do sistema (de src/main.cpp)
MAX_RPM = 51000.0  # RPM máximo
MAX_OMEGA = (MAX_RPM * 2.0 * math.pi) / 60.0  # rad/s
MAX_OMEGA_SQ = MAX_OMEGA ** 2  # rad²/s²
MOTOR_B_COEFF = 2.798e-7  # N/(rad/s)²

print("=" * 70)
print("VERIFICAÇÃO DA CONVERSÃO OMEGA² → THROTTLE")
print("=" * 70)
print(f"\nParâmetros do Sistema:")
print(f"  MAX_RPM       = {MAX_RPM:.0f} RPM")
print(f"  MAX_OMEGA     = {MAX_OMEGA:.2f} rad/s")
print(f"  MAX_OMEGA_SQ  = {MAX_OMEGA_SQ:.0f} rad²/s²")
print(f"  MOTOR_B_COEFF = {MOTOR_B_COEFF:.4e} N/(rad/s)²")
print("=" * 70)

def omega_sq_to_throttle_wrong(omega_sq, max_omega_sq=MAX_OMEGA_SQ):
    """Implementação INCORRETA (linear) - código original"""
    return (omega_sq / max_omega_sq) * 100.0

def omega_sq_to_throttle_correct(omega_sq, max_omega_sq=MAX_OMEGA_SQ):
    """Implementação CORRETA (com sqrt) - código corrigido"""
    return math.sqrt(omega_sq / max_omega_sq) * 100.0

# Teste 1: Comparação Numérica
print("\n📊 TESTE 1: Comparação Numérica")
print("-" * 70)
print(f"{'ω (% MAX)':>12} {'ω² (% MAX)':>12} {'Throttle ERRADO':>18} {'Throttle CORRETO':>18}")
print("-" * 70)

test_percentages = [0, 10, 25, 50, 75, 90, 100]
for omega_pct in test_percentages:
    omega = (omega_pct / 100.0) * MAX_OMEGA
    omega_sq = omega ** 2
    omega_sq_pct = (omega_sq / MAX_OMEGA_SQ) * 100.0

    throttle_wrong = omega_sq_to_throttle_wrong(omega_sq)
    throttle_correct = omega_sq_to_throttle_correct(omega_sq)

    print(f"{omega_pct:>12.0f}% {omega_sq_pct:>11.2f}% {throttle_wrong:>17.2f}% {throttle_correct:>17.2f}%")

print("-" * 70)

# Teste 2: Exemplo com Thrust Real
print("\n🧮 TESTE 2: Exemplo com Thrust = 0.3 N (hover típico)")
print("-" * 70)

thrust = 0.3  # N - empuxo total para hover
omega_sq_required = thrust / (4.0 * MOTOR_B_COEFF)  # ω² necessário por motor
omega_required = math.sqrt(omega_sq_required)
rpm_required = (omega_required * 60.0) / (2.0 * math.pi)

throttle_wrong = omega_sq_to_throttle_wrong(omega_sq_required)
throttle_correct = omega_sq_to_throttle_correct(omega_sq_required)

print(f"\nPara Thrust = {thrust:.3f} N:")
print(f"  ω² requerido  = {omega_sq_required:.0f} rad²/s²")
print(f"  ω requerido   = {omega_required:.2f} rad/s")
print(f"  RPM requerido = {rpm_required:.0f} RPM")
print(f"  % de MAX_OMEGA = {(omega_required/MAX_OMEGA)*100:.2f}%")
print(f"\nConversão para Throttle:")
print(f"  ❌ ERRADO (linear):  {throttle_wrong:.2f}%")
print(f"  ✅ CORRETO (sqrt):   {throttle_correct:.2f}%")
print(f"\nDiferença: {abs(throttle_correct - throttle_wrong):.2f} pontos percentuais")

# Verificação reversa
omega_achieved_wrong = (throttle_wrong / 100.0) * MAX_OMEGA
omega_sq_achieved_wrong = omega_achieved_wrong ** 2
thrust_achieved_wrong = 4.0 * MOTOR_B_COEFF * omega_sq_achieved_wrong

omega_achieved_correct = (throttle_correct / 100.0) * MAX_OMEGA
omega_sq_achieved_correct = omega_achieved_correct ** 2
thrust_achieved_correct = 4.0 * MOTOR_B_COEFF * omega_sq_achieved_correct

print(f"\nVerificação Reversa (Thrust Resultante):")
print(f"  ❌ Com throttle ERRADO  ({throttle_wrong:.2f}%): T = {thrust_achieved_wrong:.4f} N")
print(f"  ✅ Com throttle CORRETO ({throttle_correct:.2f}%): T = {thrust_achieved_correct:.4f} N")
print(f"  🎯 Thrust desejado: {thrust:.3f} N")
print(f"\nErro com implementação INCORRETA: {abs(thrust_achieved_wrong - thrust):.4f} N ({abs(thrust_achieved_wrong - thrust)/thrust*100:.1f}%)")

print("-" * 70)

# Teste 3: Impacto no Controle
print("\n⚠️  TESTE 3: Impacto no Controle do Drone")
print("-" * 70)

# Simula diferentes comandos de thrust
thrust_commands = [0.1, 0.2, 0.3, 0.4, 0.5]  # N
print(f"\n{'Thrust Cmd (N)':>15} {'Throttle ERRADO':>18} {'Throttle CORRETO':>18} {'Diferença':>12}")
print("-" * 70)

for thrust_cmd in thrust_commands:
    omega_sq_cmd = thrust_cmd / (4.0 * MOTOR_B_COEFF)
    throttle_wrong = omega_sq_to_throttle_wrong(omega_sq_cmd)
    throttle_correct = omega_sq_to_throttle_correct(omega_sq_cmd)
    diff = throttle_correct - throttle_wrong

    print(f"{thrust_cmd:>15.2f} {throttle_wrong:>17.2f}% {throttle_correct:>17.2f}% {diff:>11.2f}%")

print("-" * 70)

# Resumo Final
print("\n" + "=" * 70)
print("📋 RESUMO")
print("=" * 70)
print("\n✅ CÓDIGO CORRETO:")
print("   throttle = sqrt(omega_sq / max_omega_sq) * 100")
print("   → Mantém relação linear: ω ∝ throttle")
print("\n❌ CÓDIGO ERRADO (original):")
print("   throttle = (omega_sq / max_omega_sq) * 100")
print("   → Relação quadrática incorreta: ω² ∝ throttle")
print("\n⚠️  IMPACTO DO BUG:")
print("   - Motores recebem MUITO MENOS throttle que o necessário")
print("   - Para 50% de velocidade, aplica apenas 25% de throttle")
print("   - Drone não gera empuxo suficiente")
print("   - Controlador tenta compensar, causando instabilidade")
print("\n🔧 CORREÇÃO APLICADA:")
print("   Arquivo: lib/MotorControl/MotorControl.cpp")
print("   Linha: 274 (adicionado sqrt)")
print("=" * 70)
