package com.team2502.lib.config

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX

enum class RobotCANBus(val instance: CANBus) {
    RIO(CANBus.roboRIO()),
    SHOOTER(CANBus("can0")),
}

@JvmRecord
data class CANConfig<C : DeviceConfig<*>>(
    val canId: Int,
    val name: String,
    val bus: RobotCANBus,
    val config: C
)

fun CANConfig<MotorConfig>.create() = TalonFX(canId, bus.instance).apply {
    for (i in 0..3) {
        if (this.configurator.apply(config.generate()).isOK) {
            return@apply
        }
    }
    for (i in 0..20) {
        println("[WARNING] Failed to apply CANConfig to ID $canId (MOTOR) on CANBus ${bus.name}")
    }
}

fun CANConfig<EncoderConfig>.create() = CANcoder(canId, bus.instance).apply {
    for (i in 0..3) {
        if (this.configurator.apply(config.generate()).isOK) {
            return@apply
        }
    }
    for (i in 0..20) {
        println("[WARNING] Failed to apply CANConfig to ID $canId (ENCODER) on CANBus ${bus.name}")
    }
}

fun CANConfig<GyroConfig>.create() = Pigeon2(canId, bus.instance).apply {
    for (i in 0..3) {
        if (this.configurator.apply(config.generate()).isOK) {
            return@apply
        }
    }
    for (i in 0..20) {
        println("[WARNING] Failed to apply CANConfig to ID $canId (GYRO) on CANBus ${bus.name}")
    }
}