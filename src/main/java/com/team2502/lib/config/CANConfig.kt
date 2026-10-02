package com.team2502.lib.config

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.team2502.robot2026.RobotCAN.GYRO

enum class RobotCANBus(val instance: CANBus) {
    RIO (CANBus.roboRIO()),
    SHOOTER (CANBus("can0")),
}

@JvmRecord
data class CANConfig<C : DeviceConfig<*>>(
    val canId: Int,
    val name: String,
    val bus: RobotCANBus,
    val config: C) {


}

fun CANConfig<MotorConfig>.create() = TalonFX(canId, bus.instance).apply {
    this.configurator.apply(config.generate())
}

fun CANConfig<EncoderConfig>.create() = CANcoder(canId, bus.instance).apply {
    this.configurator.apply(config.generate())
}

fun CANConfig<GyroConfig>.create() = Pigeon2(GYRO.canId, bus.instance).apply {
    this.configurator.apply(config.generate())
}