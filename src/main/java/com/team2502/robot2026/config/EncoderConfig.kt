package com.team2502.robot2026.config

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.MagnetSensorConfigs
import com.ctre.phoenix6.signals.SensorDirectionValue

@JvmRecord
data class EncoderConfig(
    val sensorDirection: SensorDirectionValue,
    val absoluteSensorDiscontinuityPoint: Double
) : DeviceConfig<CANcoderConfiguration> {
    override fun generate(): CANcoderConfiguration {
        return CANcoderConfiguration()
            .withMagnetSensor(
                MagnetSensorConfigs()
                    .withSensorDirection(sensorDirection)
                    .withAbsoluteSensorDiscontinuityPoint(
                        absoluteSensorDiscontinuityPoint
                    )
            )
    }
}