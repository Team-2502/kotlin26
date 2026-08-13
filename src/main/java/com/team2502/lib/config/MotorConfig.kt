package com.team2502.lib.config

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

@JvmRecord
data class MotorConfig(
    val pid: PIDConfig,
    // Counter Clockwise Positive
    val ccwp: Boolean,
    val neutralMode: NeutralModeValue
) : DeviceConfig<TalonFXConfiguration> {
    override fun generate(): TalonFXConfiguration {
        return TalonFXConfiguration()
            .withSlot0(pid.generate())
            .withMotorOutput(
                MotorOutputConfigs()
                    .withInverted(
                        if (ccwp)
                            InvertedValue.CounterClockwise_Positive
                        else
                            InvertedValue.Clockwise_Positive
                    )
                    .withNeutralMode(neutralMode)
            )
    }
}