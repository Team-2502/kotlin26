package com.team2502.lib.config

import com.ctre.phoenix6.configs.Slot0Configs

@JvmRecord
data class PIDConfig(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
) {
    fun generate(): Slot0Configs {
        return Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
    }

    companion object {
        val ZEROS = PIDConfig(0.0, 0.0, 0.0)
    }
}