package com.team2502.lib.config

import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs

@JvmRecord
data class GyroConfig(
    val enableCompass: Boolean
) : DeviceConfig<Pigeon2Configuration> {
    /**
     * Default configuration with compass disabled.
     */
    constructor() : this(false)

    override fun generate(): Pigeon2Configuration {
        return Pigeon2Configuration()
            .withPigeon2Features(
                Pigeon2FeaturesConfigs()
                    .withEnableCompass(enableCompass)
            )
    }
}