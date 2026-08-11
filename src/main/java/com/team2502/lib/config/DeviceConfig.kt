package com.team2502.lib.config

import com.ctre.phoenix6.configs.ParentConfiguration

interface DeviceConfig<T : ParentConfiguration> {
    fun generate(): T
}