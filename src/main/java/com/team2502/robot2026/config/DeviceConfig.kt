package com.team2502.robot2026.config

import com.ctre.phoenix6.configs.ParentConfiguration

interface DeviceConfig<T : ParentConfiguration> {
    fun generate(): T
}