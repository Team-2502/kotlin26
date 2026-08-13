package com.team2502.lib.config

@JvmRecord
data class CANConfig<C : DeviceConfig<*>>(
    val canId: Int,
    val name: String,
    val busName: String,
    val config: C
)