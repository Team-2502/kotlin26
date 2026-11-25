package com.team2502.lib

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance

class PIDTuner(name: String, defaultKp: Double = 0.0, defaultKi: Double = 0.0, defaultKd: Double = 0.0) {

    private val autonNT = NetworkTableInstance.getDefault().getTable("PID")
    private val kPSub = autonNT.getDoubleTopic("kP-$name").getEntry(defaultKp)
    private val kISub = autonNT.getDoubleTopic("kI-$name").getEntry(defaultKi)
    private val kDSub = autonNT.getDoubleTopic("kD-$name").getEntry(defaultKd)
    private val errorPub = autonNT.getDoubleTopic("error-$name").publish()

    init {
        kPSub.setDefault(defaultKp)
        kISub.setDefault(defaultKi)
        kDSub.setDefault(defaultKd)
        errorPub.setDefault(0.0)
    }

    fun create(): PIDController {
        return PIDController(kPSub.get(), kISub.get(), kDSub.get())
    }

    fun reportError(error: Double) {
        errorPub.set(error)
    }


}