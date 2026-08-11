package com.team2502.robot2026

import edu.wpi.first.wpilibj.RobotBase
import java.util.function.Supplier

object Main {
    @JvmStatic
    fun main(args: Array<String>) {
        RobotBase.startRobot { Robot() }
    }
}
