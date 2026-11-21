package com.team2502.robot2026.control.teleop

import com.team2502.lib.CoroutineRobot.Companion.awaitDriverStationData
import com.team2502.robot2026.control.common.ShooterControl
import com.team2502.robot2026.control.common.ShooterControl.TargetingMode
import com.team2502.robot2026.subsystems.RobotContainer
import kotlinx.coroutines.launch
import kotlinx.coroutines.coroutineScope

class TeleopShooterControl(private val robotContainer: RobotContainer) {
    val shooterControl = ShooterControl(robotContainer)

    suspend fun updateLoop() {
        coroutineScope {
            launch { shooterControl.updateLoop() }
            launch { modeControlLoop() }
        }
    }

    private suspend fun modeControlLoop() {
        try {
            while (true) {
                awaitDriverStationData()
                if (robotContainer.operatorJoystick.leftBaseBottomMiddle) {
                    shooterControl.mode = TargetingMode.IDLE
                } else if (robotContainer.operatorJoystick.leftBaseBottomRight) {
                    shooterControl.mode = TargetingMode.AUTOMATIC
                }
            }
        } finally {
            shooterControl.mode = TargetingMode.IDLE
        }
    }
}