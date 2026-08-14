package com.team2502.robot2026.subsystems

import com.team2502.lib.LimelightHelpers
import com.team2502.lib.config.create
import com.team2502.robot2026.RobotCAN.GYRO
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LocalizationSubsystem : SubsystemBase() {
    val gyro = GYRO.create()

    fun getVisionPose3d(limelightName: String) {
        val pose = LimelightHelpers.getBotPose3d(limelightName)
        print(pose)
    }
}