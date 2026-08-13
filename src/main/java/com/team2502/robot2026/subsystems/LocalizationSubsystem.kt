package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.team2502.robot2026.RobotCAN.GYRO
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LocalizationSubsystem : SubsystemBase() {
    val gyro = Pigeon2(GYRO.canId, GYRO.busName)


}