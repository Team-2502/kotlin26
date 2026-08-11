package com.team2502.robot2026

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team2502.robot2026.config.CANConfig
import com.team2502.robot2026.config.MotorConfig
import com.team2502.robot2026.config.PIDConfig

object RobotCAN {
    private val DRIVE_MOTOR_CONFIG =
        MotorConfig(PIDConfig(kP = 0.5), false, NeutralModeValue.Brake)
    private val TURN_MOTOR_CONFIG =
        MotorConfig(PIDConfig(kP = 0.5), false, NeutralModeValue.Brake)

    val FRONT_LEFT_DRIVE = CANConfig(1, "Front Left Drive", "rio", DRIVE_MOTOR_CONFIG)
    val FRONT_RIGHT_DRIVE = CANConfig(2, "Front Right Drive", "rio", DRIVE_MOTOR_CONFIG)
    val BACK_LEFT_DRIVE = CANConfig(3, "Back Left Drive", "rio", DRIVE_MOTOR_CONFIG)
    val BACK_RIGHT_DRIVE = CANConfig(4, "Back Right Drive", "rio", DRIVE_MOTOR_CONFIG)

    val INTAKE_MOTOR = CANConfig(
        18,
        "Intake Drive",
        "rio",
        MotorConfig(PIDConfig(kP = 0.1), ccwp = true, neutralMode = NeutralModeValue.Coast))
}