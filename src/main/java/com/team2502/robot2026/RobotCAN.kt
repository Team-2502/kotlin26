package com.team2502.robot2026

import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.team2502.lib.config.CANConfig
import com.team2502.lib.config.EncoderConfig
import com.team2502.lib.config.GyroConfig
import com.team2502.lib.config.MotorConfig
import com.team2502.lib.config.PIDConfig
import com.team2502.lib.config.RobotCANBus

object RobotCAN {
    val DRIVE_MOTOR_CONFIG =
        MotorConfig(PIDConfig(kP = 0.3), false, NeutralModeValue.Brake)
    val TURN_MOTOR_CONFIG =
        MotorConfig(PIDConfig(kP = 40.0), true, NeutralModeValue.Coast)

    val INTAKE = CANConfig(
        18,
        "Intake",
        RobotCANBus.RIO,
        MotorConfig(PIDConfig(kP = 0.1), ccwp = false, neutralMode = NeutralModeValue.Coast))

    val TURRET = CANConfig(
        16,
        "Turret",
        RobotCANBus.RIO,
        MotorConfig(PIDConfig(kP = 8.0, kI = 2.5), ccwp = true, neutralMode = NeutralModeValue.Brake))

    val TUNNEL = CANConfig(
        21,
        "Tunnel",
        RobotCANBus.RIO,
        MotorConfig(PIDConfig(), ccwp = true, neutralMode = NeutralModeValue.Coast))

    val RAMP = CANConfig(
        22,
        "Ramp",
        RobotCANBus.RIO,
        MotorConfig(PIDConfig(), ccwp = false, neutralMode = NeutralModeValue.Brake))

    private val SHOOTER_PID_CONFIG = PIDConfig(kP = 0.03, kI = 0.12)

    val SHOOTER_RIGHT = CANConfig(
        13,
        "Shooter Right",
        RobotCANBus.SHOOTER,
        MotorConfig(SHOOTER_PID_CONFIG, ccwp = false, neutralMode = NeutralModeValue.Coast))

    val SHOOTER_LEFT = CANConfig(
        14,
        "Shooter Left",
        RobotCANBus.SHOOTER,
        MotorConfig(SHOOTER_PID_CONFIG, ccwp = true, neutralMode = NeutralModeValue.Coast))

    val HOOD = CANConfig(
        15,
        "Hood",
        RobotCANBus.SHOOTER,
        MotorConfig(PIDConfig(kP = 0.55), ccwp = false, neutralMode = NeutralModeValue.Brake)
    )

    val TURRET_ENCODER = CANConfig(
        2,
        "Turret Encoder",
        RobotCANBus.SHOOTER,
        EncoderConfig(SensorDirectionValue.CounterClockwise_Positive, 0.5)
    )

    val GYRO = CANConfig(
        23,
        "Gyro",
        RobotCANBus.RIO,
        GyroConfig()
    )
}