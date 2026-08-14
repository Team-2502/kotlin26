package com.team2502.robot2026

class Constants {
    object OI {
        const val JOYSTICK_DRIVE_LEFT: Int = 0
        const val JOYSTICK_DRIVE_RIGHT: Int = 1
        const val JOYSTICK_OPERATOR: Int = 2

        const val TRANSLATION_DEADBAND_METERS_PER_SECOND: Double = 0.5
        const val ROTATION_DEADBAND_RADIANS_PER_SECOND: Double = 0.1
    }

    object Drivetrain {
        const val MEASURED_MAX_SPEED_METERS_PER_SECOND: Double = 4.58
        const val MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND: Double = 2 * 3.1415

        const val DRIVE_MOTOR_KP: Double = 0.25
        const val DRIVE_MOTOR_KI: Double = 0.0
        const val DRIVE_MOTOR_KD: Double = 0.0

        const val TURN_MOTOR_KP: Double = 0.0
        const val TURN_MOTOR_KI: Double = 0.0
        const val TURN_MOTOR_KD: Double = 0.0
    }

    object Intake {
        const val INTAKE_IN_DUTY_CYCLE: Double = 1.0
        const val INTAKE_OUT_DUTY_CYCLE: Double = -1.0
        const val TUNNEL_IN_DUTY_CYCLE: Double = 1.0
        const val TUNNEL_OUT_DUTY_CYCLE: Double = -1.0

        const val UNJAM_CYCLE_TIME_SECONDS: Double = 0.5
    }

    object Shooter {
        const val SHOOTER_MIN_VELOCITY: Double = 30.0
        const val SHOOTER_MAX_VELOCITY: Double = 100.0
        const val HOOD_MIN_POSITION: Double = 0.0
        const val HOOD_MAX_POSITION: Double = 2.2992

        const val SHOOTER_RAMP_IN_DUTY_CYCLE: Double = 1.0
        const val SHOOTER_RAMP_OUT_DUTY_CYCLE: Double = -1.0
    }

    object Turret {
        const val MOTOR_TO_TURRET_RATIO: Double = 34.5
        const val ABS_TO_RELATIVE_RATIO: Double = 6.0

        const val TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = -0.199463

        // Max amount of revolution per frame of turret motor
        const val TURRET_CLAMP: Double = 2.5
    }

    object Localization {
        const val LIMELIGHT_SIDE_NAME: String = "limelight-side"
        const val LIMELIGHT_FRONT_NAME: String = "limelight-front"
    }
}