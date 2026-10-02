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
    }

    object Intake {
        const val INTAKE_IN_DUTY_CYCLE: Double = 1.0
        const val INTAKE_OUT_DUTY_CYCLE: Double = -1.0
        const val TUNNEL_IN_DUTY_CYCLE: Double = 1.0
        const val TUNNEL_OUT_DUTY_CYCLE: Double = -1.0
        const val RAMP_IN_DUTY_CYCLE: Double = 0.5
        const val RAMP_OUT_DUTY_CYCLE: Double = -0.5

        const val UNJAM_CYCLE_TIME_SECONDS: Double = 0.25
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

        const val ORIGIN_TO_TURRET_CENTER_X = 0.1016
        const val ORIGIN_TO_TURRET_CENTER_Y = 0.05715
    }

    object Localization {
        const val LIMELIGHT_SIDE_NAME: String = "limelight-side"
        const val LIMELIGHT_FRONT_NAME: String = "limelight-front"
    }

    /// Choreo coordinates: blue right is (0,0), towards red is +x
    object Field {
        const val HALF_FIELD_WIDTH_METERS = 8.042656 / 2.0 /// y
        const val HALF_FIELD_LENGTH_METERS = 16.513048 / 2.0 /// x

        const val BLUE_HUB_X_METERS = 4.625594
        const val BLUE_HUB_Y_METERS = 4.034536
        const val RED_HUB_X_METERS = 11.915394
        const val RED_HUB_Y_METERS = 4.034536

        const val RED_TOP_PASS_OFFSET_X_METERS = 2.0
        const val RED_TOP_PASS_OFFSET_Y_METERS = 2.01
        const val RED_BOTTOM_PASS_OFFSET_X_METERS = 2.0
        const val RED_BOTTOM_PASS_OFFSET_Y_METERS = -2.01
        const val BLUE_TOP_PASS_OFFSET_X_METERS = -2.0
        const val BLUE_TOP_PASS_OFFSET_Y_METERS = 2.01
        const val BLUE_BOTTOM_PASS_OFFSET_X_METERS = -2.0
        const val BLUE_BOTTOM_PASS_OFFSET_Y_METERS = -2.01
    }

    object Weights {
        const val COMMANDED_VELOCITY_WEIGHT = 0.65
        const val GYRO_EMA_WEIGHT = 0.01

        const val CHASSIS_XY_STDDEV_COEFFICIENT = 0.04
        const val DEFAULT_XY_STDDEV = 0.25
    }
}