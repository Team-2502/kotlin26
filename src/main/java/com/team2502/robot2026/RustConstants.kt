package com.team2502.robot2026

import edu.wpi.first.math.geometry.Translation2d

/**
 * Constants converted from the RobotCode2026 Rust constants.rs file.
 * 
 * Rust modules are represented as nested static classes.
 */
class RustConstants private constructor() {
    object Config {
        /** Wheel-to-wheel width of robot.  */
        const val WHEELBASE_WIDTH_METERS: Double = 0.4953

        /** Wheel-to-wheel length of robot.  */
        const val WHEELBASE_LENGTH_METERS: Double = 0.5715
        const val FIELD_ORIENTED: Boolean = true
        const val MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND: Double = 16.3
        const val MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND: Double = 4.5
        const val MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND: Double = 2.0 * Math.PI
        const val MINIMUM_MODULE_VELOCITY_METERS_PER_SECOND: Double = 0.05
        const val SHOOTER_INITAL_DISTANCE_OFFSET_FEET: Double = 1.0

        const val HALF_FIELD_WIDTH_METERS: Double = 8.042656 / 2.0
        const val HALF_FIELD_LENGTH_METERS: Double = 16.513048 / 2.0
        const val BLUE_HUB_X_INCHES: Double = 182.11
        const val RED_HUB_X_INCHES: Double = 469.11

        val HUB_RED: Translation2d = Translation2d(11.915394, 4.034536)
        val HUB_BLUE: Translation2d = Translation2d(4.625594, 4.034536)
        val RED_PASS_TOP_OFFSET_METERS: Translation2d = Translation2d(2.0, 2.01)
        val RED_PASS_BOTTOM_OFFSET_METERS: Translation2d = Translation2d(2.0, -2.01)
        val BLUE_PASS_TOP_OFFSET_METERS: Translation2d = Translation2d(-2.0, 2.01)
        val BLUE_PASS_BOTTOM_OFFSET_METERS: Translation2d = Translation2d(-2.0, -2.01)
    }

    class RobotMap private constructor() {
        object DrivetrainMap {
            const val GYRO_ID: Int = 23

            /**
             * Equivalent to Rust Option<String>::None.
             * Set to a CAN bus name when using a non-default bus.
            </String> */
            val DRIVETRAIN_CANBUS: String? = null

            const val FL_ENCODER_ID: Int = 1
            const val FL_DRIVE_ID: Int = 2
            const val FL_TURN_ID: Int = 3

            const val BL_ENCODER_ID: Int = 4
            const val BL_DRIVE_ID: Int = 5
            const val BL_TURN_ID: Int = 6

            const val BR_ENCODER_ID: Int = 7
            const val BR_DRIVE_ID: Int = 8
            const val BR_TURN_ID: Int = 9

            const val FR_ENCODER_ID: Int = 10
            const val FR_DRIVE_ID: Int = 11
            const val FR_TURN_ID: Int = 12
        }

        object Shooter {
            const val SHOOTER_CANBUS: String = "can0"
            const val SHOOTER_MOTOR_LEFT_ID: Int = 13
            const val SHOOTER_MOTOR_RIGHT_ID: Int = 14
            const val HOOD_MOTOR_ID: Int = 15

            const val HOOD_MAX: Double = 2.4
            const val SHOOTER_SPEED: Double = 0.0
        }

        object Turret {
            const val SPIN_MOTOR_ID: Int = 16
            const val ENCODER_ID: Int = 2
        }

        object Intake {
            const val INTAKE_TOP_MOTOR_ID: Int = 17
            const val INTAKE_BOTTOM_MOTOR_ID: Int = 18
            const val PIVOT_TOP_MOTOR_ID: Int = 19
            const val PIVOT_BOTTOM_MOTOR_ID: Int = 20
            const val INDEXER_MOTOR_ID: Int = 21
            const val HANDOFF_MOTOR_ID: Int = 22

            const val INTAKE_DOWN_POSITION: Double = 0.0
            const val INTAKE_UP_POSITION: Double = 0.0
            const val INTAKE_IN_SPEED: Double = 1.0
            const val INTAKE_REVSERSE_SPEED: Double = -0.5
            const val HANDOFF_SPEED: Double = 1.0
            const val INTAKE_SPEED_OSCILLATION_TIME_SECS: Double = 0.25 // 0.25 og
        }
    }

    object Shooter {
        const val MAX_FLYWHEEL_SPEED: Double = 100.0
        const val SHOOTER_DISTANCE_ERROR_SMUDGE: Double = 1.0 // Original alternatives from Rust:
        // public static final double SHOOTER_DISTANCE_ERROR_SMUDGE = 0.88;  // Original
        // public static final double SHOOTER_DISTANCE_ERROR_SMUDGE = 0.94;  // Northern Lights
        // public static final double SHOOTER_DISTANCE_ERROR_SMUDGE = 1.125;
    }

    object Turret {
        const val RELATIVE_TO_TURRET_RATIO: Double = 34.5
        const val ABS_TO_TURRET_RATIO: Double = 5.75
        const val ABS_TO_REL_RATIO: Double = 6.0
        const val TURRET_MAX: Double = 180.0
        const val TURRET_MIN: Double = -180.0
        const val TURRET_CLAMP: Double = 2.5

        // Exponential moving average
        const val TURRET_EMA_ALPHA: Double = 0.9639
        const val TURRET_EMA_TOLERANCE: Double = 3.0
        const val TURRET_DEADZONE: Double = 0.3

        const val ORIGIN_TO_TURRET_CENTER_X_INCHES: Double = 4.0
        const val ORIGIN_TO_TURRET_CENTER_Y_INCHES: Double = 2.25

        // 0.05 margin, 2.2992 range
        const val HOOD_MAX_SOFTSTOP: Double = 2.2992 - 0.0
        const val HOOD_MIN_SOFTSTOP: Double = 0.0

        const val TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = -0.199463 // og
        const val TOLERANCE: Double = 0.001

        const val DISTANCE_SCALAR_SMUDGE_METERS: Double = 1.1
    }

    object Vision {
        /** Pitch of the Limelight in degrees.  */
        const val LIMELIGHT_PITCH_DEGREES: Double = 0.0

        /** Yaw of the Limelight in degrees (counterclockwise positive).  */
        const val LIMELIGHT_YAW_DEGREES: Double = 90.0

        /** Limelight height off the ground in inches.  */
        const val LIMELIGHT_HEIGHT_INCHES: Double = 20.92

        /** Distance from center of robot to Limelight in inches as (x, y).  */
        val ROBOT_CENTER_TO_LIMELIGHT_INCHES: Translation2d = Translation2d(11.118, 10.352)

        /**
         * Compensates for underestimation caused by angled views of the target.
         * 
         * Formula:
         * TX_FUDGE_FACTOR =
         * (realDistance - estimatedDistance) / (estimatedDistance * abs(tx))
         * 
         * Increase distance by 13.5% for every 20 degrees of |tx|.
         * Set this to 0 for new robots.
         */
        const val TX_FUDGE_FACTOR: Double = 0.0

        /** Starting FOM for the Limelight in meters.  */
        const val LIMELIGHT_BASE_FOM: Double = 0.001

        /** Meters of inaccuracy per degree of |tx|.  */
        const val LIMELIGHT_INACCURACY_PER_DEGREE_TX: Double = 0.015

        /** Meters of inaccuracy per radian/second of drivetrain angular velocity.  */
        const val LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY: Double = 2.0

        /** Meters of inaccuracy per meter/second of drivetrain linear velocity.  */
        const val LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY: Double = 2.0

        const val MAX_LIMELIGHT_POSE_DIFFERENCE_METERS: Double = 1.0
        const val LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT: Int = 4
    }

    object Drivetrain {
        const val SWERVE_TURN_RATIO: Double = 12.8
        const val SWERVE_DRIVE_RATIO: Double = 6.12

        // public static final double SWERVE_WHEEL_CIRCUMFERENCE_INCHES = Math.PI * 3.51715;
        const val SWERVE_WHEEL_CIRCUMFERENCE_INCHES: Double = 12.46875 * 0.831506

        const val WHEEL_ENCODER_STD_DEV: Double = 0.0005
        const val PIGEON_YAW_STD_DEV: Double = 0.001
        const val CURRENT_STATE_TRUST_SCALAR_DRIVE: Double = 0.003
        const val CURRENT_STATE_TRUST_SCALAR_YAW: Double = 0.1
        const val LIMELIGHT_YAW_TRUST: Double = 2.0 // degrees

        const val GYRO_OFFSET_UPDATE_RATIO: Double = 0.01

        const val DRIVETRAIN_ANGLE_SNAP_KP: Double = 8.0
        const val DRIVETRAIN_FISH_MODE_KP: Double = 7.0

        // Bevels towards intake side
        // public static final double FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS = 0.418212890625;
        // public static final double BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS = 0.340576;
        // public static final double BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS = 0.811767578125;
        // public static final double FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS = 0.90576171875;
        const val FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = 0.418
        const val BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = 0.340
        const val BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = 0.811
        const val FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: Double = 0.905
    }

    object Localization {
        // How much to distrust current localization state
        const val CURRENT_STATE_DRIVE_TRUST: Double = 3.0
        const val CURRENT_STATE_YAW_TRUST: Double = 0.1
        const val CURRENT_STATE_LINEAR_VELOCITY_TRUST: Double = 0.5
        const val CURRENT_STATE_ANGULAR_VELOCITY_TRUST: Double = 0.5

        const val LINEAR_VELOCITY_EMA_ALPHA: Double = 0.9
        const val ANGULAR_VELOCITY_EMA_ALPHA: Double = 0.775

        const val COMMANDED_VELOCITY_WEIGHT: Double = 0.65 // og 0.65
        const val VELOCITY_MIN_CONF: Double = 0.01
        const val LINEAR_VEL_CONF_SCALAR: Double = 0.1
        const val ANGULAR_VEL_CONF_SCALAR: Double = 0.1

        const val MAX_LIMELIGHT_POSE_DIFFERENCE_METERS: Double = 1.0
        const val LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT: Int = 10

        const val LIMELIGHT_YAW_TRUST: Double = 2.0

        const val POSE_ANTICIPATION_TIMESTEP_SECS: Double = 0.1
        const val YAW_ANTICIPATION_TIMESTEP_SECS: Double = 0.05
    }

    object Auto {
        const val SWERVE_TURN_KP: Double = 0.6
        const val SWERVE_DRIVE_KP: Double = 0.7
        const val SWERVE_DRIVE_KI: Double = 2.0
        const val SWERVE_DRIVE_KD: Double = 50.0
        const val SWERVE_DRIVE_KF: Double = 0.0 // Velocity FF
        const val SWERVE_DRIVE_KFA: Double = 0.0 // Acceleration FF

        const val SWERVE_DRIVE_MAX_ERR: Double = 0.15
        const val SWERVE_DRIVE_SUGGESTION_ERR: Double = 0.35
        const val SWERVE_DRIVE_IE: Double = 0.175 // Integral enable
    }

    object JoystickMap {
        // Joystick IDs (set in Driver Station)
        const val RIGHT_DRIVE: Int = 0
        const val LEFT_DRIVE: Int = 1
        const val OPERATOR: Int = 2
    }
}