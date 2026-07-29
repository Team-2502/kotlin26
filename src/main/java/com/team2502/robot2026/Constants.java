package com.team2502.robot2026;

public final class Constants {
    public static final class OI {
        public static final int JOYSTICK_DRIVE_LEFT = 0;
        public static final int JOYSTICK_DRIVE_RIGHT = 1;
        public static final int JOYSTICK_OPERATOR = 2;

        public static final double JOYSTICK_DEADBAND = 0.5;
        public static final double JOYSTICK_ROTATION_DEADBAND = 0.1;
    }

    public static final class Drivetrain {
        public static final double MEASURED_MAX_SPEED_METERS_PER_SECOND = 4.58;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2 * 3.1415;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 18;
    }
}