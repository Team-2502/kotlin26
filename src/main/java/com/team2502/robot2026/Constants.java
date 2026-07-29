package com.team2502.robot2026;

public final class Constants {
    public static final class OI {
        public static final int JOYSTICK_DRIVE_LEFT = 0;
        public static final int JOYSTICK_DRIVE_RIGHT = 1;
        public static final int JOYSTICK_OPERATOR = 2;

        public static final double TRANSLATION_DEADBAND_METERS_PER_SECOND = 0.5;
        public static final double ROTATION_DEADBAND_RADIANS_PER_SECOND = 0.1;
    }

    public static final class Drivetrain {
        public static final double MEASURED_MAX_SPEED_METERS_PER_SECOND = 4.58;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2 * 3.1415;

        public static final double DRIVE_MOTOR_KP = 0.25;
        public static final double DRIVE_MOTOR_KI = 0.0;
        public static final double DRIVE_MOTOR_KD = 0.0;

        public static final double TURN_MOTOR_KP = 0.0;
        public static final double TURN_MOTOR_KI = 0.0;
        public static final double TURN_MOTOR_KD = 0.0;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR_ID = 18;

        public static final double INTAKE_IN_DUTY_CYCLE = 1.0;
        public static final double INTAKE_OUT_DUTY_CYCLE = -1.0;
    }

    public static final class Handoff {
        public static final int TUNNEL_MOTOR_ID = 21;
        public static final int RAMP_MOTOR_ID = 22;

        public static final double HANDOFF_TUNNEL_IN_DUTY_CYCLE = 1.0;
        public static final double HANDOFF_TUNNEL_OUT_DUTY_CYCLE = -1.0;
        public static final double HANDOFF_RAMP_IN_DUTY_CYCLE = 1.0;
        public static final double HANDOFF_RAMP_OUT_DUTY_CYCLE = -1.0;
    }
}