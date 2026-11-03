// stolen from frc 2052 knightcrawlers :)
package com.team2502.lib

import edu.wpi.first.wpilibj.Joystick

class TMJoystick(port: Int) {
    private val joystick = Joystick(port)

    // --- Stick Buttons ---
    val trigger: Boolean
        get() = joystick.getRawButton(1)

    val middleThumbButton: Boolean
        get() = joystick.getRawButton(2)

    val leftThumbButton: Boolean
        get() = joystick.getRawButton(3)

    val rightThumbButton: Boolean
        get() = joystick.getRawButton(4)

    // --- Base Buttons (Left Side) ---
    val leftBaseTopLeft: Boolean
        get() = joystick.getRawButton(5)

    val leftBaseTopMiddle: Boolean
        get() = joystick.getRawButton(6)

    val leftBaseTopRight: Boolean
        get() = joystick.getRawButton(7)

    val leftBaseBottomLeft: Boolean
        get() = joystick.getRawButton(10)

    val leftBaseBottomMiddle: Boolean
        get() = joystick.getRawButton(9)

    val leftBaseBottomRight: Boolean
        get() = joystick.getRawButton(8)

    // --- Base Buttons (Right Side) ---
    val rightBaseTopLeft: Boolean
        get() = joystick.getRawButton(13)

    val rightBaseTopMiddle: Boolean
        get() = joystick.getRawButton(12)

    val rightBaseTopRight: Boolean
        get() = joystick.getRawButton(11)

    val rightBaseBottomLeft: Boolean
        get() = joystick.getRawButton(14)

    val rightBaseBottomMiddle: Boolean
        get() = joystick.getRawButton(15)

    val rightBaseBottomRight: Boolean
        get() = joystick.getRawButton(16)

    // --- Axes ---
    val x: Double
        get() = joystick.getRawAxis(0)

    val y: Double
        get() = -joystick.getRawAxis(1)

    /// CCW+
    val twist: Double
        get() = -joystick.getRawAxis(2)

    val slider: Double
        get() = joystick.getRawAxis(3)
}
