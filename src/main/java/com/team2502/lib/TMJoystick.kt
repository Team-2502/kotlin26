// stolen from frc 2052 knightcrawlers :)
package com.team2502.lib

import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.Trigger

class TMJoystick(port: Int) : CommandJoystick(port) {
    // --- Stick Buttons ---
    fun frontTrigger(): Trigger {
        return button(1)
    }

    fun middleThumbButton(): Trigger {
        return button(2)
    }

    fun leftThumbButton(): Trigger {
        return button(3)
    }

    fun rightThumbButton(): Trigger {
        return button(4)
    }

    // --- Base Buttons (Left Side) ---
    fun leftBaseTopLeft(): Trigger {
        return button(5)
    }

    fun leftBaseTopMiddle(): Trigger {
        return button(6)
    }

    fun leftBaseTopRight(): Trigger {
        return button(7)
    }

    fun leftBaseBottomLeft(): Trigger {
        return button(10)
    }

    fun leftBaseBottomMiddle(): Trigger {
        return button(9)
    }

    fun leftBaseBottomRight(): Trigger {
        return button(8)
    }

    // --- Base Buttons (Right Side) ---
    fun rightBaseTopLeft(): Trigger {
        return button(13)
    }

    fun rightBaseTopMiddle(): Trigger {
        return button(12)
    }

    fun rightBaseTopRight(): Trigger {
        return button(11)
    }

    fun rightBaseBottomLeft(): Trigger {
        return button(14)
    }

    fun rightBaseBottomMiddle(): Trigger {
        return button(15)
    }

    fun rightBaseBottomRight(): Trigger {
        return button(16)
    }

    // --- Axes ---
    override fun getX(): Double {
        return getRawAxis(0)
    }

    override fun getY(): Double {
        return -getRawAxis(1)
    }

    /// CCW+
    override fun getTwist(): Double {
        return -getRawAxis(2)
    }

    val slider: Double
        get() = -getRawAxis(3)
}