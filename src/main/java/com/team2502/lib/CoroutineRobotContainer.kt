package com.team2502.lib

import com.team2502.lib.CoroutineRobot.Companion.awaitDriverStationData
import edu.wpi.first.wpilibj.DriverStation

/**
 * Coordinates initialization, background updates, and stopping for registered [CoroutineSubsystem]s.
 * Register subsystems during construction, call [initialize] from [CoroutineRobot.robotInit], and run [updateLoop]
 * from [CoroutineRobot.runBackgroundRobot]. Call [stop] when disabling the robot or when explicitly shutting down outputs.
 *
 * Hooks run sequentially in registration order. Keep them short and non-blocking.
 * This container manages lifecycle calls; control coroutines remain responsible for mechanism ownership and cleanup.
 */
open class CoroutineRobotContainer {

	private val subsystems = ArrayList<CoroutineSubsystem>()

	/**
	 * Adds [subsystem] to the lifecycle calls and returns the same instance for assignment to a property.
	 * Register each instance once, before [initialize] or [updateLoop]; registration does not initialize it.
	 */
	protected fun <T : CoroutineSubsystem> registerSubsystem(subsystem: T): T {
		subsystems.add(subsystem)
		return subsystem
	}

	/**
	 * Initializes each registered subsystem in order. Call once during robot startup, before starting control tasks.
	 * Exceptions propagate and prevent later subsystems from initializing. Repeated calls repeat initialization.
	 */
	fun initialize() {
		for (subsystem in subsystems)
			subsystem.initialize()
	}

	/**
	 * Waits for each new driver station update, calls every subsystem's [CoroutineSubsystem.periodic], then publishes
	 * localization telemetry. Run one instance as a background task so updates continue in enabled and disabled modes.
	 *
	 * Suspends between updates rather than using a fixed timer. Cancellation or an uncaught update failure ends the loop
	 * and propagates to its caller; neither automatically calls [stop].
	 */
	suspend fun updateLoop() {
		while (true) {
			awaitDriverStationData()
			for (subsystem in subsystems)
				subsystem.periodic()
			Localization.updateNetworkTables()
		}
	}

	/**
	 * Requests safe outputs from every registered subsystem in order. Reports each failure to the driver station and
	 * continues attempting to stop the remaining subsystems.
	 *
	 * Does not cancel control tasks or end [updateLoop]. Coordinate with mechanism owners so they cannot immediately
	 * command outputs again; subsystem [CoroutineSubsystem.stop] implementations should tolerate repeated calls.
	 */
	fun stop() {
		for (subsystem in subsystems) {
			try {
				subsystem.stop()
			} catch (t: Throwable) {
				DriverStation.reportError("Failed to stop ${subsystem.javaClass.simpleName}: $t", t.stackTrace)
			}
		}
	}
}
