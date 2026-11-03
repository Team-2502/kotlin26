package com.team2502.lib

/**
 * Lifecycle hooks for a mechanism registered with [CoroutineRobotContainer]. The container calls these hooks
 * synchronously, normally on the shared robot coroutine thread; keep them short and non-blocking.
 *
 * Control coroutines decide what the mechanism should do and must coordinate ownership of its outputs.
 * Registering a subsystem does not start a control coroutine or arrange cleanup when that coroutine is cancelled.
 */
interface CoroutineSubsystem {

	/**
	 * Configures hardware and establishes initial state during container initialization, before control tasks start.
	 * The default implementation does nothing. Failures propagate through [CoroutineRobotContainer.initialize].
	 */
	fun initialize() {}

	/**
	 * Performs bounded sensor or telemetry work after each driver station update observed by the container.
	 * Runs in disabled mode too, so do not attempt motor outputs. An uncaught failure ends [CoroutineRobotContainer.updateLoop].
	 */
	fun periodic()

	/**
	 * Commands all owned actuators to a safe state, for example by stopping motors or applying a brake.
	 * Keep this non-blocking and safe to call repeatedly, including from a control coroutine's `finally` block.
	 *
	 * This hook does not cancel control tasks or prevent future output commands. The container reports failures and
	 * continues stopping other subsystems; a direct caller receives any exception thrown here.
	 */
	fun stop()
}
