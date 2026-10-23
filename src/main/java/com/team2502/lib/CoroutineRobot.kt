package com.team2502.lib

import choreo.auto.AutoFactory
import edu.wpi.first.wpilibj.IterativeRobotBase
import edu.wpi.first.hal.DriverStationJNI
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.util.concurrent.Event
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.sync.Mutex
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.max
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import kotlin.time.DurationUnit

/**
 * Runs robot coroutines on a shared thread. Mode tasks are cancelled on exit; background tasks survive mode changes.
 * Suspend regularly so other tasks can run, and stop owned mechanisms in `finally` when a task ends.
 */
abstract class CoroutineRobot : IterativeRobotBase(WATCHDOG_TIMEOUT.toDouble(DurationUnit.SECONDS)) {

	private val shutdown = Job()
	private val activeModeMutex = Mutex()
	private var activeMode: Job? = null

	private lateinit var robotScope: CoroutineScope

	/** Runs once after [robotInit]; use this as a suspending loop for work that continues across all modes. */
	open suspend fun runBackgroundRobot() {}

	/** Runs once after [simulationInit], only in simulation; continues across mode changes until it returns. */
	open suspend fun runBackgroundSimulation() {}

	/** Runs once per disabled entry; canceled when disabled mode ends. */
	abstract suspend fun runDisabled()

	/** Runs once per autonomous entry; canceled when autonomous mode ends. */
	abstract suspend fun runAutonomous()

	/** Runs once per teleop entry; canceled when teleop mode ends. */
	abstract suspend fun runTeleop()

	/** Runs once per test entry; canceled when test mode ends. */
	abstract suspend fun runTest()

	private fun startMode(modeFunction: suspend () -> Unit) {
		activeMode?.cancel()
		activeMode = robotScope.launch {
			activeModeMutex.lock()
			try {
				modeFunction()
			} finally {
				activeModeMutex.unlock()
			}
		}
	}

	/** Initializes the robot once before background and mode coroutines start. */
	override fun robotInit() {}

	/** Initializes simulation once before [runBackgroundSimulation] starts. */
	override fun simulationInit() {}

	final override fun disabledInit() = startMode(::runDisabled)
	final override fun autonomousInit() = startMode(::runAutonomous)
	final override fun teleopInit() = startMode(::runTeleop)
	final override fun testInit() = startMode(::runTest)

	final override fun robotPeriodic() {}
	final override fun simulationPeriodic() {}
	final override fun disabledPeriodic() {}
	final override fun autonomousPeriodic() {}
	final override fun teleopPeriodic() {}
	final override fun testPeriodic() {}

	final override fun disabledExit() {
		activeMode?.cancel()
	}

	final override fun autonomousExit() {
		activeMode?.cancel()
	}

	final override fun teleopExit() {
		activeMode?.cancel()
	}

	final override fun testExit() {
		activeMode?.cancel()
	}

	/** Runs the shared coroutine event loop until shutdown; reports and rethrows uncaught failures. */
	override fun startCompetition() {
		try {
			runBlocking(shutdown) {
				val robotJob = Job(coroutineContext[Job])
				robotScope = CoroutineScope(coroutineContext + robotJob)

				var commandJob: Job? = null
				try {
					robotInit()
					robotScope.launch { runBackgroundRobot() }

					if (isSimulation()) {
						simulationInit()
						robotScope.launch { runBackgroundSimulation() }
					}

					DriverStationJNI.observeUserProgramStarting()
					commandJob = launch { runCommandLoop() }

					val dsUpdates = Channel<Unit>(Channel.CONFLATED)
					DriverStationDataListener { dsUpdates.trySend(Unit) }.use {
						runLoopFunction(dsUpdates)
					}
				} finally {
					commandJob?.cancel()
					robotJob.cancel()
				}
			}
		} catch (e: CancellationException) {
			if (!shutdown.isCancelled) // Non-normal shutdown
				throw e
		} catch (t: Throwable) {
			DriverStation.reportError("Uncaught robot exception: $t", t.stackTrace)
			throw t
		}
	}

	/** Requests cancellation of the event loop and all robot tasks without waiting for cleanup. */
	override fun endCompetition() {
		shutdown.cancel()
	}

	private suspend fun CoroutineScope.runCommandLoop() {
		val loopStart = System.nanoTime()
		val periodNs = COMMAND_PERIOD.inWholeNanoseconds
		while (isActive) {
			CommandScheduler.getInstance().run()
			val current = System.nanoTime()
			delay((periodNs - (current - loopStart) % periodNs).nanoseconds)
		}
	}

	private suspend fun CoroutineScope.runLoopFunction(dsUpdates: Channel<Unit>) {
		var hasUpdate = false
		while (isActive) {
			val loopStart = System.nanoTime()
			loopFunc()
			if (hasUpdate) _updates.tryEmit(Unit)
			hasUpdate = false
			yield()
			if (dsUpdates.tryReceive().getOrNull() != null) {
				hasUpdate = true
			} else {
				val current = System.nanoTime()
				val periodNs = HEARTBEAT_PERIOD.inWholeNanoseconds
				withTimeoutOrNull(max(0L, periodNs - (current - loopStart)).nanoseconds) {
					dsUpdates.receive()
					hasUpdate = true
				}
			}
		}
	}

	/** Calls [onNewData] on a dedicated daemon thread when driver station data arrives; close to release resources. */
	class DriverStationDataListener(private val onNewData: () -> Unit) : AutoCloseable {
		private val event = Event()
		private val thread: Thread
		private val running = AtomicBoolean(false)

		init {
			DriverStationJNI.provideNewDataEventHandle(event.handle)
			thread = Thread({
				try {
					while (running.get()) {
						WPIUtilJNI.waitForObject(event.handle)
						if (running.get()) onNewData()
					}
				} catch (_: InterruptedException) {
					// Listener shutting down
				}
			}, "DriverStationDataListener")
			thread.isDaemon = true
			running.set(true)
			thread.start()
		}

		/** Unregisters the listener, waits for its thread to stop, and releases the event handle. Safe to call again. */
		override fun close() {
			if (!running.getAndSet(false)) return
			DriverStationJNI.removeNewDataEventHandle(event.handle)

			event.set() // Wake the waiting thread
			thread.join()
			event.close()
		}
	}

	companion object {
		private val COMMAND_PERIOD = 20.milliseconds
		private val HEARTBEAT_PERIOD = 20.milliseconds
		private val WATCHDOG_TIMEOUT = 50.milliseconds

		private val _updates =
			MutableSharedFlow<Unit>(replay = 0, extraBufferCapacity = 1, onBufferOverflow = BufferOverflow.DROP_OLDEST)
		private val updates: SharedFlow<Unit> = _updates.asSharedFlow()

		/** Suspends until the next driver station update; previously emitted updates are not replayed. */
		suspend fun awaitDriverStationData() {
			updates.first()
		}

		/** Waits for the next driver station update; returns false on [timeout] and propagates caller cancellation. */
		suspend fun awaitDriverStationData(timeout: Duration): Boolean {
			return withTimeoutOrNull(timeout) { updates.first() } != null
		}

		/** Waits at least one driver station update, then checks [condition] on each update until true. */
		suspend fun awaitDriverStationCondition(condition: () -> Boolean) {
			currentCoroutineContext().ensureActive()
			awaitDriverStationData()
			while (!condition()) awaitDriverStationData()
		}

		/** Waits for [condition] to go false then true, checking on driver station updates; skips the first wait if false. */
		suspend fun awaitDriverStationRisingEdge(condition: () -> Boolean) {
			currentCoroutineContext().ensureActive()
			while (condition()) awaitDriverStationData()
			while (!condition()) awaitDriverStationData()
		}

		/** Waits for [condition] to go true then false, checking on driver station updates; skips the first wait if true. */
		suspend fun awaitDriverStationFallingEdge(condition: () -> Boolean) {
			currentCoroutineContext().ensureActive()
			while (!condition()) awaitDriverStationData()
			while (condition()) awaitDriverStationData()
		}

		/**
		 * Launches [action] as a child even if [condition] is initially false; cancels it when false on a later update.
		 * Waits for child cleanup. An early return from [action] does not end the condition wait.
		 */
		suspend fun runWhile(condition: () -> Boolean, action: suspend () -> Unit) {
			coroutineScope {
				val actionJob = launch {
					action()
				}
				awaitDriverStationCondition { !condition() }
				actionJob.cancel()
			}
		}

		/**
		 * Sets the estimated pose to the start of [trajectoryName], applying [autoFactory]'s alliance flipping rules.
		 * Call once before the first path, with the robot physically at that start; this does not move the robot.
		 * Returns after the reset. If Choreo cannot obtain a starting pose, waits until canceled; use a timeout if needed.
		 */
		suspend fun resetOdometry(autoFactory: AutoFactory, trajectoryName: String) {
			runCommand(autoFactory.resetOdometry(trajectoryName))
		}

		/**
		 * Follows [trajectoryName] using [autoFactory] without resetting odometry, allowing sequential path segments.
		 * The robot should already be near the path's starting pose with a compatible velocity.
		 * Returns when Choreo's command ends, normally on elapsed time rather than measured arrival at the endpoint.
		 * Cancellation interrupts the command; the caller must stop the drivetrain in `finally`.
		 * Runs outside scheduler ownership checks, so keep a single task in control of the drivetrain.
		 */
		suspend fun followChoreoPath(autoFactory: AutoFactory, trajectoryName: String) {
			runCommand(autoFactory.trajectoryCmd(trajectoryName))
		}

		private suspend fun runCommand(command: Command, period: Duration = 20.milliseconds) {
			var interrupted = true
			val loopStart = System.nanoTime()
			val periodNs = period.inWholeNanoseconds
			currentCoroutineContext().ensureActive()
			command.initialize()
			try {
				while (true) {
					currentCoroutineContext().ensureActive()
					command.execute()
					if (command.isFinished)
						break
					val current = System.nanoTime()
					delay((periodNs - (current - loopStart) % periodNs).nanoseconds)
				}
				interrupted = false
			} finally {
				command.end(interrupted)
			}
		}
	}

}
