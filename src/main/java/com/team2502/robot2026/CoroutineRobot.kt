package com.team2502.robot2026

import edu.wpi.first.wpilibj.IterativeRobotBase
import edu.wpi.first.hal.DriverStationJNI
import kotlinx.coroutines.*

abstract class CoroutineRobot(private val period: Double = 0.02) : IterativeRobotBase(period) {

    private val shutdown = Job()

    protected lateinit var robotScope: CoroutineScope
        private set

    override fun startCompetition() {
        try {
            runBlocking(shutdown) {
                robotScope = this

                robotInit()

                if (isSimulation()) {
                    simulationInit()
                }

                DriverStationJNI.observeUserProgramStarting()

                val loopStartMs = System.nanoTime() / 1000000
                val periodMs = (period * 1000).toLong()
                while (isActive) {
                    loopFunc()
                    val currentTime = System.nanoTime() / 1000000
                    delay(periodMs - (currentTime - loopStartMs) % periodMs)
                }
            }
        } catch (_: CancellationException) {
            // Normal shutdown
        }
    }

    override fun endCompetition() {
        shutdown.cancel()
    }

    protected fun launchRobot(block: suspend CoroutineScope.() -> Unit): Job {
        return robotScope.launch(block = block)
    }
}