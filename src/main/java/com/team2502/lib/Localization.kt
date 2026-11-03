package com.team2502.lib

import com.ctre.phoenix6.swerve.SwerveDrivetrain
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.withTimeoutOrNull
import java.util.concurrent.atomic.AtomicReference
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

object Localization {

	private val _poses =
		MutableSharedFlow<Pose>(
			replay = 0,
			extraBufferCapacity = 1,
			onBufferOverflow = BufferOverflow.DROP_OLDEST
		)
	private val poses = _poses.asSharedFlow()
	private val lastPose = AtomicReference<Pose?>(null)

	private val poseNT = NetworkTableInstance.getDefault().getTable("localization")
	private val xPub = poseNT.getDoubleTopic("xPub").publish()
	private val yPub = poseNT.getDoubleTopic("yPub").publish()
	private val headingPub = poseNT.getDoubleTopic("headingPub").publish()
	private val posePub = poseNT.getStructTopic("pose", Pose2d.struct).publish()

	fun setPose(state: SwerveDrivetrain.SwerveDriveState, commandedVelocity: ChassisSpeeds) {
		val pose = Pose(state, commandedVelocity)
		lastPose.set(pose)
		_poses.tryEmit(pose)
	}

	fun updateNetworkTables() {
		val pose = getCurrentPose() ?: return
		xPub.set(pose.pose.x)
		yPub.set(pose.pose.y)
		headingPub.set(pose.pose.rotation.degrees)

		posePub.set(pose.pose)
		posePub.set(pose.pose)
	}

	fun getCurrentPose(): Pose? {
		return lastPose.get()
	}

	suspend fun awaitNewPose(timeout: Duration = 100.milliseconds): Pose? {
		return withTimeoutOrNull(timeout) { poses.first() }
	}

	data class Pose(val state: SwerveDrivetrain.SwerveDriveState, val commandedVelocity: ChassisSpeeds) {
		val pose: Pose2d = state.Pose
		val velocity: ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.rotation)
	}

}
