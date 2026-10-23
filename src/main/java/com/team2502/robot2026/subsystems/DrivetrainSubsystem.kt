package com.team2502.robot2026.subsystems

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.team2502.lib.CoroutineSubsystem
import com.team2502.lib.Localization
import com.team2502.robot2026.Constants.OI
import com.team2502.robot2026.subsystems.drive.CommandSwerveDrivetrain
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Subsystem

class DrivetrainSubsystem : CoroutineSubsystem {
	var speedLimit: SpeedLimiter? = null

	private val drivetrainSubsystem: CommandSwerveDrivetrain = TunerConstants.createDrivetrain()
	private val vision = VisionSubsystem(drivetrainSubsystem)

	val choreoXController = PIDController(5.0, 0.0, 0.0)
	val choreoYController = PIDController(5.0, 0.0, 0.0)
	val choreoThetaController = PIDController(5.0, 0.0, 0.0)

	private val driveNT = NetworkTableInstance.getDefault().getTable("driveSubsystemTable")
	private val statePub = driveNT.getStringTopic("state").publish()
	private val xPub = driveNT.getDoubleTopic("velX").publish()
	private val yPub = driveNT.getDoubleTopic("velY").publish()
	private val rotationPub = driveNT.getDoubleTopic("rotationRate").publish()

	private var state = "INITIALIZED"
	private val driveRequestBase = SwerveRequest.FieldCentric()
		.withDeadband(OI.TRANSLATION_DEADBAND_METERS_PER_SECOND)
		.withRotationalDeadband(OI.ROTATION_DEADBAND_RADIANS_PER_SECOND)
		.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

	val subsystem: Subsystem = drivetrainSubsystem

	init {
		choreoThetaController.enableContinuousInput(-Math.PI, Math.PI)
		drivetrainSubsystem.registerTelemetry { state ->
			val robotRelative = drivetrainSubsystem.kinematics.toChassisSpeeds(*state.ModuleTargets)
			val commanded = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, state.Pose.rotation)
			Localization.setPose(state.clone(), commanded)
		}
	}

	override fun periodic() {
		drivetrainSubsystem.periodic()
		vision.update()

		statePub.set(state)
		xPub.set(driveRequestBase.VelocityX)
		yPub.set(driveRequestBase.VelocityY)
		rotationPub.set(driveRequestBase.RotationalRate)
	}

	override fun stop() {
		state = "STOPPED"
		drivetrainSubsystem.setControl(SwerveRequest.SwerveDriveBrake())
	}

	fun resetChoreoPose(pose: Pose2d) {
		drivetrainSubsystem.resetChoreoPose(pose)
	}

	fun followTrajectory(sample: SwerveSample) {
		state = "DRIVE_AUTO"
		val pose = drivetrainSubsystem.state.Pose

		val vx = sample.vx + choreoXController.calculate(pose.x, sample.x)
		val vy = sample.vy + choreoYController.calculate(pose.y, sample.y)
		val omega = sample.omega + choreoThetaController.calculate(pose.rotation.radians, sample.heading)

		drivetrainSubsystem.setControl(
			SwerveRequest.FieldCentric()
				.withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
				.withVelocityX(vx)
				.withVelocityY(vy)
				.withRotationalRate(omega)
				.withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
		)
	}

	fun driveDirection(velX: Double, velY: Double, rotationRate: Double) {
		if (speedLimit != null) {
			// TODO: limit velX, velY
		}
		state = "DRIVE_DIRECTION"
		drivetrainSubsystem.setControl(
			driveRequestBase
				.withVelocityX(velX)
				.withVelocityY(velY)
				.withRotationalRate(rotationRate)
		)
	}

	class SpeedLimiter(val directionX: Double, val directionY: Double)
}