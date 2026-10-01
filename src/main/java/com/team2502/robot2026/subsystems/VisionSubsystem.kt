package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.Utils
import com.team2502.lib.LimelightHelpers
import com.team2502.lib.LimelightHelpers.PoseEstimate
import com.team2502.lib.LimelightHelpers.RawFiducial
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_FRONT_NAME
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_SIDE_NAME
import com.team2502.robot2026.Constants.Weights.CHASSIS_XY_STDDEV_COEFFICIENT
import com.team2502.robot2026.Constants.Weights.DEFAULT_XY_STDDEV
import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotState
import java.util.*
import kotlin.math.pow


object VisionSubsystem {
    fun getVisionPose2d(limelightName: String) : PoseEstimate? {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    private fun calculateStandardDeviation(currentPose: PoseEstimate?, estimate: PoseEstimate): Matrix<N3, N1>? {


        val optStdDev =
            Arrays.stream(estimate.rawFiducials)
                .mapToDouble { fiducial: RawFiducial? -> fiducial!!.distToCamera }.min()
        var stdDev: Double
        val headingStdDev = 1000.0

        var closestTagDist = Double.MAX_VALUE
        for (fiducial in estimate.rawFiducials) {
            if (fiducial.distToCamera < closestTagDist) {
                closestTagDist = fiducial.distToCamera
            }
        }
        if (closestTagDist < 1) closestTagDist = 1.0

        val distanceToCurrentSTD: Double = currentPose.translation
                .getDistance(estimate.pose.translation)

        stdDev =
            ((CHASSIS_XY_STDDEV_COEFFICIENT
                    * closestTagDist.pow(2.0) * distanceToCurrentSTD)
                    / estimate.tagCount
                    + DEFAULT_XY_STDDEV)

        return VecBuilder.fill(stdDev, stdDev, headingStdDev)
    }

    fun update() {
        val driveSubsystem = RobotContainer.INSTANCE.drivetrainSubsystem

        val sidePos = getVisionPose2d(LIMELIGHT_SIDE_NAME)
        val frontPos = getVisionPose2d(LIMELIGHT_FRONT_NAME)
        val sideStdDev = getStdDev(LIMELIGHT_SIDE_NAME)
        val frontStdDev = getStdDev(LIMELIGHT_FRONT_NAME)

        if (sideStdDev != null && sidePos != null) {
            if (sidePos.tagCount > 0) {
                driveSubsystem.addVisionMeasurement(
                    sidePos.pose,
                    Utils.fpgaToCurrentTime(sidePos.timestampSeconds),
                    sideStdDev,
                    )
            }
        }

        if (frontStdDev != null && frontPos != null) {
            if (frontPos.tagCount > 0) {
                driveSubsystem.addVisionMeasurement(
                    frontPos.pose,
                    Utils.fpgaToCurrentTime(frontPos.timestampSeconds),
                    frontStdDev,
                    )
            }
        }
    }
}