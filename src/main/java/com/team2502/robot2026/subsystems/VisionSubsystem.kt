package com.team2502.robot2026.subsystems

import com.team2502.lib.LimelightHelpers
import com.team2502.lib.LimelightHelpers.PoseEstimate
import com.team2502.lib.LimelightHelpers.RawFiducial
import com.team2502.lib.Localization
import com.team2502.robot2026.Constants.Localization.ACCEPTABLE_OUTLIER_COUNT
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_FRONT_NAME
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_SIDE_NAME
import com.team2502.robot2026.Constants.Localization.MAX_LIMELIGHT_POSE_DIFFERENCE_METERS
import com.team2502.robot2026.Constants.Weights.CHASSIS_XY_STDDEV_COEFFICIENT
import com.team2502.robot2026.Constants.Weights.DEFAULT_XY_STDDEV
import com.team2502.robot2026.Constants.Weights.VISION_HEADING_STD_DEV_HARDCODE
import com.team2502.robot2026.subsystems.drive.TunerConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import java.util.*
import kotlin.math.pow


class VisionSubsystem(private val drivetrain: TunerConstants.TunerSwerveDrivetrain) {
    private var previousEstimateTimestamp = 0.0

    var outlierCount = 0

    fun getVisionPose2d(limelightName: String): PoseEstimate? {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
    }

    private fun getStdDev(currentPose: Pose2d, limelightEstimate: PoseEstimate?): Matrix<N3, N1>? {
        val estimate = limelightEstimate ?: return null

        val optStdDev =
            Arrays.stream(estimate.rawFiducials)
                .mapToDouble { fiducial: RawFiducial? -> fiducial!!.distToCamera }.min()
        var stdDev: Double

        var closestTagDist = Double.MAX_VALUE
        for (fiducial in estimate.rawFiducials) {
            if (fiducial.distToCamera < closestTagDist) {
                closestTagDist = fiducial.distToCamera
            }
        }
        if (closestTagDist < 1) closestTagDist = 1.0

        val distanceToCurrentSTD: Double = currentPose.translation.getDistance(estimate.pose.translation)

        stdDev =
            ((CHASSIS_XY_STDDEV_COEFFICIENT
                    * closestTagDist.pow(2.0) * distanceToCurrentSTD)
                    / estimate.tagCount
                    + DEFAULT_XY_STDDEV) / 10.0

        return VecBuilder.fill(stdDev, stdDev, VISION_HEADING_STD_DEV_HARDCODE)
    }

    fun update() {
        val currentPose = Localization.getCurrentPose()?.pose ?: return
        val sidePos = getVisionPose2d(LIMELIGHT_SIDE_NAME)
        val frontPos = getVisionPose2d(LIMELIGHT_FRONT_NAME)
        val sideStdDev = getStdDev(currentPose, sidePos)
        val frontStdDev = getStdDev(currentPose, frontPos)

        val sideFirst = sidePos != null && frontPos != null && sidePos.timestampSeconds < frontPos.timestampSeconds
        if (sideFirst) {
            addToFilter(currentPose, sidePos, sideStdDev)
            addToFilter(currentPose, frontPos, frontStdDev)
        } else {
            addToFilter(currentPose, frontPos, frontStdDev)
            addToFilter(currentPose, sidePos, sideStdDev)
        }
    }

    fun addToFilter(currentPose: Pose2d, pose: PoseEstimate?, stdDev: Matrix<N3, N1>?) {
        if (stdDev != null && pose != null) {
            if (pose.tagCount > 0) {
                if (pose.timestampSeconds > previousEstimateTimestamp) {
                    previousEstimateTimestamp = pose.timestampSeconds
                    val translationDelta = pose.pose.minus(currentPose)
                    val yawDelta = pose.pose.rotation.minus(currentPose.rotation)

                    if (translationDelta.translation.norm > MAX_LIMELIGHT_POSE_DIFFERENCE_METERS
                        || yawDelta.degrees > 90.0
                        || yawDelta.degrees < -90.0) {
                        outlierCount++

                        if (outlierCount < ACCEPTABLE_OUTLIER_COUNT) {
                            return
                        }
                    }
                    outlierCount = 0

                    drivetrain.addVisionMeasurement(
                        pose.pose,
                        pose.timestampSeconds,
                        stdDev,
                    )
                }
            }
        }
    }
}