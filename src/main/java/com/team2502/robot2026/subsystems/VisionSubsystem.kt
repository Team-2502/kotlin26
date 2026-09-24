package com.team2502.robot2026.subsystems

import com.ctre.phoenix6.Utils
import com.team2502.lib.LimelightHelpers
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_FRONT_NAME
import com.team2502.robot2026.Constants.Localization.LIMELIGHT_SIDE_NAME
import com.team2502.robot2026.RobotContainer
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import java.util.*
import kotlin.math.pow


object VisionSubsystem {
    fun getVisionPose2d(limelightName: String) : LimelightHelpers.PoseEstimate? {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    fun getStdDev(limelightName: String) : Matrix<N3, N1>? {
        val tagArea = LimelightHelpers.getTA(limelightName)
        val distAreaModifier = 0.00000961227 * tagArea.pow(-1.25093)

        val stdDev = LimelightHelpers.getLimelightNTDoubleArray(
            limelightName,
            "stddevs"
        )

        if (stdDev.size == 0) {
            return null
        } else {
            return VecBuilder.fill(
                0.05,
                0.05,
                0.05,
            );

            // TODO: think about this lol
//            return VecBuilder.fill(
//                distAreaModifier,
//                distAreaModifier,
//                stdDev[5]
//            );
        }
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