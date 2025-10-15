package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time

/**
 * Represents a RoadRunner Mecanum drivetrain from the QuickStart.
 */
abstract class NextFTCMecanumDrive {
    abstract val controller: HolonomicController

    abstract fun getPose(): Pose2d

    abstract fun setDrivePowersFF(powers: PoseVelocity2dDual<Time>)

    open fun setDrivePowersFF(powers: PoseVelocity2d) =
        setDrivePowersFF(PoseVelocity2dDual.constant(powers, 1))

    abstract fun setDrivePowers(powers: PoseVelocity2d)

    abstract fun updatePoseEstimate(): PoseVelocity2d

    abstract fun commandBuilder(beginPose: Pose2d): TrajectoryCommandBuilder

    open fun commandBuilder() = commandBuilder(getPose())
}