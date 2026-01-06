package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Vector2d
import dev.nextftc.core.commands.utility.LambdaCommand
import java.util.function.Supplier

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

    /**
     * Creates a command for driver-controlled operation.
     *
     * @param xPower A function providing the x-axis power.
     * @param yPower A function providing the y-axis power.
     * @param headingPower A function providing the heading power.
     * @param robotCentric Whether the controls are robot-centric (default: true).
     * @return A [LambdaCommand] that updates the drivetrain powers.
     */
    @JvmOverloads fun driverControlledCommand(
        xPower: Supplier<Double>,
        yPower: Supplier<Double>,
        headingPower: Supplier<Double>,
        robotCentric: Boolean = true
    ) = LambdaCommand().setUpdate {
        var transVel = Vector2d(xPower.get(), yPower.get())

        if (!robotCentric) {
            transVel = getPose().heading * transVel
        }

        setDrivePowers(PoseVelocity2d(transVel, headingPower.get()))
    }
}