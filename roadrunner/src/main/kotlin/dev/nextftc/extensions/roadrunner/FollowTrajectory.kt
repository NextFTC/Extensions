package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Trajectory
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.commands.Command

/**
 * Follows a trajectory parametrized by time.
 *
 * Note that the output of [TrajectoryCommandBuilder.build] only returns a [Command]
 * object and not a [FollowTrajectory] object,
 * because the builder sequences the trajectory with other actions,
 * such as [Turn]s, [dev.nextftc.core.commands.delays.Delay]s or user-specified callbacks.
 */
class FollowTrajectory(private val mecanumDrive: NextFTCMecanumDrive, private val trajectory: TimeTrajectory) : Command() {
    constructor(mecanumDrive: NextFTCMecanumDrive, trajectory: Trajectory) :
            this(mecanumDrive, TimeTrajectory(trajectory))

    val timer = ElapsedTime()

    override val isDone: Boolean
        get() = timer.seconds() >= trajectory.duration

    override fun start() {
        timer.reset()
    }

    override fun update() {
        val txWorldTarget: Pose2dDual<Time> = trajectory[timer.seconds()]

        val robotVelRobot: PoseVelocity2d = mecanumDrive.updatePoseEstimate()

        val command =
            mecanumDrive.controller.compute(txWorldTarget, mecanumDrive.getPose(), robotVelRobot)

        mecanumDrive.setDrivePowersFF(command)
    }

    override fun stop(interrupted: Boolean) {
        mecanumDrive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
    }
}