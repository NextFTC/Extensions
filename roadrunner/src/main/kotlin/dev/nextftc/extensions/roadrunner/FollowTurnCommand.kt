package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.core.commands.Command

class FollowTurnCommand(private val mecanumDrive: NextFtcMecanumDrive, private val turn: TimeTurn) : Command() {
    val timer = ElapsedTime()

    override val isDone: Boolean
        get() = timer.seconds() >= turn.duration

    override fun start() {
        timer.reset()
    }

    override fun update() {
        val txWorldTarget: Pose2dDual<Time> = turn[timer.seconds()]

        val robotVelRobot: PoseVelocity2d = mecanumDrive.updatePoseEstimate()

        val command =
            mecanumDrive.controller.compute(txWorldTarget, mecanumDrive.getPose(), robotVelRobot)

        mecanumDrive.setDrivePowersFF(command)
    }

    override fun stop(interrupted: Boolean) {
        mecanumDrive.setDrivePowersFF(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
    }
}