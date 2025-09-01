package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.NullCommand

/**
 * Utility functions for constructing trajectories and turns;
 * the name [seqCons] refers to the "cons" operation in functional programming.
 */
internal fun seqCons(head: Command, tail: Command): Command =
    when (tail) {
        is NullCommand -> head
        is SequentialGroup -> SequentialGroup(*(arrayOf(head) + tail.commands))
        else -> SequentialGroup(head, tail)
    }

internal sealed class MarkerFactory(
    val segmentIndex: Int,
) {
    abstract fun make(t: TimeTrajectory, segmentDisp: Double): Command
}

internal class TimeMarkerFactory(segmentIndex: Int, val dt: Double, val a: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Delay(t.profile.inverse(segmentDisp) + dt), a)
}

internal class DispMarkerFactory(segmentIndex: Int, val ds: Double, val a: Command) : MarkerFactory(segmentIndex) {
    override fun make(t: TimeTrajectory, segmentDisp: Double) =
        seqCons(Delay(t.profile.inverse(segmentDisp + ds)), a)
}

fun interface TurnCommandFactory {
    fun make(turn: TimeTurn): Command
}

fun interface TrajectoryCommandFactory {
    fun make(trajectory: TimeTrajectory): Command
}

