@file:JvmName("Commands")

package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.NullCommand

internal fun seqCons(hd: Command, tl: Command): Command =
    when (tl) {
        is NullCommand -> hd
        is SequentialGroup -> SequentialGroup(*(arrayOf(hd) + tl.commands))
        else -> SequentialGroup(hd, tl)
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
    fun make(t: TimeTurn): Command
}

fun interface TrajectoryCommandFactory {
    fun make(t: TimeTrajectory): Command
}

