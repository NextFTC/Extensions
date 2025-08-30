package dev.nextftc.extensions.pedro

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle

class TurnTo(private val angle: Angle) : Command() {

    override val isDone: Boolean
        get() = !follower.isTurning

    override fun start() {
        follower.turnTo(angle.inRad)
    }
}