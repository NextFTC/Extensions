package dev.nextftc.extensions.pedro

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

class TurnTo(private val angle: Angle) : Command() {

    init {
        named("TurnTo($angle)")
    }

    override val isDone: Boolean
        get() = !follower.isTurning

    override fun start() {
        follower.turnTo(angle.inRad)
    }
}