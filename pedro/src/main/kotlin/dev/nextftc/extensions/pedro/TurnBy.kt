package dev.nextftc.extensions.pedro

import dev.nextftc.core.commands.Command
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.abs
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

class TurnBy(private val angle: Angle) : Command() {

    init {
        named("TurnBy($angle)")
    }

    override val isDone: Boolean
        get() = !follower.isTurning

    override fun start() {
        follower.turn(abs(angle).inRad, angle.sign > 0)
    }
}