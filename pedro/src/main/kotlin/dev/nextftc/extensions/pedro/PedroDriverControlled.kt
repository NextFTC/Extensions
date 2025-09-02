package dev.nextftc.extensions.pedro

import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.hardware.driving.DriverControlledCommand
import java.util.function.Supplier

class PedroDriverControlled @JvmOverloads constructor(
    drivePower: Supplier<Double>,
    strafePower: Supplier<Double>,
    turnPower: Supplier<Double>,
    private val robotCentric: Boolean = true
) : DriverControlledCommand(drivePower, strafePower, turnPower) {

    override fun start() {
        follower.startTeleopDrive()
    }

    override fun calculateAndSetPowers(powers: DoubleArray) {
        val (drive, strafe, turn) = powers
        follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) follower.breakFollowing()
    }
}