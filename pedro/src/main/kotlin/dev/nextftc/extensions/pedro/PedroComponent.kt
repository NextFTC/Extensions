package dev.nextftc.extensions.pedro

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.Component
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.ActiveOpMode
import java.util.function.Supplier

class PedroComponent(private val followerFactory: (HardwareMap) -> Follower) : Component {

    override fun preInit() {
        _follower = followerFactory(ActiveOpMode.hardwareMap)
    }

    override fun preWaitForStart() = follower.update()
    override fun preUpdate() = follower.update()

    override fun postStop() {
        _follower = null
    }

    companion object {
        private var _follower: Follower? = null

        @get:JvmName("follower")
        @JvmStatic
        val follower: Follower
            get() = _follower ?: error("Follower not initialized! Make sure you added PedroComponent to your OpMode.")

        @get:JvmName("gyro")
        @JvmStatic
        val gyro: Supplier<Angle> = Supplier { follower.totalHeading.rad.normalized }

    }
}