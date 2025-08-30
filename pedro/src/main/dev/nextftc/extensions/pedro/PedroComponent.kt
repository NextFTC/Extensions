@file:JvmName("FollowerHolder")

package dev.nextftc.extensions.pedro

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.components.Component
import dev.nextftc.ftc.ActiveOpMode

class PedroComponent(private val followerFactory: (hardwareMap: HardwareMap) -> Follower) : Component {
    override fun preInit() {
        _follower = followerFactory(ActiveOpMode.hardwareMap)
    }

    override fun preWaitForStart() = follower.update()
    override fun preUpdate() = follower.update()

    override fun postStop() {
        _follower = null
    }
}

private var _follower: Follower? = null

@get:JvmName("follower")
val follower: Follower
    get() = _follower ?: error("Follower not initialized! Make sure you added PedroComponent to your OpMode.")