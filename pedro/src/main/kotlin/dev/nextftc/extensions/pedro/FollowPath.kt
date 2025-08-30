package dev.nextftc.extensions.pedro

import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroComponent.follower

class FollowPath @JvmOverloads constructor(
    private val path: PathChain,
    private val holdEnd: Boolean? = null,
    private val maxPower: Double? = null
) : Command() {

    init {
        require(maxPower == null || holdEnd != null) { "If maxPower is passed, holdEnd must be passed as well." }
    }

    @JvmOverloads
    constructor(path: Path, holdEnd: Boolean? = null, maxPower: Double? = null) : this(
        PathChain(path),
        holdEnd,
        maxPower
    )

    override val isDone: Boolean
        get() = !follower.isBusy

    override fun start() {
        if (holdEnd !== null && maxPower !== null) follower.followPath(path, maxPower, holdEnd)
        else if (holdEnd !== null) follower.followPath(path, holdEnd)
        else follower.followPath(path)
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) follower.breakFollowing()
    }
}