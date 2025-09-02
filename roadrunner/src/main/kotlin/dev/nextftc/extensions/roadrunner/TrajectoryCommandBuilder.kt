package dev.nextftc.extensions.roadrunner

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.IdentityPoseMap
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseMap
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Rotation2dDual
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.map
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.NullCommand
import kotlin.time.Duration
import kotlin.time.DurationUnit

/**
 * Builder that combines trajectories, turns, and other actions.
 */
class TrajectoryCommandBuilder private constructor(
    // constants
    val turnCommandFactory: TurnCommandFactory,
    val trajectoryCommandFactory: TrajectoryCommandFactory,
    val trajectoryBuilderParams: TrajectoryBuilderParams,
    val beginEndVel: Double,
    val baseTurnConstraints: TurnConstraints,
    val baseVelConstraint: VelConstraint,
    val baseAccelConstraint: AccelConstraint,
    val poseMap: PoseMap,
    // vary throughout
    private val tb: TrajectoryBuilder,
    private val n: Int,
    // lastPose, lastTangent are post-mapped
    private val lastPoseUnmapped: Pose2d,
    private val lastPose: Pose2d,
    private val lastTangent: Rotation2d,
    private val ms: List<MarkerFactory>,
    private val cont: (Command) -> Command,
) {
    @JvmOverloads
    constructor(
        turnCommandFactory: TurnCommandFactory,
        trajectoryCommandFactory: TrajectoryCommandFactory,
        trajectoryBuilderParams: TrajectoryBuilderParams,
        beginPose: Pose2d,
        beginEndVel: Double,
        baseTurnConstraints: TurnConstraints,
        baseVelConstraint: VelConstraint,
        baseAccelConstraint: AccelConstraint,
        poseMap: PoseMap = IdentityPoseMap(),
    ) :
            this(
                turnCommandFactory,
                trajectoryCommandFactory,
                trajectoryBuilderParams,
                beginEndVel,
                baseTurnConstraints,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap,
                TrajectoryBuilder(
                    trajectoryBuilderParams,
                    beginPose, beginEndVel,
                    baseVelConstraint, baseAccelConstraint,
                    poseMap,
                ),
                0,
                beginPose,
                poseMap.map(beginPose),
                poseMap.map(beginPose).heading,
                emptyList(),
                { it },
            )

    private constructor(
        ab: TrajectoryCommandBuilder,
        tb: TrajectoryBuilder,
        n: Int,
        lastPoseUnmapped: Pose2d,
        lastPose: Pose2d,
        lastTangent: Rotation2d,
        ms: List<MarkerFactory>,
        cont: (Command) -> Command,
    ) :
            this(
                ab.turnCommandFactory,
                ab.trajectoryCommandFactory,
                ab.trajectoryBuilderParams,
                ab.beginEndVel,
                ab.baseTurnConstraints,
                ab.baseVelConstraint,
                ab.baseAccelConstraint,
                ab.poseMap,
                tb,
                n,
                lastPoseUnmapped,
                lastPose,
                lastTangent,
                ms,
                cont
            )

    /**
     * Ends the current trajectory in progress. No-op if no trajectory segments are pending.
     */
    fun endTrajectory() =
        if (n == 0) {
            require(ms.isEmpty()) { "Cannot end trajectory with pending markers" }

            this
        } else {
            val ts = tb.build()
            val endPoseUnmapped = ts.last().path.basePath.end(1).value()
            val end = ts.last().path.end(2)
            val endPose = end.value()
            val endTangent = end.velocity().value().linearVel.angleCast()
            TrajectoryCommandBuilder(
                this,
                TrajectoryBuilder(
                    trajectoryBuilderParams,
                    endPoseUnmapped,
                    beginEndVel,
                    baseVelConstraint,
                    baseAccelConstraint,
                    poseMap,
                ),
                0,
                endPoseUnmapped,
                endPose,
                endTangent,
                emptyList()
            ) { tail ->
                val (aNew, msRem) = ts.zip(ts.scan(0) { acc, t -> acc + t.offsets.size - 1 }).foldRight(
                    Pair(tail, ms)
                ) { (traj, offset), (acc, ms) ->
                    val timeTraj = TimeTrajectory(traj)
                    val actions = mutableListOf(seqCons(trajectoryCommandFactory.make(timeTraj), acc))
                    val msRem = mutableListOf<MarkerFactory>()
                    for (m in ms) {
                        val i = m.segmentIndex - offset
                        if (i >= 0) {
                            actions.add(m.make(timeTraj, traj.offsets[i]))
                        } else {
                            msRem.add(m)
                        }
                    }

                    when (actions.size) {
                        0 -> Pair(NullCommand(), msRem)
                        1 -> Pair(actions.first(), msRem)
                        else -> Pair(ParallelGroup(*actions.toTypedArray()), msRem)
                    }
                }

                require(msRem.isEmpty()) { "Unresolved markers" }

                cont(aNew)
            }
        }

    /**
     * Stops the current trajectory (like [endTrajectory]) and adds command [command] next.
     */
    fun stopAndAdd(command: Command): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return TrajectoryCommandBuilder(b, b.tb, b.n, b.lastPoseUnmapped, b.lastPose, b.lastTangent, b.ms) { tail ->
            b.cont(seqCons(command, tail))
        }
    }
    fun stopAndAdd(f: Runnable) = stopAndAdd(InstantCommand("StopAndAdd", f))

    /**
     * Waits [t] seconds.
     */
    fun waitSeconds(t: Double): TrajectoryCommandBuilder {
        require(t >= 0.0) { "Time ($t) must be non-negative" }

        return stopAndAdd(Delay(t))
    }

    /**
     * Waits for the duration of [t].
     */
    fun waitSeconds(t: Duration): TrajectoryCommandBuilder {
        require(t.isPositive()) { "Time ($t) must be non-negative" }

        return stopAndAdd(Delay(t))
    }

    /**
     * Schedules command [command] to execute in parallel starting at a displacement [disp] after the last trajectory segment.
     * The command start is clamped to the span of the current trajectory.
     *
     * Cannot be called without an applicable pending trajectory.
     */
    // TODO: Should calling this without an applicable trajectory implicitly begin an empty trajectory and execute the
    // action immediately?
    fun afterDisplacement(disp: Double, command: Command): TrajectoryCommandBuilder {
        require(disp >= 0.0) { "Displacement ($disp) must be non-negative" }

        return TrajectoryCommandBuilder(
            this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
            ms + listOf(DispMarkerFactory(n, disp, command)), cont
        )
    }
    fun afterDisplacement(disp: Double, f: Runnable) = afterDisplacement(disp, InstantCommand("DispMarker", f))

    /**
     * Schedules command [command] to execute in parallel starting [duration] seconds after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(duration: Double, command: Command): TrajectoryCommandBuilder {
        require(duration >= 0.0) { "Time ($duration) must be non-negative" }

        return if (n == 0) {
            TrajectoryCommandBuilder(this, tb, 0, lastPoseUnmapped, lastPose, lastTangent, emptyList()) { tail ->
                val m = seqCons(Delay(duration), command)
                if (tail is NullCommand) {
                    cont(m)
                } else {
                    cont(ParallelGroup(tail, m))
                }
            }
        } else {
            TrajectoryCommandBuilder(
                this, tb, n, lastPoseUnmapped, lastPose, lastTangent,
                ms + listOf(TimeMarkerFactory(n, duration, command)), cont
            )
        }
    }
    fun afterTime(duration: Double, f: Runnable) = afterTime(duration, InstantCommand("TimeMarker", f))


    /**
     * Schedules command [command] to execute in parallel starting [duration] after the last trajectory segment, turn, or
     * other action.
     */
    fun afterTime(duration: Duration, command: Command) = afterTime(duration.toDouble(DurationUnit.SECONDS), command)
    fun afterTime(duration: Duration, f: Runnable) = afterTime(duration, InstantCommand("TimeMarker", f))

    fun setTangent(r: Rotation2d) =
        TrajectoryCommandBuilder(this, tb.setTangent(r), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)
    fun setTangent(r: Double) = setTangent(Rotation2d.Companion.exp(r))

    fun setReversed(reversed: Boolean) =
        TrajectoryCommandBuilder(this, tb.setReversed(reversed), n, lastPoseUnmapped, lastPose, lastTangent, ms, cont)

    @JvmOverloads
    fun turn(angle: Double, turnConstraintsOverride: TurnConstraints? = null): TrajectoryCommandBuilder {
        val b = endTrajectory()
        val mappedAngle =
            poseMap.map(
                Pose2dDual(
                    Vector2dDual.Companion.constant(b.lastPose.position, 2),
                    Rotation2dDual.Companion.constant<Arclength>(b.lastPose.heading, 2) + DualNum(listOf(0.0, angle))
                )
            ).heading.velocity().value()
        val b2 = b.stopAndAdd(
            turnCommandFactory.make(
                TimeTurn(b.lastPose, mappedAngle, turnConstraintsOverride ?: baseTurnConstraints)
            )
        )
        val lastPoseUnmapped = Pose2d(b2.lastPoseUnmapped.position, b2.lastPoseUnmapped.heading + angle)
        val lastPose = Pose2d(b2.lastPose.position, b2.lastPose.heading + mappedAngle)
        val lastTangent = b2.lastTangent + mappedAngle
        return TrajectoryCommandBuilder(
            b2,
            TrajectoryBuilder(
                trajectoryBuilderParams,
                lastPoseUnmapped,
                beginEndVel,
                baseVelConstraint,
                baseAccelConstraint,
                poseMap
            ),
            b2.n, lastPoseUnmapped, lastPose, lastTangent, b2.ms, b2.cont
        )
    }
    @JvmOverloads
    fun turnTo(heading: Rotation2d, turnConstraintsOverride: TurnConstraints? = null): TrajectoryCommandBuilder {
        val b = endTrajectory()
        return b.turn(heading - b.lastPose.heading, turnConstraintsOverride)
    }
    @JvmOverloads
    fun turnTo(heading: Double, turnConstraintsOverride: TurnConstraints? = null) =
        turnTo(Rotation2d.Companion.exp(heading), turnConstraintsOverride)

    @JvmOverloads
    fun lineToX(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToX(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXConstantHeading(
        posX: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXConstantHeading(
            posX, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXLinearHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXLinearHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToXSplineHeading(
        posX: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToXSplineHeading(
            posX, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToY(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToY(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYConstantHeading(
        posY: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYConstantHeading(
            posY, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYLinearHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYLinearHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun lineToYSplineHeading(
        posY: Double,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.lineToYSplineHeading(
            posY, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeTo(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeTo(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToConstantHeading(
        pos: Vector2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToConstantHeading(
            pos, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun strafeToLinearHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToLinearHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun strafeToSplineHeading(
        pos: Vector2d,
        heading: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.strafeToSplineHeading(
            pos, heading, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineTo(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineTo(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToConstantHeading(
        pos: Vector2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToConstantHeading(
            pos, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToLinearHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToLinearHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Rotation2d,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )
    @JvmOverloads
    fun splineToSplineHeading(
        pose: Pose2d,
        tangent: Double,
        velConstraintOverride: VelConstraint? = null,
        accelConstraintOverride: AccelConstraint? = null
    ) = TrajectoryCommandBuilder(
        this,
        tb.splineToSplineHeading(
            pose, tangent, velConstraintOverride, accelConstraintOverride
        ),
        n + 1, lastPoseUnmapped, lastPose, lastTangent, ms, cont
    )

    /**
     * Creates a new builder with the same settings at the current pose, tangent.
     */
    fun fresh() = endTrajectory().let {
        TrajectoryCommandBuilder(
            it.turnCommandFactory,
            it.trajectoryCommandFactory,
            it.trajectoryBuilderParams,
            it.lastPoseUnmapped,
            it.beginEndVel, it.baseTurnConstraints, it.baseVelConstraint, it.baseAccelConstraint,
            it.poseMap
        ).setTangent(it.lastTangent)
    }

    /**
     * Returns the built command.
     * This command is not a [FollowTrajectory],
     * but a [ParallelGroup] of [FollowTrajectory]s.
     */
    fun build(): Command {
        return endTrajectory().cont(NullCommand())
    }
}