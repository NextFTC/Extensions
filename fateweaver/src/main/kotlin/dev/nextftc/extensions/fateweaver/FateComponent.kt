package dev.nextftc.extensions.fateweaver

import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.Component
import gay.zharel.fateweaver.flight.FlightLogChannel
import gay.zharel.fateweaver.flight.FlightRecorder
import gay.zharel.fateweaver.log.LogChannel
import gay.zharel.fateweaver.schemas.FateSchema
import java.util.function.Supplier
import kotlin.reflect.KClass

/**
 * FateWeaver integration component for NextFTC.
 *
 * This component provides a high-level interface for logging telemetry data using FateWeaver's
 * flight recorder system.
 * It manages log channels, handles periodic data publishing, and
 * automatically captures command snapshots during robot operation.
 *
 * Most FateComponent calls forward to [FlightRecorder].
 * 
 * @see FlightRecorder
 */
object FateComponent : Component {
    /**
     * Internal channel used to log command manager snapshots.
     */
    private lateinit var snapshotChannel: FlightLogChannel<Array<String>>

    /**
     * Map of registered publishers that will be automatically invoked during periodic updates.
     * Each publisher supplies data to its associated log channel.
     */
    private val registeredPublishers: MutableMap<LogChannel<Any>, Supplier<Any>> = mutableMapOf()

    /**
     * Creates a new log channel with a custom schema.
     *
     * Forwards to [FlightRecorder.createChannel].
     *
     * @param T The type of data that will be logged to this channel
     * @param name The name of the channel
     * @param schema The schema defining how to serialize/deserialize the data
     * @return A new [FlightLogChannel] for logging data
     */
    @JvmStatic
    fun <T> createChannel(name: String, schema: FateSchema<T>) =
        FlightRecorder.createChannel(name, schema)

    /**
     * Creates a new log channel for a specific class type.
     *
     * Forwards to [FlightRecorder.createChannel].
     *
     * @param T The type of data that will be logged to this channel
     * @param name The name of the channel
     * @param cls The Java class representing the data type
     * @return A new [FlightLogChannel] for logging data
     */
    @JvmStatic
    fun <T: Any> createChannel(name: String, cls: Class<T>) =
        FlightRecorder.createChannel(name, cls)

    /**
     * Creates a new log channel for a specific class type.
     *
     * Forwards to [FlightRecorder.createChannel].
     *
     * @param T The type of data that will be logged to this channel
     * @param name The name of the channel
     * @param cls The Kotlin class representing the data type
     * @return A new [FlightLogChannel] for logging data
     */
    @JvmStatic
    fun <T: Any> createChannel(name: String, cls: KClass<T>) =
        FlightRecorder.createChannel(name, cls.java)

    /**
     * Writes data to a specific log channel.
     *
     * Forwards to [FlightRecorder.write].
     *
     * @param T The type of data being logged
     * @param channel The channel to write to
     * @param obj The data object to log
     */
    @JvmStatic
    fun <T> write(channel: LogChannel<T>, obj: T) =
        FlightRecorder.write(channel, obj)

    /**
     * Writes data to a channel by name.
     *
     * Forwards to [FlightRecorder.write].
     *
     * @param name The name of the channel to write to
     * @param obj The data object to log
     */
    @JvmStatic
    fun write(name: String, obj: Any) =
        FlightRecorder.write(name, obj)

    /**
     * Registers a publisher that will automatically supply data to a channel during periodic updates.
     *
     * The supplier will be invoked during [postWaitForStart] and [postUpdate] to retrieve
     * the latest data and write it to the channel.
     *
     * @param T The type of data being published
     * @param channel The channel to publish to
     * @param supplier A supplier that provides the data to log
     */
    @JvmStatic
    @Suppress("UNCHECKED_CAST")
    fun <T> registerPublisher(channel: LogChannel<T>, supplier: Supplier<T>) {
        registeredPublishers[channel as LogChannel<Any>] = supplier as Supplier<Any>
    }

    /**
     * Registers a publisher for a new channel with a custom schema.
     *
     * This convenience method creates a channel and registers a publisher in one call.
     *
     * @param T The type of data being published
     * @param name The name of the channel to create
     * @param schema The schema defining how to serialize/deserialize the data
     * @param supplier A supplier that provides the data to log
     */
    @JvmStatic
    fun <T> registerPublisher(name: String, schema: FateSchema<T>, supplier: Supplier<T>) {
        registerPublisher(createChannel(name, schema), supplier)
    }

    /**
     * Registers a publisher for a new channel with a Java class type.
     *
     * This convenience method creates a channel and registers a publisher in one call.
     *
     * @param T The type of data being published
     * @param name The name of the channel to create
     * @param cls The Java class representing the data type
     * @param supplier A supplier that provides the data to log
     */
    @JvmStatic
    fun <T: Any> registerPublisher(name: String, cls: Class<T>, supplier: Supplier<T>) {
        registerPublisher(createChannel(name, cls), supplier)
    }

    /**
     * Registers a publisher for a new channel with a Kotlin class type.
     *
     * This convenience method creates a channel and registers a publisher in one call.
     *
     * @param T The type of data being published
     * @param name The name of the channel to create
     * @param cls The Kotlin class representing the data type
     * @param supplier A supplier that provides the data to log
     */
    @JvmStatic
    fun <T: Any> registerPublisher(name: String, cls: KClass<T>, supplier: Supplier<T>) {
        registerPublisher(createChannel(name, cls), supplier)
    }

    override fun preInit() {
        snapshotChannel = createChannel("CommandSnapshot", Array<String>::class)
    }

    override fun postWaitForStart() {
        snapshotChannel.put(CommandManager.snapshot.toTypedArray())
        registeredPublishers.forEach { (channel, supplier) ->
            write(channel, supplier.get())
        }
        FlightRecorder.timestamp()
    }

    override fun postUpdate() {
        snapshotChannel.put(CommandManager.snapshot.toTypedArray())
        registeredPublishers.forEach { (channel, supplier) ->
            write(channel, supplier.get())
        }
        FlightRecorder.timestamp()
    }

    override fun postStop() {
        registeredPublishers.clear()
    }
}