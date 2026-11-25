package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Ivy has no Subsystem class: any object can be a command requirement.
 * This interface just gives every mechanism the same lifecycle hooks so {@link org.firstinspires.ftc.teamcode.Robot}
 * can loop over them.
 */
public interface Subsystem {
    /**
     * Called once at INIT, after the hardware is set up. Only servos may move here (to hold the
     * starting configuration); motors wait for {@link #start()}.
     */
    default void init() {}

    /** Called once when the driver presses START. Put "move to a safe starting state" code here. */
    default void start() {}

    /** Called every loop after {@code Scheduler.execute()}. */
    default void periodic() {}

    /** Called when the OpMode ends. Turn everything off. */
    default void stop() {}

    /**
     * Register actions that Turtle Tracer event markers can call by name.
     * Names must match the marker names you type in Turtle Tracer exactly.
     */
    default void registerNamedCommands() {}

    default void telemetry(Telemetry telemetry) {}
}
