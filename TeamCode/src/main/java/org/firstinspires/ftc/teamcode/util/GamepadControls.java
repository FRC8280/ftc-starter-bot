package org.firstinspires.ftc.teamcode.util;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * One place to define every driver control. Ivy has no gamepad triggers, so this
 * covers the two kinds a TeleOp needs:
 *
 * <ul>
 *   <li>{@link #whileActive}: a command that runs every loop for the whole match,
 *       for sticks, analog triggers and held buttons (drive, intake, launcher).</li>
 *   <li>{@link #onPress}: schedule a command on the frame a button goes from released
 *       to pressed (re-set field-centric forward).</li>
 * </ul>
 *
 * <pre>
 * GamepadControls controls = new GamepadControls()
 *         .whileActive(robot.drive.teleopDrive(() -> -gamepad1.left_stick_y, () -> -gamepad1.left_stick_x, () -> -gamepad1.right_stick_x))
 *         .onPress(() -> gamepad1.options, robot.drive::setDriverForward);
 * controls.schedule(); // after START
 * </pre>
 *
 * The onPress factory is called on every press, so each press gets a fresh command
 * (that matters for commands with timers).
 */
public class GamepadControls {
    private static class Binding {
        final BooleanSupplier button;
        final Supplier<Command> command;
        boolean wasPressed = false;

        Binding(BooleanSupplier button, Supplier<Command> command) {
            this.button = button;
            this.command = command;
        }
    }

    private final List<Command> continuous = new ArrayList<>();
    private final List<Binding> bindings = new ArrayList<>();

    /** A command that runs for the whole TeleOp, reading its stick or trigger every loop. */
    public GamepadControls whileActive(Command command) {
        continuous.add(command);
        return this;
    }

    public GamepadControls onPress(BooleanSupplier button, Supplier<Command> command) {
        bindings.add(new Binding(button, command));
        return this;
    }

    /**
     * Turn an analog trigger into a button with two thresholds: pressed above {@code pressAt},
     * released only below {@code releaseAt}. With one threshold, a trigger held near it
     * flickers between pressed and released and counts as several presses.
     */
    public static BooleanSupplier trigger(DoubleSupplier value, double pressAt, double releaseAt) {
        boolean[] pressed = {false};
        return () -> {
            double v = value.getAsDouble();
            if (v >= pressAt) {
                pressed[0] = true;
            } else if (v <= releaseAt) {
                pressed[0] = false;
            }
            return pressed[0];
        };
    }

    /** Start every control. Call once, after START. */
    public void schedule() {
        // A button already held at START is not a press: it has to be released and pressed again.
        for (Binding binding : bindings) {
            binding.wasPressed = binding.button.getAsBoolean();
        }
        for (Command command : continuous) {
            Scheduler.schedule(command);
        }
        Scheduler.schedule(infinite(this::checkButtons));
    }

    private void checkButtons() {
        for (Binding binding : bindings) {
            boolean pressed = binding.button.getAsBoolean();
            if (pressed && !binding.wasPressed) {
                Scheduler.schedule(binding.command.get());
            }
            binding.wasPressed = pressed;
        }
    }
}
