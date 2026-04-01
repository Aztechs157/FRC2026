package org.team157.robot.subsystems.uptake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Defines the input data to be logged by AdvantageKit, along with methods and
 * {@link Command}s which an implementation of this IO interface must have.
 */
public interface UptakeIO {

    /**
     * Represents the set of inputs which are to be logged by AdvantageKit
     * and updated by an implementation of the {@link UptakeIO} interface.
     */
    @AutoLog
    public static class UptakeIOInputs {
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double temperatureCelsius = 0.0;
        public double mechanismVelocityDegreesPerSecond = 0.0;
        public boolean uptakeRunning = false;
    }

    /**
     * Updates the inputs to be logged by AdvantageKit.
     *
     * @param inputs The set of inputs to be logged, including information on the motor and mechanism.
     */
    default void updateInputs(UptakeIOInputs inputs) {}

    /** Updates the values for the simulated version of the uptake roller mechanism. */
    default void simIterate() {}

    /**
     * Stops the uptake roller.
     * @return a {@link Command} setting the motor's output power to 0.
     */
    default Command stop() {
        return Commands.none();
    }

    /**
     * Directly sets the output duty cycle of the uptake roller motor.
     *
     * @param dutyCycle The duty cycle to apply to the motor, between -1 and 1.
     * @return a {@link Command} setting the motor's duty cycle to the specified value.
     */
    default Command set(double dutyCycle) {
        return Commands.none();
    }
}