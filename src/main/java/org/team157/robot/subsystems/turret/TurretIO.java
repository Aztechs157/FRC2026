package org.team157.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

/**
 * Defines the input data to be logged by AdvantageKit, along with methods and
 * {@link Command}s which an implementation of this IO interface must have.
 */
public interface TurretIO {

    /**
     * Represents the set of inputs which are to be logged by AdvantageKit
     * and updated by an implementation of the {@link TurretIO} interface.
     */
    @AutoLog
    public static class TurretIOInputs {
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetAngleDegrees = 0.0;
        public double angleDegrees = 0.0;
        public double angleFromEncoderDegrees = 0.0;
        public double scaledEncoderPosition = 0.0;
        public double encoderPositionRotations = 0.0;
        public double mechanismVelocityDegreesPerSecond = 0.0;
    }

    /**
     * Updates the inputs to be logged by AdvantageKit.
     *
     * @param inputs The set of inputs to be logged, including information on the motor, encoder, and mechanism.
     */
    default void updateInputs(TurretIOInputs inputs) {}

    /** Updates the values for the simulated version of the turret mechanism. */
    default void simIterate() {}

    /**
     * Stops the turret.
     */
    default void stop() {}

    /**
     * Sets the target angle of the turret mechanism.
     *
     * @param angle Angle to go to.
     * @return a {@link Command} setting the target angle of the turret to the specified angle.
     */
    default Command setTargetAngle(Angle angle) {
        return Commands.none();
    }

    /**
     * Sets the target angle of the turret mechanism, using an Angle Supplier.
     *
     * @param angle Supplier of the angle to go to.
     * @return a {@link Command} setting the target angle of the turret to the supplied angle.
     */
    default Command setTargetAngle(Supplier<Angle> angle) {
        return Commands.none();
    }

    /**
     * Directly sets the output duty cycle of the turret motor.
     *
     * @param dutyCycle The duty cycle to apply to the motor, between -1 and 1.
     * @return a {@link Command} setting the motor's duty cycle to the specified value.
     */
    default Command set(double dutyCycle) {
        return Commands.none();
    }

    /**
     * Gets the raw position of the turret's absolute encoder.
     *
     * @return The current position of the turret's encoder in rotations.
     */
    default double getEncoderPosition() {
        return 0.0;
    }
}
