package org.team157.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the input data to be logged by AdvantageKit, along with methods and {@link Command}s
 * which an implementation of this IO interface must have.
 */
public interface FlywheelIO {

  /**
   * Represents the set of inputs which are to be logged by AdvantageKit and updated by an
   * implementation of the {@link FlywheelIO} interface.
   */
  @AutoLog
  public static class FlywheelIOInputs {
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double temperatureCelsius = 0.0;
    public double mechanismVelocityRPM = 0.0;
    public double targetVelocityRPM = 0.0;
  }

  /**
   * Updates the inputs to be logged by AdvantageKit.
   *
   * @param inputs The set of inputs to be logged, including information on the motor and mechanism.
   * @param flywheelSetpoint The current target angular velocity of the flywheel, used for logging
   *     the error between the target and actual velocities.
   */
  default void updateInputs(FlywheelIOInputs inputs, AngularVelocity flywheelSetpoint) {}

  /** Updates the values for the simulated version of the flywheel mechanism. */
  default void simIterate() {}

  /** Stops the flywheel. */
  default void stop() {}

  /**
   * Directly sets the output duty cycle of the flywheel motor.
   *
   * @param dutyCycle The duty cycle to apply to the motor, between -1 and 1.
   * @return a {@link Command} setting the motor's duty cycle to the specified value.
   */
  default Command set(double dutyCycle) {
    return Commands.none();
  }

  /**
   * Sets the flywheel to a fixed target angular velocity.
   *
   * @param velocity The target angular velocity.
   * @return a {@link Command} setting the flywheel to the specified velocity.
   */
  default Command setVelocity(AngularVelocity velocity) {
    return Commands.none();
  }

  /**
   * Sets the flywheel to a dynamically-supplied target angular velocity.
   *
   * @param velocity A {@link Supplier} providing the target angular velocity.
   * @return a {@link Command} continuously updating the flywheel velocity from the supplier.
   */
  default Command setVelocity(Supplier<AngularVelocity> velocity) {
    return Commands.none();
  }
}
