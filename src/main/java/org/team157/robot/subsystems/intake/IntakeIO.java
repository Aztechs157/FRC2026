package org.team157.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the input data to be logged by AdvantageKit, along with methods and {@link Command}s
 * which an implementation of this IO interface must have.
 */
public interface IntakeIO {

  /**
   * Represents the set of inputs which are to be logged by AdvantageKit and updated by an
   * implementation of the {@link HoodIO} interface.
   */
  @AutoLog
  public static class IntakeIOInputs {
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVolts = 0.0;
    public double temperatureCelsius = 0.0;
    public double mechanismVelocityDegreesPerSecond = 0.0;
    public boolean intakeRunning = false;
  }

  /**
   * Updates the inputs to be logged by AdvantageKit.
   *
   * @param inputs The set of inputs to be logged, including information on the motor, encoder, and
   *     mechanism.
   */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Updates the values for the simulated version of the intake roller mechanism. */
  default void simIterate() {}

  /**
   * Stops the intake roller.
   *
   * @return a {@link Command} setting the motor's output power to 0.
   */
  default Command stop() {
    return Commands.none();
  }

  /**
   * Directly sets the output power of the intake rollers' motors
   *
   * @param dutycycle Power to be applied to the motors, from 1 to -1.
   * @return a {@link Command} applying the specified power to the motors.
   */
  default Command set(double dutycycle) {
    return Commands.none();
  }
}
