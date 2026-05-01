package org.team157.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the Hopper subsystem, which feeds balls from the intake into the uptake for shooting.
 */
public class Hopper extends SubsystemBase {

  // The IO interface for interacting with the hopper's motor.
  private HopperIO io;

  // Inputs from the motor and mechanism, to be updated periodically and logged.
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  /** Creates a new Hopper. */
  public Hopper() {}
  /**
   * Specifies the IO implementation to be used for the hopper.
   *
   * @param io An implementation of the Hopper's IO layer, i.e. HopperIOTalonFX
   */
  public void setIO(HopperIO io) {
    this.io = io;
  }

  /**
   * Sets the default command of the hopper, stopping motor output when no other commands are
   * running.
   *
   * @return Command setting the duty cycle of the hopper's motor to 0
   */
  public Command getDefault() {
    return io.stop();
  }

  /**
   * Set the duty cycle output of the intake motor. Primarily used for manual control
   *
   * @param dutycycle The power to be applied to the motor.
   */
  public Command set(double dutycycle) {
    return io.set(dutycycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.simIterate();
  }
}
