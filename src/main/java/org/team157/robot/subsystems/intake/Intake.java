// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {}

  /**
   * Specifies the IO implementation to be used for the hood.
   * 
   * @param io An implementation of the Hood's IO layer, i.e. HoodIOTalonFX
   */
  public void setIO(IntakeIO io) {
    this.io = io;
  }

  /**
   * Sets the default command of the hood, stopping motor output when no other
   * commands are running.
   * 
   * @return Command setting the duty cycle output of the hood's motor to 0
   */
  public Command getDefault() {
    return io.stop();
  }

  /**
   * Set the duty cycle output of the intake motor.
   * Primarily used for manual control
   * 
   * @param dutycycle The power to be applied to the motor.
   */
  public Command set(double dutycycle) {
    return io.set(dutycycle);
  }

  /**
   * Run the intake at a set speed.
   * Used for autonomous and button bindings.
   * 
   * @return a {@link Command} running the intake motors at 100% duty cycle
   */
  public Command runIntake() {
    return set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.simIterate();
  }

}