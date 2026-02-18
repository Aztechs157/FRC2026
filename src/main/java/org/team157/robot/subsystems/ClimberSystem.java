
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Constants.ClimberConstants;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimberSystem extends SubsystemBase {

  private SmartMotorControllerConfig climberMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
    //.withMechanismCircumference(Meters.of(Inches.of(ClimberConstants.DRUM_RADIUS).in(Meters) * 22))
    // Feedback Constants (PID Constants)
    .withClosedLoopController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD, MetersPerSecond.of(ClimberConstants.MAX_VELOCITY), MetersPerSecondPerSecond.of(ClimberConstants.MAX_ACCELERATION))
    .withSimClosedLoopController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD, MetersPerSecond.of(ClimberConstants.MAX_VELOCITY), MetersPerSecondPerSecond.of(ClimberConstants.MAX_ACCELERATION))
    // Feedforward Constants
    // .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    // .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    // Telemetry name and verbosity level
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
    // Gearing from the motor rotor to final shaft.
    .withWheelRadius(Inches.of(ClimberConstants.DRUM_RADIUS))

    .withGearing(ClimberConstants.GEARING)
    // Motor properties to prevent over currenting.
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(ClimberConstants.RAMP_RATE))
    .withOpenLoopRampRate(Seconds.of(ClimberConstants.RAMP_RATE));

  // Vendor motor controller object
  private TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID, Constants.RIO_CAN_BUS);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), climberMotorConfig);

  private ElevatorConfig climberConfig = new ElevatorConfig(smartMotor)
      .withStartingHeight(Meters.of(ClimberConstants.STARTING_HEIGHT))
      .withHardLimits(Meters.of(ClimberConstants.LOWER_HARD_LIMIT), Meters.of(ClimberConstants.UPPER_HARD_LIMIT))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(1))
      .withDrumRadius(Inches.of(ClimberConstants.DRUM_RADIUS));
    

  // Elevator Mechanism
  private Elevator elevator = new Elevator(climberConfig);

  /**
   * Set the height of the elevator and does not end the command when reached.
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) { return elevator.run(height);}
  
  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop controller.
   * @param angle Distance to go to.
   * @return A Command
   */
  //public Command setHeightAndStop(Distance height) { return elevator.runTo(height);}
  
  /**
   * Set the elevators closed loop controller setpoint.
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) { elevator.setMeasurementPositionSetpoint(height);}

  /**
   * Move the elevator up and down.
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) { return elevator.set(dutycycle);}

  /**
   * Run sysId on the {@link Elevator}
   */
  public Command sysId() { return elevator.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  /** Creates a new ExampleSubsystem. */
  public ClimberSystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    elevator.simIterate();
  }
}
