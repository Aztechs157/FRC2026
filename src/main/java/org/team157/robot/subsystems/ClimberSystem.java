// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.team157.robot.Constants;
import org.team157.robot.Constants.ClimberConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimberSystem extends SubsystemBase {
  /** Creates a new ClimberSystem. */
  public ClimberSystem() {
TalonFX motor = new TalonFX(ClimberConstants.MOTOR_ID, Constants.RIO_CAN_BUS);


    
    
  SmartMotorControllerConfig climberMotorConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Mechanism Circumference is the distance traveled by each mechanism rotation converting rotations to meters.
  .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
  // Feedback Constants (PID Constants)
  .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
  // Feedforward Constants
  .withFeedforward(new ElevatorFeedforward(0, 0, 0))
  .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

// Vendor motor controller object
SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), climberMotorConfig);


ElevatorConfig climberConfig = new ElevatorConfig(smartMotor)
      .withStartingHeight(Meters.of(0.5)) // Starting height of the Elevator
      .withHardLimits(Meters.of(0), Meters.of(3)) // Hard limits defined
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH) // Telemetry Name
      .withMass(Pounds.of(16)); // Mass of the carraige
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
