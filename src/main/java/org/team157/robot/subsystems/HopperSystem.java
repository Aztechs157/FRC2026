// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;

import org.team157.robot.Constants.HopperConstants;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;

public class HopperSystem extends SubsystemBase {

  private TalonFX motor  = new TalonFX(HopperConstants.MOTOR_ID);

  private SmartMotorControllerConfig hopperSystemConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40));

  // vendor motor controller object
  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),hopperSystemConfig);

  private final FlyWheelConfig hopperConfig = new FlyWheelConfig(smartMotor)
  .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  // flywheel mechanism
  private FlyWheel hopper = new FlyWheel(hopperConfig);

  /**
   * Set the dutycycle of the hopper
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return hopper.set(dutyCycle);
  }

  /** Creates a new HopperSystem */
  public HopperSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hopper.updateTelemetry();
  }

    @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    hopper.simIterate();
  }
}