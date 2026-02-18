// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import org.team157.robot.Constants;
import org.team157.robot.Constants.HopperConstants;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HopperSystem extends SubsystemBase {
    //////////////////////
   /// HOPPER ROLLER ///
  /////////////////////
  private TalonFX rollerMotor  = new TalonFX(HopperConstants.MOTOR_ID, Constants.RIO_CAN_BUS);

  private SmartMotorControllerConfig hopperRollerMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("HopperRollerMotor", TelemetryVerbosity.HIGH)
    .withGearing(1)
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(HopperConstants.CURRENT_LIMIT));

  // vendor motor controller object
  private SmartMotorController smartRollerMotor = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX44(1), hopperRollerMotorConfig);

  private final FlyWheelConfig hopperRollerConfig = new FlyWheelConfig(smartRollerMotor)
  .withTelemetry("Hopper", TelemetryVerbosity.HIGH);

  // flywheel mechanism
  private FlyWheel hopperRollers = new FlyWheel(hopperRollerConfig);

  /**
   * Set the dutycycle of the hopper.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setRoller(double dutyCycle) {
    return hopperRollers.set(dutyCycle);
  }

  /** Creates a new HopperSystem */
  public HopperSystem() {
    
  }

  public Command setDefault() {
    // return setRoller(0).
    return run(() -> {
      // hopperPivot.setDutyCycleSetpoint(0);
      hopperRollers.setDutyCycleSetpoint(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Send values to NT to display on Elastic.
    /*  TODO: look into SmartDashboard alternatives, as it's deprecated, 
     *  marked for removal along with Shuffleboard for next season.
     *  Consider publishing to NT directly.
     */
    hopperRollers.updateTelemetry();
  }

}
