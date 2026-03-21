// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;

import org.team157.robot.Constants;
import org.team157.robot.Constants.IntakeConstants;
import org.team157.robot.Constants.TelemetryConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class IntakeRollerSystem extends SubsystemBase {
  
    //////////////////////
   /// INTAKE ROLLER ///
  /////////////////////
  private TalonFX motor  = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, Constants.RIO_CAN_BUS);
  private TalonFX motor_follower = new TalonFX(IntakeConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);

  private SmartMotorControllerConfig intakeRollerMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
    .withGearing(1)
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withFollowers(Pair.of(motor_follower, true));

  // vendor motor controller object
  private SmartMotorController smartRollerMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), intakeRollerMotorConfig);

  private final FlyWheelConfig intakeRollerConfig = new FlyWheelConfig(smartRollerMotor)
  .withTelemetry("Intake", TelemetryConstants.TELEMETRY_VERBOSITY)
  .withDiameter(Inches.of(3))
  .withMass(Kilograms.of(0.5)); //TODO: measure mass of the intake roller and update this constant

  // flywheel mechanism
  private FlyWheel intakeRollers = new FlyWheel(intakeRollerConfig);


  /**
   * Set the dutycycle of the intake.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setRoller(double dutyCycle) {
    return intakeRollers.set(dutyCycle);
  }

  /** Run the intake at a set speed. Used for autonomous and button bindings. */
  public Command runIntake() {
    return setRoller(1);
  } 

  /** Creates a new IntakeSystem */
  public IntakeRollerSystem() {
    
  }

  
  public AngularVelocity getRollerVelocity() {
    return intakeRollers.getSpeed();
  }
  public boolean isIntakeRunning(){
    return !intakeRollers.gte(DegreesPerSecond.of(1)).getAsBoolean();
  }

  public Command setDefault() {
    return run(() -> {
      intakeRollers.setDutyCycleSetpoint(0);
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
    if(TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber("Intake Roller Velocity", getRollerVelocity().in(RPM));
      SmartDashboard.putBoolean("Intake Rollers Running", (isIntakeRunning()));
    }

    
    intakeRollers.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Updates the intake roller simulation's values,
    intakeRollers.simIterate();
  }

  



}
