// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;
// import the stuff 
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.motorcontrollers.SmartMotorController;


import org.team157.robot.Constants.FlywheelConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSystem extends SubsystemBase {

  private TalonFX motor  = new TalonFX(FlywheelConstants.MOTOR_ID);
  private static TalonFX motor_follower = new TalonFX(FlywheelConstants.MOTOR_ID_FOLLOWER);



  private SmartMotorControllerConfig flywheelSystemConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // feedback constant (pid constants)
    .withClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    // feedforward constants
    .withFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
    .withSimFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
    // telemetry name and verbosity level
    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
    // gearing from the motor rotor to final shaft
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(FlywheelConstants.GEARING)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withFollowers(Pair.of(motor_follower, false));
    // vendor motor controller object


  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),flywheelSystemConfig);
 // private SmartMotorController smartMotorFollower = new TalonFXWrapper(motor_follower, DCMotor.getKrakenX60(1),flywheelSystemConfig);

  private final FlyWheelConfig flyWheelConfig = new FlyWheelConfig(smartMotor)
  // diameter of the flywheel 
  .withDiameter(Inches.of(FlywheelConstants.FLYWHEEL_DIAMETER))
  // mass of the flywheel
  .withMass(Pounds.of(FlywheelConstants.FLYWHEEL_MASS))
  // maximum speed of the shooter
  .withUpperSoftLimit(RPM.of(FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER))
  // telemetry name and verbosity
  .withTelemetry("FlywheelDynamics", TelemetryVerbosity.HIGH);

  // flywheel mechanism
  private FlyWheel flyWheel = new FlyWheel(flyWheelConfig);


    /**
     * 
     * gets the current velocity of the flywheel
     * 
     * @return flywheel velocity
     */
  public AngularVelocity getVelocity() {return flyWheel.getSpeed();}

  /**
   * set the shooter velocity
   * @param speed speed to set
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity (AngularVelocity speed) {
    return flyWheel.setSpeed(speed); 
  }

  
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return flyWheel.set(dutyCycle);}

  /** Creates a new FlywheelSystem. */
  public FlywheelSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flyWheel.updateTelemetry();
  }

    @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    flyWheel.simIterate();
  }
}


