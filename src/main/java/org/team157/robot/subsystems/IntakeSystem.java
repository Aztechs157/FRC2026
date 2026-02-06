// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants.HoodConstants;
import org.team157.robot.Constants.IntakeConstants;
import org.team157.robot.Constants.UptakeConstants;
import org.team157.robot.generated.TunerConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSystem extends SubsystemBase {
  private TalonFX rollerMotor  = new TalonFX(IntakeConstants.MOTOR_ID);

  private SmartMotorControllerConfig intakeSystemConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
    .withGearing(IntakeConstants.GEARING)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40));

  // vendor motor controller object
  private SmartMotorController smartRollerMotor = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX44(1), intakeSystemConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig(smartRollerMotor)
  .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  // flywheel mechanism
  private FlyWheel intake = new FlyWheel(intakeConfig);

  /**
   * Set the dutycycle of the intake.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setRoller(double dutyCycle) {
    return intake.set(dutyCycle);
  }

  /** Creates a new IntakeSystem */
  public IntakeSystem() {}

  ////////////////////////////////////////////////////////
  /// INTAKE PIVOT
  ///////////////////////////////////////////////////////
   private TalonFX pivotMotor = new TalonFX(HoodConstants.MOTOR_ID, TunerConstants.kCANBus);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(HoodConstants.ENCODER_ID);

  // Configure the hood motor controller for use with YAMS.
  private SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90)) //TODO: tune this PID
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 5)))
      .withTelemetry("Hood Motor", TelemetryVerbosity.HIGH) 
      .withStatorCurrentLimit(Amps.of(30))
      .withClosedLoopRampRate(Seconds.of(0.25));
      // .withExternalEncoder(encoder)
      // .withExternalEncoderInverted(false)
      // .withExternalEncoderGearing(1)
      // .withExternalEncoderZeroOffset(Degrees.of(180))
      // .withUseExternalFeedbackEncoder(true);
      // TODO: add .withMOI() for simulation

  // Create the hood's motor controller with the above configuration.
  private SmartMotorController smartPivotMotor = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX44(1), hoodMotorConfig);

  // Configure the physical characteristics of the hood.
  private PivotConfig hoodConfig= new PivotConfig(smartPivotMotor)
      .withStartingPosition(Degrees.of(0))
      .withWrapping(Degrees.of(-180), Degrees.of(180))
      .withHardLimit(Degrees.of(-135), Degrees.of(135))
      .withSoftLimits(Degrees.of(-120), Degrees.of(120))
      .withTelemetry("Hood", TelemetryVerbosity.HIGH);

  // Create the hood pivot system with the above configuration.
  private Pivot hood = new Pivot(hoodConfig);
  
  /**
   * Set the target angle of the hood.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) { 
    return hood.setAngle(angle);
  }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command setPivot(double dutycycle) { 
    return hood.set(dutycycle);
  }

  /**
   * Run sysId on the {@link HoodSystem}.
   */
  public Command sysId() { 
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /**
   * Set the duty cycle output of the hood motor.
   * Primarily used for manual control
   * @param power The power to be applied to the motor.
   */
  public void runMotor(double power) {
    smartPivotMotor.setDutyCycle(power);
  }

  // This is not used currently.
  public void runWithLimits() {
    smartPivotMotor.setDutyCycle(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
  }

  /**
   * Get the raw position of the hood's encoder.
   * @return The current position of the hood in encoder rotations.
   */
  public double getPos() {
    return encoder.get();
  }

  /**
   * Get the scaled position of the hood from 0 to 1.
   * @return The position of the hood scaled from 0 to 1.
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), IntakeConstants.MIN_POSITION, IntakeConstants.MAX_POSITION, 0.0,
        1.0);
  }

  /**
   * Get the current angle of the hood, based on the YAMS pivot system.
   * @return The angle of the hood, in degrees, from -180 to 180, using the YAMS pivot system.
   */
  public double getScaledPosAngleYAMS() {
    return hood.getAngle().in(Degrees);
  }
  /**
   * Get the current angle of the hood, directly from the encoder value.
   * @return The angle of the hood, in degrees, from -180 to 180, using the encoder directly.
   */
  public double getScaledPosAngleEncoder() {
    return PosUtils.mapRange(getPos(), IntakeConstants.MIN_POSITION, IntakeConstants.MAX_POSITION, -135,
        135);
  }

  /**
   * Get the current velocity of the hood.
   * @return The velocity of the hood, in degrees per second.
   */
  public double getVelocity() {
    return smartPivotMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  public Command setDefault() {
    return setRoller(0).alongWith(setPivot(0));
  }

}
