// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Constants.HoodConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.robot.subsystems.FlywheelSystem;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSystem extends SubsystemBase {

  ////////////////////
  ///  HOOD PIVOT ///
  ///////////////////
  private TalonFX motor = new TalonFX(HoodConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(HoodConstants.ENCODER_ID);

  // Configure the hood motor controller for use with YAMS.
  private SmartMotorControllerConfig hoodPivotMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(HoodConstants.KP, HoodConstants.KI, HoodConstants.KD, HoodConstants.ANGULAR_VELOCITY,
          HoodConstants.ANGULAR_ACCELERATION)
      .withSimClosedLoopController(HoodConstants.SIM_KP, HoodConstants.SIM_KI, HoodConstants.SIM_KD,
          HoodConstants.ANGULAR_VELOCITY, HoodConstants.ANGULAR_ACCELERATION)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withGearing(HoodConstants.GEARING)
      .withTelemetry("Hood Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withSoftLimit((HoodConstants.LOWER_SOFT_LIMIT), (HoodConstants.UPPER_SOFT_LIMIT))
      .withStatorCurrentLimit(HoodConstants.CURRENT_LIMIT)
      .withClosedLoopRampRate((HoodConstants.RAMP_RATE));

  // Create the hood's motor controller with the above configuration.
  private SmartMotorController smartHoodMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1),
      hoodPivotMotorConfig);

  // Configure the physical characteristics of the hood.
  private PivotConfig hoodConfig = new PivotConfig(smartHoodMotor)
      .withStartingPosition(Degrees.of(getScaledPosAngleEncoder()))
      .withHardLimit((HoodConstants.LOWER_HARD_LIMIT), (HoodConstants.UPPER_HARD_LIMIT))
      .withTelemetry("Hood", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withMOI(Meters.of(0.2), Kilograms.of(0.5));

  // Create the hood pivot system with the above configuration.
  private Pivot hood = new Pivot(hoodConfig);

  /** Creates a new HoodSystem */
  public HoodSystem() {
    var configurator = motor.getConfigurator();
    configurator
        .refresh(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true));

  }

  public Command setDefault() {
    // return setRoller(0).
    return set(0);
  }

  /**
   * Set the target angle of the hood.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return hood.setAngle(angle).finallyDo(() -> hood.setDutyCycleSetpoint(0));
  }

  public Command setAngleThenStop(Angle angle) {
    return setAngle(angle)
        .until(() -> PosUtils.isOscillating(angle.in(Degrees), hood.getAngle().in(Degrees), 3.0, 0.0, 1.0));

  }

  /**
   * Move the hood up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the hood too.
   */
  public Command set(double dutycycle) {
    return hood.set(dutycycle);
  }

  public Command setDynamicHoodAngle() {
    return hood.setAngle(FlywheelSystem::getDesiredHoodAngle);
  }

  /**
   * Run sysId on the {@link HoodSystem}.
   * Default code from YAMS Template
   */
  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /**
   * Set the duty cycle output of the hood motor.
   * Primarily used for manual control
   * 
   * @param power The power to be applied to the motor.
   */
  public void runMotor(double power) {
    smartHoodMotor.setDutyCycle(power);
  }

  /**
   * Get the raw position of the hood's encoder.
   * 
   * @return The current position of the hood in encoder rotations.
   */
  public double getPos() {
    return encoder.get();
  }

  /**
   * Get the scaled position of the hood from 0 to 1.
   * 
   * @return The position of the hood scaled from 0 to 1.
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), HoodConstants.MIN_ENCODER_POSITION, HoodConstants.MAX_ENCODER_POSITION, 0.0,
        1.0);
  }

  /**
   * Get the current angle of the hood, based on the YAMS pivot system.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the YAMS
   *         pivot system.
   */
  public double getScaledPosAngleYAMS() {
    return hood.getAngle().in(Degrees);
  }

  /**
   * Get the current angle of the hood, based on the YAMS pivot system, relative
   * to the simulated zero position.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the YAMS
   *         pivot system.
   */
  public double getScaledPosAngleSim() {
    return PosUtils.mapRange(getScaledPosAngleYAMS(), HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE,
        HoodConstants.MAX_ANGLE, HoodConstants.MIN_ANGLE) - 40;
  }

  /**
   * Get the current angle of the hood, directly from the encoder value.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the
   *         encoder directly.
   */
  public double getScaledPosAngleEncoder() {
    return PosUtils.mapRange(getPos(), HoodConstants.MIN_ENCODER_POSITION, HoodConstants.MAX_ENCODER_POSITION,
        HoodConstants.MIN_ANGLE,
        HoodConstants.MAX_ANGLE);
  }

  /**
   * Get the current velocity of the hood.
   * 
   * @return The velocity of the hood, in degrees per second.
   */
  public double getVelocity() {
    return smartHoodMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Send values to NT to display on Elastic.
    /*
     * TODO: look into SmartDashboard alternatives, as 
     * it's deprecated, marked for removal along with Shuffleboard for next season.
     * Consider publishing to NT directly.
     */
    if (TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber("Hood Pos", getPos());
      SmartDashboard.putNumber("Scaled Hood Pos", getScaledPos());
      SmartDashboard.putNumber("Hood Angle (Encoder)", getScaledPosAngleEncoder());
      SmartDashboard.putNumber("Hood Angle (YAMS)", getScaledPosAngleYAMS());
    }

    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Updates the intake pivot simulation's values,
    hood.simIterate();
  }

  public Pose3d getHoodPose() {
    return new Pose3d(ModelConstants.ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET, new Rotation3d(0, -getScaledPosAngleYAMS(), 0));
  }

}

// seal gyu