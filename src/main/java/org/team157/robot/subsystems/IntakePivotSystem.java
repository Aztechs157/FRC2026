// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Robot;
import org.team157.robot.Constants.IntakeConstants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakePivotSystem extends SubsystemBase {
  
    ////////////////////
   /// INTAKE PIVOT ///
  ////////////////////
  private TalonFX pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, Constants.RIO_CAN_BUS);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER_ID);

  // Configure the hood motor controller for use with YAMS.
  private SmartMotorControllerConfig intakePivotMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD,
          IntakeConstants.ANGULAR_VELOCITY, IntakeConstants.ANGULAR_ACCELERATION)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(true)
      .withGearing(IntakeConstants.PIVOT_GEARING)
      .withTelemetry("Intake Pivot Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withStatorCurrentLimit(IntakeConstants.CURRENT_LIMIT)
      .withClosedLoopRampRate(IntakeConstants.RAMP_RATE);

  // Create the hood's motor controller with the above configuration.
  private SmartMotorController smartIntakePivotMotor = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX44(1),
      intakePivotMotorConfig);

  // Configure the physical characteristics of the hood.
  private PivotConfig intakePivotConfig = new PivotConfig(smartIntakePivotMotor)
      .withStartingPosition(Degrees.of(getScaledPosAngleEncoder()))
      .withHardLimit((IntakeConstants.LOWER_HARD_LIMIT), (IntakeConstants.UPPER_HARD_LIMIT))
      .withSoftLimits((IntakeConstants.LOWER_SOFT_LIMIT), (IntakeConstants.UPPER_SOFT_LIMIT))
      .withTelemetry("Intake Pivot", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withMOI(Meters.of(0.75), Kilograms.of(1));

  // Create the hood pivot system with the above configuration.
  private Pivot intakePivot = new Pivot(intakePivotConfig);

  public boolean deployState = false;

  public boolean getDeployState() {
    return deployState;

  }

  /**
   * @param 
   * @return
   */
  public Command invertDeployState() {
    return new InstantCommand(() -> deployState = !deployState);
  }


  /** Creates a new IntakeSystem */
  public IntakePivotSystem() {
    
  }

  /**
   * Set the target angle of the intake pivot.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return intakePivot.setAngle(angle).finallyDo(() -> intakePivot.setDutyCycleSetpoint(0));
  }

  public Command setAngleThenStop(Angle angle) {
    return setAngle(angle)
        .until(() -> PosUtils.isOscillating(angle.in(Degrees), intakePivot.getAngle().in(Degrees), 2.0, 0.0, 1.0));

  }

  public Command deployIntake() {
    return setAngleThenStop(Degrees.of(0));
  }

  public Command deployIntakeAndHold() {
    return setAngle(Degrees.of(0)); //TODO: confirm weither we still need the hold current and/or/also consider setting the angle to a negative to constantly press intake down and have pid at the same time
  }


  public Command retractIntake() {
    return setAngleThenStop(Degrees.of(78));
  }

  /** Quickly moves the intake upd and down to agitate fuel. */
  public Command wiggleIntake() {
    return setAngleThenStop(Degrees.of(40)).andThen(setAngleThenStop(Degrees.of(0)));
  }

  /**
   * Move the arm up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command setPivot(double dutycycle) {
    return intakePivot.set(dutycycle);
  }

  /**
   * Run sysId on the {@link HoodSystem}.
   * Base code from YAMS Template
   */
  public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /**
   * Set the duty cycle output of the hood motor.
   * Primarily used for manual control
   * 
   * @param power The power to be applied to the motor.
   */
  public void runMotor(double power) {
    smartIntakePivotMotor.setDutyCycle(power);
  }

  // This is not used currently.
  public void runWithLimits() {
    smartIntakePivotMotor.setDutyCycle(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
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
    return PosUtils.mapRange(getPos(), IntakeConstants.MIN_ENCODER_POSITION, IntakeConstants.MAX_ENCODER_POSITION, 0.0,
        1.0);
  }

  /**
   * Get the current angle of the hood, based on the YAMS pivot system.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the YAMS
   *         pivot system.
   */
  public double getScaledPosAngleYAMS() {
    return intakePivot.getAngle().in(Degrees);
  }

  /**
   * Get the current angle of the hood, directly from the encoder value.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the
   *         encoder directly.
   */
  public double getHopperWallsPosition() {
    return PosUtils.mapRange(getScaledPosAngleYAMS(), IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE, 0.3048, 0);
  }

  /**
   * Get the current angle of the hood, directly from the encoder value.
   * 
   * @return The angle of the hood, in degrees, from -180 to 180, using the
   *         encoder directly.
   */
  public double getScaledPosAngleEncoder() {
    if (!Robot.isReal()) {
      return 0;
    }
    return PosUtils.mapRange(getPos(), IntakeConstants.MIN_ENCODER_POSITION, IntakeConstants.MAX_ENCODER_POSITION,
        IntakeConstants.MIN_ANGLE,
        IntakeConstants.MAX_ANGLE);
  }

  /**
   * Get the current velocity of the hood.
   * 
   * @return The velocity of the hood, in degrees per second.
   */
  public double getVelocity() {
    return smartIntakePivotMotor.getMechanismVelocity().in(DegreesPerSecond);
  }


  public Command setDefault() {
    return run(() -> {
      // intakePivot.setDutyCycleSetpoint(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Send values to NT to display on Elastic.
    /*
     * TODO: look into SmartDashboard alternatives, as it's deprecated,
     * marked for removal along with Shuffleboard for next season.
     * Consider publishing to NT directly.
     */
    if(TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
    SmartDashboard.putNumber("Intake Pivot Pos", getPos());
    SmartDashboard.putNumber("Scaled Intake Pivot Pos", getScaledPos());
    SmartDashboard.putNumber("Intake Pivot Angle (Encoder)", getScaledPosAngleEncoder());
    }

    
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during 
    // simulation. Updates the intake pivot simulation's values,
    intakePivot.simIterate();
  }

  public Pose3d getHopperWallsPose() {
    return new Pose3d(getHopperWallsPosition(), 0, 0, new Rotation3d());
  }

  public Pose3d getIntakePivotPose() {
    return new Pose3d(ModelConstants.ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET,
        new Rotation3d(0, -Math.toRadians(getScaledPosAngleYAMS()), 0));
  }

}
