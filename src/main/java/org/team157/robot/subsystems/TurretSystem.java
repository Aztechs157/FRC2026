// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Constants.TurretConstants;
import org.team157.robot.generated.TunerConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);

  // Configure the turret motor controller for use with YAMS.
  private SmartMotorControllerConfig turretMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD, DegreesPerSecond.of(TurretConstants.ANGULAR_VELOCITY), DegreesPerSecondPerSecond.of(TurretConstants.ANGULAR_ACCELERATION))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withGearing(TurretConstants.GEARING)
      .withTelemetry("Turret Motor", TelemetryVerbosity.HIGH) 
      .withStatorCurrentLimit(Amps.of(TurretConstants.CURRENT_LIMIT))
      .withClosedLoopRampRate(Seconds.of(TurretConstants.RAMP_RATE))
      .withSoftLimit(Degrees.of(TurretConstants.LOWER_SOFT_LIMIT), Degrees.of(TurretConstants.UPPER_SOFT_LIMIT));

  // Create the turret's motor controller with the above configuration.
  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), turretMotorConfig);

  // Configure the physical characteristics of the turret.
  private PivotConfig turretConfig = new PivotConfig(smartMotor)
      .withStartingPosition(Degrees.of(getScaledPosAngleEncoder()))
      .withHardLimit(Degrees.of(TurretConstants.LOWER_HARD_LIMIT), Degrees.of(TurretConstants.UPPER_HARD_LIMIT))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH)
      .withMOI(Meters.of(0.1), Kilograms.of(4));

  // Create the turret pivot system with the above configuration.
  private Pivot turret = new Pivot(turretConfig);

  /** Creates a new TurretSystem. */
  public TurretSystem() {
    var configurator = motor.getConfigurator();
    configurator.refresh(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true));
    configurator.refresh(new ClosedLoopGeneralConfigs().withContinuousWrap(false));
  }
  
  /**
   * Set the target angle of the turret.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) { 
    return turret.setAngle(angle);
  }

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { 
    return turret.set(dutycycle);
  }

  /**
   * Run sysId on the {@link TurretSystem}.
   * To be used for tuning
   */
  public Command sysId() { 
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /**
   * Set the duty cycle output of the turret motor.
   * Primarily used for manual control
   * @param power The power to be applied to the motor.
   */
  public void runMotor(double power) {
    smartMotor.setDutyCycle(power);
  }

  // This is not used currently.
  public void runWithLimits() {
    smartMotor.setDutyCycle(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
  }

  /**
   * Get the raw position of the turret's encoder.
   * @return The current position of the turret in encoder rotations.
   */
  public double getPos() {
    return encoder.get();
  }

  /**
   * Get the scaled position of the turret from 0 to 1.
   * @return The position of the turret scaled from 0 to 1.
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_ENCODER_POSITION, TurretConstants.MAX_ENCODER_POSITION, 0.0,
        1.0);
  }

  /**
   * Get the current angle of the turret, based on the YAMS pivot system.
   * @return The angle of the turret, in degrees, from -180 to 180, using the YAMS pivot system.
   */
  public double getScaledPosAngleYAMS() {
    return turret.getAngle().in(Degrees);
  }
  /**
   * Get the current angle of the turret, directly from the encoder value.
   * @return The angle of the turret, in degrees, from -135 to 135, using the encoder directly.
   */
  public double getScaledPosAngleEncoder() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_ENCODER_POSITION, TurretConstants.MAX_ENCODER_POSITION, -135,
        135);
  }

  /**
   * Get the current velocity of the turret.
   * @return The velocity of the turret, in degrees per second.
   */
  public double getVelocity() {
    return smartMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Send values to NT to display on Elastic.
    /*  TODO: look into SmartDashboard alternatives, as it's deprecated, 
     *  marked for removal along with Shuffleboard for next season.
     *  Consider publishing to NT directly.
     */
    SmartDashboard.putNumber("Turret Pos", getPos());
    SmartDashboard.putNumber("Scaled Turret Pos", getScaledPos());
    SmartDashboard.putNumber("Turret Angle (YAMS)", getScaledPosAngleYAMS());
    SmartDashboard.putNumber("Turret Angle (Encoder)", getScaledPosAngleEncoder());
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    // Updates the turret simulation's values,
    turret.simIterate();
  }
}
