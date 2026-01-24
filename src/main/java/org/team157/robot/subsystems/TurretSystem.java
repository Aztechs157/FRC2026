// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants.TurretConstants;
import org.team157.robot.generated.TunerConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID, TunerConstants.kCANBus);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);
  private DutyCycleOut request = new DutyCycleOut(0.0);

  private SmartMotorControllerConfig conf = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(6, 0, 0) //TODO: this PID will probably not be accurate
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withGearing(10)
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH) //TODO: maybe change telemetry velocity
      .withStatorCurrentLimit(Amps.of(30))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withExternalEncoder(encoder)
      .withExternalEncoderInverted(false)
      .withExternalEncoderGearing(1)
      .withExternalEncoderZeroOffset(Degrees.of(180))
      .withUseExternalFeedbackEncoder(true);
  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), conf);
  private PivotConfig m_config= new PivotConfig(smartMotor)
      .withStartingPosition(Degrees.of(0))
      .withWrapping(Degrees.of(-180), Degrees.of(180))
      .withHardLimit(Degrees.of(-135), Degrees.of(135))
      .withSoftLimits(Degrees.of(-120), Degrees.of(120))
      .withTelemetry("Pivot", TelemetryVerbosity.HIGH);
  private Pivot turret = new Pivot(m_config);

  // public static StructArrayPublisher<Pose3d> zeroedPoses =
  // NetworkTableInstance.getDefault()
  // .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();

  // public static DoublePublisher renameMe = NetworkTableInstance.

  /** Creates a new TurretSystem. */

  
  /**
   * Set the angle of the arm.
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
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { 
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }


  public TurretSystem() {

  }

  public void runMotor(double power) {
    smartMotor.setDutyCycle(power);
  }

  public void runWithLimits() {
    smartMotor.setDutyCycle(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
  }

  public double getPos() {
    return encoder.get();
  }

  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, 0.0,
        1.0);
  }
  
  public double getScaledPosAngleKraken() {
    return turret.getAngle().in(Degrees);
  }

  public double getScaledPosAngleEncoder() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, -135,
        135);
  }

  public double getVelocity() {
    return smartMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Pos", getPos());
    SmartDashboard.putNumber("Scaled Turret Pos", getScaledPos());
    SmartDashboard.putNumber("Turret Angle (YAMS)", getScaledPosAngleKraken());
    SmartDashboard.putNumber("Turret Angle (Encoder)", getScaledPosAngleEncoder());
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
