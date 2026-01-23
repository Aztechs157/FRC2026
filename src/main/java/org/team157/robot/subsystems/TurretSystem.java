// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.team157.robot.Constants.TurretConstants;
import org.team157.robot.generated.TunerConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.PivotConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSystem extends SubsystemBase {
  private TalonFX motor;
  private DutyCycleEncoder encoder;
  private DutyCycleOut request;
  private SmartMotorControllerConfig conf;
  private SmartMotorController smartMotor ;
  private PivotConfig m_config;

  // public static StructArrayPublisher<Pose3d> zeroedPoses =
  // NetworkTableInstance.getDefault()
  // .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();

  // public static DoublePublisher renameMe = NetworkTableInstance.

  /** Creates a new TurretSystem. */
  public TurretSystem() {
    motor = new TalonFX(TurretConstants.MOTOR_ID, TunerConstants.kCANBus);
    encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);
    request = new DutyCycleOut(0.0);
    conf = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0) //TODO: this PID will probably not be accurate
    .withIdleMode(MotorMode.BRAKE)
    .withMotorInverted(false)
    .withGearing(0.1)
    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH) //TODO: maybe change telemetry velocity
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.25));
    smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), conf); //TODO: is this a falcon 500? and also is this only 1 motor?
    m_config = new PivotConfig(smartMotor)
    .withStartingPosition(Degrees.of(0))
    .withTelemetry("PivotExample", TelemetryVerbosity.HIGH);

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

  public double getVelocity() {
    return smartMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Pos", getPos());
    SmartDashboard.putNumber("Scaled Turret Pos", getScaledPos());
  }
}
