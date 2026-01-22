// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import org.team157.robot.Constants.TurretConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSystem extends SubsystemBase {
  private TalonFX motor;
  private DutyCycleEncoder encoder;

  // public static StructArrayPublisher<Pose3d> zeroedPoses =
  // NetworkTableInstance.getDefault()
  // .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();

  // public static DoublePublisher renameMe = NetworkTableInstance.

  /** Creates a new TurretSystem. */
  public TurretSystem() {
    motor = new TalonFX(TurretConstants.MOTOR_ID);
    encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);
    

  }

  public void runMotor(double power) {
    motor.set(power);
  }

  public void runWithLimits() {
    motor.set(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
  }

  public double getPos() {
    return encoder.get();
  }

  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, 0.0,
        1.0);
  }

  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Pos", getPos());
    SmartDashboard.putNumber("Scaled Turret Pos", getScaledPos());
  }
}
