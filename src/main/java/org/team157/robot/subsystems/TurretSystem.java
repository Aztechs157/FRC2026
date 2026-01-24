// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import org.team157.robot.Constants.TurretConstants;
import org.team157.utilities.PosUtils;
import org.team157.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID, TunerConstants.kCANBus);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);

  private DutyCycleOut request = new DutyCycleOut(0.0);
  private final MotionMagicExpoVoltage mm_request = new MotionMagicExpoVoltage(0);
  // we can switch to the bottom one for dynamic stuff later
  // private final DynamicMotionMagicExpoVoltage mm_request = new DynamicMotionMagicExpoVoltage(0, 0.12, 0.1);

  // TODO: find proper values for slot0Configs
  private final Slot0Configs slot0Configs = new Slot0Configs()
      .withKG(0) // output to overcome gravity (output)
      .withKS(0.25) // output to overcome static friction (output)
      .withKV(0.12) // output per unit of target velocity (output/rps)
      .withKA(0.01) // output per unit of target acceleration (output/(rps/s))
      .withKP(4.8) // output per unit of error in position (output/rotation)
      .withKI(0) // output per unit of integrated error in position (output/(rotation*s))
      .withKD(0.1); // output per unit of error in velocity (output/rps)
  
  // TODO: find proper values for motionMagicConfigs
  private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(0) // this one should stay as 0
      .withMotionMagicExpo_kV(0.12)
      .withMotionMagicExpo_kA(0.1);

  private final TalonFXConfiguration motorConfigs = new TalonFXConfiguration().withSlot0(slot0Configs).withMotionMagic(motionMagicConfigs);

  // public static StructArrayPublisher<Pose3d> zeroedPoses =
  // NetworkTableInstance.getDefault()
  // .getStructArrayTopic("ZeroedComponentPoses", Pose3d.struct).publish();

  // public static DoublePublisher renameMe = NetworkTableInstance.

  /** Creates a new TurretSystem. */
  public TurretSystem() {
    motor.getConfigurator().apply(motorConfigs);
  }

  public void runMotor(double power) {
    motor.set(power);
  }

  public void runWithLimits() {
    motor.set(PosUtils.runWithLimits(getPos(), getScaledPos(), getPos()));
  }

  public void goToPos(double angle) {
    // don't do it if angle is past limits
    if (angle > -135 && angle < 135) { // TODO: add some margin
      double rotations = angle * 0; // TODO: figure out the actual math to convert from angle to rotations
      motor.setControl(mm_request.withPosition(rotations));
    }
  }

  public double getPos() {
    return encoder.get();
  }

  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, 135.0,
        -135.0);
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
