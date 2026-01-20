// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import org.team157.robot.Constants.TurretConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(TurretConstants.POS_SENSOR_ID);
  /** Creates a new TurretSystem. */
  public TurretSystem() {}

   public void setPower(double power) {
    motor.set(power);
  }

  public double getPos() {
    return encoder.get();
  }

   public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_POSITION, TurretConstants.MAX_POSITION, 0.0,
        1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
