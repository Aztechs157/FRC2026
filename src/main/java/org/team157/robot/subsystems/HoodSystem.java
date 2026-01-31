// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import org.team157.robot.Constants.HoodConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSystem extends SubsystemBase {

  private SparkMax motor = new SparkMax(HoodConstants.MOTOR_ID, MotorType.kBrushless);
  private AnalogInput pot = new AnalogInput(HoodConstants.POT_ID);
  
  /** Creates a new azmimuth. */
  public HoodSystem() {}

  public void setPower(double power) {
    motor.set(power);
  }

  public double getPower() {
    return motor.getEncoder().getVelocity();
  }

  public int getPosition() {
    return pot.getValue();
  }

}
