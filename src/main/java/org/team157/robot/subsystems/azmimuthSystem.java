// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;

import org.team157.robot.Constants.AzmimuthConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class azmimuthSystem extends SubsystemBase {

  private SparkMax motor = new SparkMax(AzmimuthConstants.MOTOR_ID, MotorType.kBrushless);
  /** Creates a new azmimuth. */
  public azmimuthSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
