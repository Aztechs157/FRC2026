// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;



public final class TurretConstants {
    public static final int MOTOR_ID = 20;
    public static final int ENCODER_ID = 0;
    public static final double MIN_ENCODER_POSITION = 0.996, MAX_ENCODER_POSITION = 0.014;
    public static final double MIN_ANGLE = -178, MAX_ANGLE = 171;
    public static final double KP = 157, KI = 0, KD = 0;
    public static final double SIM_KP = 5, SIM_KI = 0, SIM_KD = 0;
    public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360), SIM_ANGULAR_VELOCITY = DegreesPerSecond.of(3.6);
    public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(2880), SIM_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(28.8);
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("28:1"));
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final Time RAMP_RATE = Seconds.of(0.00157);
    public static final Angle LOWER_SOFT_LIMIT = Degrees.of(-178), UPPER_SOFT_LIMIT = Degrees.of(171);
    public static final Angle LOWER_HARD_LIMIT = Degrees.of(-178), UPPER_HARD_LIMIT = Degrees.of(171);
    // Angle offset to account for misalignment between turret zero position and robot forward, in degrees.
    public static final double TURRET_ANGLE_OFFSET = 168.5;

  }