// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import org.team157.robot.parsing.PositionDetails;

import com.ctre.phoenix6.CANBus;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public final class Constants {
  public static final CANBus RIO_CAN_BUS = new CANBus("rio", "./logs/example.hoot");
  public static final CANBus DRIVE_CAN_BUS = new CANBus("canivore", "./logs/example2.hoot");

  public static class ModifierConstants {
    // Reduces drive speed by this factor when precision mode is active.
    public static final double PRECISION_DRIVE_MODIFIER = 0.75;
    // When true, reduces drive speed by 50%.
    public static final boolean ROOKIE_MODE = true;
    public static final double ROOKIE_DRIVE_MODIFIER = 0.5;
    // When true, reduces drive speed by 75% and disables auto positioning.
    // Overrides ROOKIE_MODE.
    public static final boolean DEMO_MODE = false;
    public static final double DEMO_DRIVE_MODIFIER = 0.25;
  }

  public static class ControllerConstants {
      // Ports for the Joysticks, as set in Driver Station
      public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
      // Joystick Deadband
      // TODO: since these are all the same, could we not just have a single DEADBAND constant?
      public static final double LEFT_X_DEADBAND = 0.05;
      public static final double LEFT_Y_DEADBAND = 0.05;
      public static final double RIGHT_X_DEADBAND = 0.05;
  }

  public static class IntakeConstants {
    // Pivot
    public static final int PIVOT_MOTOR_ID = 15;
    public static final int PIVOT_ENCODER_ID = 1;
    // public static final MechanismGearing PIVOT_GEARING = new MechanismGearing(GearBox.fromReductionStages(1, 95.83));
    public static final MechanismGearing PIVOT_GEARING = new MechanismGearing(GearBox.fromStages("23:1", "50:12"));
    public static final double KP = 157, KI = 0, KD = 0;
    public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(540);
    public static final double MIN_ENCODER_POSITION = 0.01, MAX_ENCODER_POSITION = 0.91;
    public static final double MIN_ANGLE = 0, MAX_ANGLE = 80;
    public static final Angle LOWER_SOFT_LIMIT = Degrees.of(0), UPPER_SOFT_LIMIT = Degrees.of(80);
    public static final Angle LOWER_HARD_LIMIT = Degrees.of(0), UPPER_HARD_LIMIT = Degrees.of(80);
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final Time RAMP_RATE = Seconds.of(0.00157);

    public static final int ROLLER_MOTOR_ID = 14;
  }

  public static class HopperConstants {
    public static final int MOTOR_ID = 16;
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("2:1"));

  }

  public static class UptakeConstants {
    public static final int MOTOR_ID = 17;
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("9:1"));

  }

  public static class TurretConstants {
    public static final int MOTOR_ID = 18;
    public static final int ENCODER_ID = 0;
    public static final double MIN_ENCODER_POSITION = 0.99, MAX_ENCODER_POSITION = 0.014;
    public static final double MIN_ANGLE = -179, MAX_ANGLE = 179;
    public static final double KP = 157, KI = 0, KD = 0;
    public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(2880);
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("28:1"));
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final Time RAMP_RATE = Seconds.of(0.00157);
    public static final Angle LOWER_SOFT_LIMIT = Degrees.of(-179), UPPER_SOFT_LIMIT = Degrees.of(179);
    public static final Angle LOWER_HARD_LIMIT = Degrees.of(-179), UPPER_HARD_LIMIT = Degrees.of(179);
  }

  public static class HoodConstants {
    // TODO: update these constants when testing on actual robot
    public static final int MOTOR_ID = 19;
    public static final int ENCODER_ID = 2;

    public static final double MIN_ENCODER_POSITION = 0.375, MAX_ENCODER_POSITION = 0.94;
    public static final double MIN_ANGLE = 40, MAX_ANGLE = 65;
    public static final Angle LOWER_SOFT_LIMIT = Degrees.of(45), UPPER_SOFT_LIMIT = Degrees.of(60);
    public static final Angle LOWER_HARD_LIMIT = Degrees.of(40), UPPER_HARD_LIMIT = Degrees.of(65);
    public static final double KP = 157, KI = 0,  KD = 0;
    public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(360);
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("32:14", "16:1"));
    public static final Current CURRENT_LIMIT = Amps.of(40);
    public static final Time RAMP_RATE = Seconds.of(0.00157);
    
  }
  
  public static class FlywheelConstants {
    public static final int MOTOR_ID = 20;
    public static final int MOTOR_ID_FOLLOWER = 21;
    // TODO: do actual tuning
    public static final double P = 0.000157, I = 0, D = 0;
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("1:1"));
    //TODO: put real values here and not made up ones
    public static final Distance FLYWHEEL_DIAMETER = Inches.of(3.75);
    public static final Mass FLYWHEEL_MASS = Pounds.of(2);
    public static final Distance HEIGHT = Feet.of(2.5);
    public static final Time RAMP_RATE = Seconds.of(0.25);

  }

  public static class VisionConstants {
    public static final String FRONTLEFT_CAMERA_NICKNAME = "frontLeftCam";
    public static final Rotation3d FRONTLEFT_CAMERA_ROTATION = new Rotation3d(0, 0, -Math.toRadians(65));
    public static final Translation3d FRONTLEFT_CAMERA_TRANSLATION = new Translation3d(-0.128, -0.329, 0.394);
    public static final Transform3d FRONTLEFT_CAMERA_PLACEMENT = new Transform3d(
            FRONTLEFT_CAMERA_TRANSLATION, FRONTLEFT_CAMERA_ROTATION);

    public static final String FRONTRIGHT_CAMERA_NICKNAME = "frontRightCam";
    public static final Rotation3d FRONTRIGHT_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(65));
    public static final Translation3d FRONTRIGHT_CAMERA_TRANSLATION = new Translation3d(0.128, -0.329, 0.394);
    public static final Transform3d FRONTRIGHT_CAMERA_PLACEMENT = new Transform3d(
            FRONTRIGHT_CAMERA_TRANSLATION, FRONTRIGHT_CAMERA_ROTATION);  

    public static final String BACK_CAMERA_NICKNAME = "backCam";
    public static final Rotation3d BACK_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(180));
    public static final Translation3d BACK_CAMERA_TRANSLATION = new Translation3d(-0.322, 0, 0.381);
    public static final Transform3d BACK_CAMERA_PLACEMENT = new Transform3d(
            BACK_CAMERA_TRANSLATION, BACK_CAMERA_ROTATION);

    public static final String TURRET_CAMERA_NICKNAME = "turretCam";
    public static final Rotation3d TURRET_CAMERA_ROTATION = new Rotation3d(0, 0, 0);
    public static final Translation3d TURRET_CAMERA_TRANSLATION = new Translation3d(0.2602992, 0, 0.126);
    public static final Transform3d TURRET_CAMERA_PLACEMENT = new Transform3d(
            TURRET_CAMERA_TRANSLATION, TURRET_CAMERA_ROTATION);


    public static final PIDController AIMING_PID = new PIDController(0.05, 0, 0.01);
    // How close the robot can be (bumper to tag, in meters) before losing the
    // ability to auto-align.
    public static final double MIN_DISTANCE_TO_TAG = 0.8;
  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Meters.of(16.54175);
    public static final Distance FIELD_WIDTH = Meters.of(8.0137);
    
    public static final PositionDetails positionDetails = new PositionDetails();
    

  }

  public static class ModelConstants {
    public static final Translation3d ORIGIN_TO_TURRET_BASE_OFFSET = new Translation3d(-0.171, 0, 0.460);
    public static final Translation3d ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET = new Translation3d(-0.0465, 0, 0.530);
    public static final Translation3d ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET = new Translation3d(0.146050, 0, 0.197803);
  }
}
