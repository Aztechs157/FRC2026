// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.Meters;

import org.team157.robot.parsing.PositionDetails;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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
    public static final int PIVOT_MOTOR_ID = 14;
    public static final int PIVOT_ENCODER_ID = 1;
    // public static final MechanismGearing PIVOT_GEARING = new MechanismGearing(GearBox.fromReductionStages(1, 95.83));
    public static final MechanismGearing PIVOT_GEARING = new MechanismGearing(GearBox.fromStages("23:1", "50:12"));
    public static final double KP = 157, KI = 0, KD = 0;
    public static final double ANGULAR_VELOCITY = 360, ANGULAR_ACCELERATION = 520;
    public static final double MIN_ENCODER_POSITION = 0.0157, MAX_ENCODER_POSITION = 0.9157;
    public static final double MIN_ANGLE = -0, MAX_ANGLE = 80;
    public static final double LOWER_SOFT_LIMIT = 0, UPPER_SOFT_LIMIT = 78;
    public static final double LOWER_HARD_LIMIT = -5, UPPER_HARD_LIMIT = 85;
    public static final double CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.00157;

    public static final int ROLLER_MOTOR_ID = 15;
  }

  public static class HopperConstants {
    public static final int MOTOR_ID = 16;
    public static final double CURRENT_LIMIT = 40;
  }

  public static class UptakeConstants {
    public static final int MOTOR_ID = 17;
    public static final double CURRENT_LIMIT = 40;
  }

  public static class TurretConstants {
    public static final int MOTOR_ID = 18;
    public static final int ENCODER_ID = 0;
    public static final double MIN_ENCODER_POSITION = 0.1, MAX_ENCODER_POSITION = 1.2;
    public static final double MIN_ANGLE = -32, MAX_ANGLE = 430;
    public static final double KP = 157, KI = 0, KD = 0;
    public static final double ANGULAR_VELOCITY = 360, ANGULAR_ACCELERATION = 2880;
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 5));
    public static final double CURRENT_LIMIT = 30;
    public static final double RAMP_RATE = 0.00157;
    public static final double LOWER_SOFT_LIMIT = -120, UPPER_SOFT_LIMIT = 120;
    public static final double LOWER_HARD_LIMIT = -135, UPPER_HARD_LIMIT = 135;
  }

  public static class HoodConstants {
    // TODO: update these constants when testing on actual robot
    public static final int MOTOR_ID = 19;
    public static final int ENCODER_ID = 2;
    public static final double MIN_ENCODER_POSITION = 0.0157, MAX_ENCODER_POSITION = 0.9157;
    public static final double MIN_ANGLE = 0, MAX_ANGLE = 40;
    public static final double LOWER_SOFT_LIMIT = 0, UPPER_SOFT_LIMIT = 45;
    public static final double LOWER_HARD_LIMIT = 0, UPPER_HARD_LIMIT = 45;
    public static final double KP = 100, KI = 0,  KD = 0;
    public static final double ANGULAR_VELOCITY = 360, ANGULAR_ACCELERATION = 520;
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("23:1", "50:12"));
    public static final double CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.00157;
    
  }
  
  public static class FlywheelConstants {
    public static final int MOTOR_ID = 20;
    public static final int MOTOR_ID_FOLLOWER = 21;
    // TODO: do actual tuning
    public static final double P = 1, I = 0, D = 0;
    public static final double KS = 0, KV = 0, KA = 0;
    public static final double GEARING = 0.7;
    //TODO: put real values here and not made up ones
    public static final double FLYWHEEL_DIAMETER = 3.75; // inches
    public static final double FLYWHEEL_MASS = 4; // pounds
    public static final double FLYWHEEL_RPM_LIMIT_UPPER = 1000; 
    public static final double HEIGHT = 2.5;

  }

  public static class VisionConstants {
    public static final String FRONTLEFT_CAMERA_NICKNAME = "frontLeftCam";
    public static final Rotation3d FRONTLEFT_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(25));
    public static final Translation3d FRONTLEFT_CAMERA_TRANSLATION = new Translation3d(-0.136, 0.310, 0.339);
    public static final Transform3d FRONTLEFT_CAMERA_PLACEMENT = new Transform3d(
            FRONTLEFT_CAMERA_TRANSLATION, FRONTLEFT_CAMERA_ROTATION);

    public static final String FRONTRIGHT_CAMERA_NICKNAME = "frontRightCam";
    public static final Rotation3d FRONTRIGHT_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(25));
    public static final Translation3d FRONTRIGHT_CAMERA_TRANSLATION = new Translation3d(-0.136, -0.310, 0.339);
    public static final Transform3d FRONTRIGHT_CAMERA_PLACEMENT = new Transform3d(
            FRONTRIGHT_CAMERA_TRANSLATION, FRONTRIGHT_CAMERA_ROTATION);  

    public static final String BACK_CAMERA_NICKNAME = "backCam";
    public static final Rotation3d BACK_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(180));
    public static final Translation3d BACK_CAMERA_TRANSLATION = new Translation3d(-0.301, 0, 0.294);
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
