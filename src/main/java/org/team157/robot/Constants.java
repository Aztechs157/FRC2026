// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

  public static class ModifierConstants {
    // Reduces drive speed by this factor when precision mode is active.
    public static final double PRECISION_DRIVE_MODIFIER = 0.75;
    // When true, reduces drive speed by 50%.
    public static final boolean ROOKIE_MODE = false;
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
    public static final int MOTOR_ID = 21;
  }

  public static class UptakeConstants {
    public static final int MOTOR_ID = 22;
  }

  public static class TurretConstants {
    public static final int MOTOR_ID = 31;
    public static final int POS_SENSOR_ID = 32;
  }

  public static class HoodConstants {
    public static final int MOTOR_ID = 33;
    public static final int POT_ID = 34;
  }
  
  public static class FlywheelConstants {
    public static final int MOTOR_ID = 35;
    // public static final int MOTOR2_ID = 36; if we end up having 2 flywheels
  }

  public static class VisionConstants {
    public static final String FRONTLEFT_CAMERA_NICKNAME = "frontLeftCam";
    public static final Rotation3d FRONTLEFT_CAMERA_ROTATION = new Rotation3d(0, -0.0523599, -0.139626);
    public static final Translation3d FRONTLEFT_CAMERA_TRANSLATION = new Translation3d(0.2286, 0.3302, 0.4953);
    public static final Transform3d FRONTLEFT_CAMERA_PLACEMENT = new Transform3d(
            FRONTLEFT_CAMERA_TRANSLATION, FRONTLEFT_CAMERA_ROTATION);
    public static final String FRONTRIGHT_CAMERA_NICKNAME = "frontRightCam";
    public static final Rotation3d FRONTRIGHT_CAMERA_ROTATION = new Rotation3d(0, -0.0523599, 0.139626);
    public static final Translation3d FRONTRIGHT_CAMERA_TRANSLATION = new Translation3d(0.2286, 0.3038, 0.4953);
    public static final Transform3d FRONTRIGHT_CAMERA_PLACEMENT = new Transform3d(
            FRONTRIGHT_CAMERA_TRANSLATION, FRONTRIGHT_CAMERA_ROTATION);  
    public static final String BACK_CAMERA_NICKNAME = "backTopCam";
    public static final Rotation3d BACK_CAMERA_ROTATION = new Rotation3d(0, -0.0523599, 3.14159);
    public static final Translation3d BACK_CAMERA_TRANSLATION = new Translation3d(-0.2921, -0.0381, 0.4953);
    public static final Transform3d BACK_CAMERA_PLACEMENT = new Transform3d(
            BACK_CAMERA_TRANSLATION, BACK_CAMERA_ROTATION);

    public static final String TURRET_CAMERA_NICKNAME = "turretCam";
    public static final Rotation3d TURRET_CAMERA_ROTATION = new Rotation3d(0, -0.349066, 0);
    public static final Translation3d TURRET_CAMERA_TRANSLATION = new Translation3d(0.2602992, 0, 0.126);
    public static final Transform3d TURRET_CAMERA_PLACEMENT = new Transform3d(
            TURRET_CAMERA_TRANSLATION, TURRET_CAMERA_ROTATION);


    public static final PIDController AIMING_PID = new PIDController(0.05, 0, 0.01);
    // How close the robot can be (bumper to tag, in meters) before losing the
    // ability to auto-align.
    public static final double MIN_DISTANCE_TO_TAG = 0.8;
  }


}
