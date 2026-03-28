// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import org.team157.robot.parsing.PositionDetails;
import com.ctre.phoenix6.CANBus;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final CANBus RIO_CAN_BUS = new CANBus("rio", "./logs/example.hoot");
  public static final CANBus DRIVE_CAN_BUS = new CANBus("canivore", "./logs/example2.hoot");

  public static class ModifierConstants {
    // Reduces drive speed by this factor when precision mode is active.
    public static final double PRECISION_DRIVE_MODIFIER = 0.5;
    // When true, reduces drive speed by 50%.
    public static final boolean ROOKIE_MODE = false;
    public static final double ROOKIE_DRIVE_MODIFIER = 0.5;
    // When true, reduces drive speed by 75%.
    // Overrides ROOKIE_MODE.
    public static final boolean DEMO_MODE = false;
    public static final double DEMO_DRIVE_MODIFIER = 0.25;
    /**
     * When true, swaps the intake and shooting triggers, 
     * per Maya's preference. Makes no change to drivetrain speed.
     * For future reference, avoid making catches for specific 
     * user edge cases like this, make decisions off general consensus.
     */
    public static final boolean MAYA_MODE = false;

  }

  public static class ControllerConstants {
    // Ports for the Joysticks, as set in Driver Station
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
    // Joystick Deadband
    public static final double JOYSTICK_DEADBAND = 0.05;
  }

  public static class VisionConstants {
    public static final String FRONTLEFT_CAMERA_NICKNAME = "frontLeftCam";
    public static final Rotation3d FRONTLEFT_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(65));
    public static final Translation3d FRONTLEFT_CAMERA_TRANSLATION = new Translation3d(-0.127, 0.3302, 0.3556);
    public static final Transform3d FRONTLEFT_CAMERA_PLACEMENT = new Transform3d(
        FRONTLEFT_CAMERA_TRANSLATION, FRONTLEFT_CAMERA_ROTATION);

    public static final String FRONTRIGHT_CAMERA_NICKNAME = "frontRightCam";
    public static final Rotation3d FRONTRIGHT_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(-65));
    public static final Translation3d FRONTRIGHT_CAMERA_TRANSLATION = new Translation3d(-0.127, -0.3302, 0.3556);
    public static final Transform3d FRONTRIGHT_CAMERA_PLACEMENT = new Transform3d(
        FRONTRIGHT_CAMERA_TRANSLATION, FRONTRIGHT_CAMERA_ROTATION);

    public static final String BACK_CAMERA_NICKNAME = "backCam";
    public static final Rotation3d BACK_CAMERA_ROTATION = new Rotation3d(0, 0, Math.toRadians(180));
    public static final Translation3d BACK_CAMERA_TRANSLATION = new Translation3d(-0.32326, 0, 0.3556);
    public static final Transform3d BACK_CAMERA_PLACEMENT = new Transform3d(
        BACK_CAMERA_TRANSLATION, BACK_CAMERA_ROTATION);
  }


  public static class TelemetryConstants {
    public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.LOW;

  }

  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Meters.of(16.54175);
    public static final Distance FIELD_WIDTH = Meters.of(8.0137); 
    /** Manual reset position in blue alliance HP corner */
    public static final Pose2d MANUAL_RESET_POSE_BLUE = new Pose2d(0.441, 0.441, new Rotation2d());
    /** Manual reset position in red alliance HP corner */
    public static final Pose2d MANUAL_RESET_POSE_RED = new Pose2d(FieldConstants.FIELD_LENGTH.in(Meters) - 0.441, FieldConstants.FIELD_WIDTH.in(Meters) - 0.441, new Rotation2d(Degrees.of(180)));
  
    public static final PositionDetails positionDetails = new PositionDetails();

  }

  public static class ModelConstants {
    // 3D offsets from the robot's origin (center of rotation) to various key points
    // on the robot, used for mechanism visualization on the AdvantageScope model.
    public static final Translation3d ORIGIN_TO_TURRET_BASE_OFFSET = new Translation3d(-0.12192, -0.11557, 0.396535);
    public static final Translation3d ORIGIN_TO_HOOD_PIVOT_POINT_OFFSET = new Translation3d(-0.0465, 0, 0.530);
    public static final Translation3d ORIGIN_TO_INTAKE_PIVOT_POINT_OFFSET = new Translation3d(0.146050, 0, 0.197803);
    // 2D offset from the robot's origin to the turret base, used in position-based
    // dynamic shooting calculations.
    public static final Transform2d XY_ORIGIN_TO_TURRET_BASE_OFFSET = new Transform2d(-0.12192, -0.11557, new Rotation2d());
  }
}
