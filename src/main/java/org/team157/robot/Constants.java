// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public final class Constants {

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
    public static final int MOTOR_ID = 21;
  }

  public static class UptakeConstants {
    public static final int MOTOR_ID = 22;
  }

  public static class TurretConstants {
    public static final int MOTOR_ID = 21;
    public static final int ENCODER_ID = 0;
    public static final double MIN_POSITION = 0.118, MAX_POSITION = 0.846;
    public static final double KP = 157, KI = 0, KD = 0;
    public static final double ANGULAR_VELOCITY = 360, ANGULAR_ACCELERATION = 2880;
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 5));
    public static final double CURRENT_LIMIT = 30;
    public static final double RAMP_RATE = 0.00157;
    public static final double LOWER_SOFT_LIMIT = -120, UPPER_SOFT_LIMIT = 120;
    public static final double LOWER_HARD_LIMIT = -135, UPPER_HARD_LIMIT = 135;
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
    public static final String FL_CAM_NAME = "Front Left Camera";
    public static final String FR_CAM_NAME = "Front Right Camera";
    public static final String BL_CAM_NAME = "Back Left Camera";
    public static final String BR_CAM_NAME = "Back Right Camera";
  }


}
