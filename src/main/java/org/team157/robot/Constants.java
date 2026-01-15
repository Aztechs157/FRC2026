// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPPERATOR_CONTROLLER_PORT = 1;
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
    public static final String FL_CAM_NAME = "Front Left Camera";
    public static final String FR_CAM_NAME = "Front Right Camera";
    public static final String BL_CAM_NAME = "Back Left Camera";
    public static final String BR_CAM_NAME = "Back Right Camera";
  }

}
