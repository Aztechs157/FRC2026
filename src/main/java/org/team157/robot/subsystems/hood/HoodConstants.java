package org.team157.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class HoodConstants {
  public static final int MOTOR_ID = 21;
  public static final int ENCODER_ID = 2;

  public static final double MIN_ENCODER_POSITION = 0.32, MAX_ENCODER_POSITION = 0.86;
  public static final double MIN_ANGLE = 40, MAX_ANGLE = 65;
  public static final Angle LOWER_SOFT_LIMIT = Degrees.of(42), UPPER_SOFT_LIMIT = Degrees.of(63);
  public static final Angle LOWER_HARD_LIMIT = Degrees.of(40), UPPER_HARD_LIMIT = Degrees.of(65);
  public static final double KP = 157, KI = 0, KD = 0;
  public static final double SIM_KP = 20, SIM_KI = 0, SIM_KD = 0;
  public static final AngularVelocity ANGULAR_VELOCITY = DegreesPerSecond.of(360);
  public static final AngularAcceleration ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(360);
  public static final MechanismGearing GEARING =
      new MechanismGearing(GearBox.fromStages("32:14", "16:1"));
  public static final Current CURRENT_LIMIT = Amps.of(40);
  public static final Time RAMP_RATE = Seconds.of(0.00157);
}
