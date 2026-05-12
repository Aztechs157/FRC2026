package org.team157.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    /** Roller motor IDs */
    public static final int ROLLER_MOTOR_ID = 14, FOLLOWER_MOTOR_ID = 15;

    public static final double KP = 1, KI = 0, KD = 0;
    public static final AngularVelocity ANGULAR_VELOCITY = RPM.of(5800);
    public static final AngularAcceleration ANGULAR_ACCELERATION =
            RotationsPerSecondPerSecond.of(11600);
}
