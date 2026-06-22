// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.uptake;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public final class UptakeConstants {
    /** CAN IDs for both Uptake motors */
    public static final int MOTOR_ID = 18, FOLLOWER_MOTOR_ID = 19;
    /** The current draw limit of the subsystem, in amps. */
    public static final Current CURRENT_LIMIT = Amps.of(40);
    /** The gear ratio of the uptake subsystem. */
    public static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromStages("9:1"));
    /**
     * The possible duty cycle outputs which the uptake can receive as its setpoint, depending on
     * the turret state.
     */
    public static final double RUN_SPEED_HIGH = 1.0, RUN_SPEED_LOW = -0.157;
}
