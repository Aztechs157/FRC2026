// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.team157.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import org.team157.robot.parsing.PositionDetails;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final CANBus RIO_CAN_BUS = new CANBus("rio", "./logs/example.hoot");

    public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Meters.of(16.54175);
        public static final Distance FIELD_WIDTH = Meters.of(8.0137);
        /** Manual reset position in blue alliance HP corner */
        public static final Pose2d MANUAL_RESET_POSE_BLUE =
                new Pose2d(0.441, 0.441, new Rotation2d());
        /** Manual reset position in red alliance HP corner */
        public static final Pose2d MANUAL_RESET_POSE_RED =
                new Pose2d(
                        FieldConstants.FIELD_LENGTH.in(Meters) - 0.441,
                        FieldConstants.FIELD_WIDTH.in(Meters) - 0.441,
                        new Rotation2d(Degrees.of(180)));

        public static final PositionDetails positionDetails = new PositionDetails();
    }
}
