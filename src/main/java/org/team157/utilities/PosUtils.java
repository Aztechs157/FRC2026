// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.utilities;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;

/** Add your docs here. */
public interface PosUtils {

  /**
   * checks if the currentPos is within posTolerance of desiredPos and the motorVelocity is within
   * velocityTolerance in the direction towards desiredPos.
   *
   * @param desiredPos - the position you wish the system to reach
   * @param currentPos - the position the system is currently at
   * @param posTolerance - the allowable deviation from the desired position
   * @param motorVelocity - the velocity that the motor is currently spinning at
   * @param velocityTolerance - the maximum motor velocity for the system to not overshoot the
   *     desired position tolerance
   * @return - boolean
   */
  static boolean isOscillating(
      double desiredPos,
      double currentPos,
      double posTolerance,
      double motorVelocity,
      double velocityTolerance) {
    boolean retval = false;

    // Checks if currentPos is within the tolerance of desiredPos
    if ((currentPos >= desiredPos - posTolerance) && (currentPos <= desiredPos + posTolerance)) {
      // Checks if motorVelocity is within its tolerance
      if (Math.abs(motorVelocity) <= velocityTolerance) {
        retval = true;
      }
    }
    return retval;
  }

  /**
   * Maps a value from one range to another, based on code found <a href=
   * "https://reference.arduino.cc/reference/en/language/functions/math/map/#_appendix">here.</a>
   * Note that while the arduino version uses integers, the calculation is the same for doubles, so
   * the warning about integer math is not applicable here.
   *
   * @param input The current reading of the value
   * @param minInput The minimum reading possible for the value, this will be mapped to <code>
   *     minOutput</code>
   * @param maxInput The maximum reading possible for the value, this will be mapped to <code>
   *     maxOutput</code>
   * @param minOutput The minimum value for the new range
   * @param maxOutput The maximum value for the new range
   * @return The input value after being properly scaled to fit inside of the new range
   */
  static double mapRange(
      double input, double minInput, double maxInput, double minOutput, double maxOutput) {
    return (input - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
  }

  static double runWithLimits(double speed, double currentPos, double limitMargin) {
    if (currentPos >= 1.0 - limitMargin && speed < 0) {
      return 0;
    } else if (currentPos >= 0.9 - limitMargin && speed < 0) {
      return speed * 0.75;
    } else if (currentPos <= 0.0 + limitMargin && speed > 0) {
      return 0;
    } else if (currentPos <= 0.1 + limitMargin && speed > 0) {
      return speed * 0.75;
    } else {
      return speed;
    }
  }

  static double runWithLimits(
      double speed, double currentPos, double limitMargin, double minPos, double maxPos) {
    if (currentPos >= maxPos - limitMargin && speed < 0) {
      return 0;
    } else if (currentPos >= (maxPos * 0.9) - limitMargin && speed < 0) {
      return speed * 0.75;
    } else if (currentPos <= minPos + limitMargin && speed > 0) {
      return 0;
    } else if (currentPos <= (minPos * 0.9) + limitMargin && speed > 0) {
      return speed * 0.75;
    } else {
      return speed;
    }
  }

  static boolean isWithin(double value, double target, double tolerance) {
    return (value > target - tolerance && value < target + tolerance);
  }

  static final StringTopic selectedTabTopic =
      NetworkTableInstance.getDefault().getStringTopic("/Elastic/SelectedTab");
  static final StringPublisher selectedTabPublisher =
      selectedTabTopic.publish(PubSubOption.keepDuplicates(true));

  public static void selectTab(String tabName) {
    selectedTabPublisher.set(tabName);
  }

  /**
   * Selects the tab of the dashboard at the given index. If this index is greater than or equal to
   * the number of tabs, this will have no effect.
   *
   * @param tabIndex the index of the tab to select.
   */
  public static void selectTab(int tabIndex) {
    selectTab(Integer.toString(tabIndex));
  }
}
