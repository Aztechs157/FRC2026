package org.team157.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.RobotContainer;
import org.team157.robot.subsystems.hood.HoodConstants;
import org.team157.robot.subsystems.vision.VisionSystem;

/**
 * Represents the Flywheel subsystem, which spins up to launch balls at a calculated velocity
 * towards the target.
 */
public class Flywheel extends SubsystemBase {

  // The IO interface for interacting with the flywheel's motors.
  private FlywheelIO io;

  // Inputs from the motors and mechanism, to be updated periodically and logged.
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** The calculated ball velocity in meters per second required for the current shot. */
  public static double ballVelocity = 0;

  public static double ballTimeOfFlight = 0;

  /** The calculated hood angle in radians required for the current shot. */
  public static Angle hoodAngle = Radians.of(0);

  /** Creates a new Flywheel. */
  public Flywheel() {}

  /**
   * Specifies the IO implementation to be used for the Flywheel.
   *
   * @param io An implementation of the Flywheel's IO layer, i.e. FlywheelIOTalonFX
   */
  public void setIO(FlywheelIO io) {
    this.io = io;
  }

  /////////////////////////
  /// FLYWHEEL COMMANDS ///
  /////////////////////////

  /**
   * Sets the default command of the flywheel, stopping motor output when no other commands are
   * running.
   *
   * @return Command setting the duty cycle output of the flywheel's motor to 0
   */
  public Command getDefault() {
    return io.set(0);
  }

  /**
   * Set the duty cycle output of the flywheel motor.
   *
   * @param dutyCycle The power to be applied to the motor, between -1 and 1.
   * @return {@link Command} setting the duty cycle of the flywheel.
   */
  public Command set(double dutyCycle) {
    return io.set(dutyCycle);
  }

  /**
   * Set the flywheel to a fixed target angular velocity.
   *
   * @param speed The target angular velocity.
   * @return {@link Command} setting the flywheel to the specified velocity.
   */
  public Command setVelocity(AngularVelocity speed) {
    return io.setVelocity(speed);
  }

  /**
   * Set the flywheel to a dynamically-calculated velocity based on the current distance and height
   * to the target.
   *
   * @return {@link Command} continuously updating the flywheel velocity.
   */
  public Command setDynamicVelocity() {
    return io.setVelocity(this::getDesiredFlywheelVelocity);
  }

  ////////////////////////
  /// FLYWHEEL METHODS ///
  ////////////////////////

  /**
   * Gets the current velocity of the flywheel.
   *
   * @return The current angular velocity of the flywheel.
   */
  public AngularVelocity getVelocity() {
    return RPM.of(inputs.mechanismVelocityRPM);
  }

  ////////////////////////////////////////////
  /////// DYNAMIC VELOCITY CALCULATION ///////
  ////////////////////////////////////////////

  /**
   * Calculates required ball velocity (m/s) for given distance, height, and launch angle <br>
   * Using projectile motion equations: y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ)) <br>
   * Solving for v₀: v₀ = sqrt((g*x²)/(2*cos²(θ)*(x*tan(θ) - y)))
   *
   * @param distance The horizontal distance to the target in meters.
   * @param height The vertical height of the target in meters.
   * @param theta The launch angle in radians.
   * @return The required ball velocity in meters per second.
   */
  static double velocityFunction(double distance, double height, double theta) {
    double g = 9.81;
    double heightDifference = height - FlywheelConstants.HEIGHT.magnitude();
    double denominator =
        2 * Math.cos(theta) * Math.cos(theta) * (distance * Math.tan(theta) - heightDifference);
    if (denominator <= 0) {
      return 157; // Invalid shot parameters, return arbitrary velocity
    }
    return Math.sqrt((g * distance * distance) / denominator);
  }

  /**
   * Determines the lower bound of the hood angle search space based on whether the robot is under
   * the trench, since the hood must avoid collisions with the trench structure.
   *
   * @param isUnderTrench Whether or not the turret is currently under the trench, determined by the
   *     drivetrain's position on the field.
   * @return The lower bound of the hood angle search space, in radians.
   */
  double getLowerBound(boolean isUnderTrench) {
    if (isUnderTrench) {
      return HoodConstants.UPPER_SOFT_LIMIT.in(Radians);
    } else {
      return HoodConstants.LOWER_SOFT_LIMIT.in(Radians);
    }
  }

  /**
   * Calculates and updates the optimal flywheel velocity and hood angle for the current shot based
   * on target distance and height.
   *
   * @param height Target height in meters.
   * @param distance Distance to target in meters.
   */
  public void setShotParams(double height, double distance) {
    double lowerBound = getLowerBound(RobotContainer.drive.isUnderTrench());
    double upperBound = HoodConstants.UPPER_SOFT_LIMIT.in(Radians);
    double steps = 50;
    double stepSize = (upperBound - lowerBound) / steps;

    double theta = lowerBound;
    double velocity = velocityFunction(distance, height, lowerBound);

    for (int i = 1; i <= steps; i++) {
      double x = lowerBound + i * stepSize;
      double y = velocityFunction(distance, height, x);
      if (y < velocity) {
        velocity = y;
        theta = x;
      }
    }

    ballVelocity = velocity;
    hoodAngle = Radians.of(theta);
    ballTimeOfFlight =
        (velocity * Math.sin(theta)
                - Math.sqrt(
                    Math.pow(velocity * Math.sin(theta), 2)
                        - 2 * 9.81 * (height - FlywheelConstants.HEIGHT.magnitude())))
            / 9.81;
  }

  /**
   * Gets the desired flywheel velocity for the current shot, recalculating shot parameters each
   * time it is called.
   *
   * @return The desired angular velocity of the flywheel.
   */
  public AngularVelocity getDesiredFlywheelVelocity() {
    double heightMeters = FieldConstants.positionDetails.getTargetHeight();
    double distanceMeters = VisionSystem.distanceToTargetFromTurret;

    setShotParams(heightMeters, distanceMeters);

    // Convert ball velocity (m/s) to flywheel RPM:
    // flywheelRPM = (ballVelocity * 60) / (π * flywheel_diameter)
    // divided by SPEED_FACTOR to account for air resistance and wheel slip
    double flywheelDiameterMeters = FlywheelConstants.FLYWHEEL_DIAMETER.in(Meters);
    double desiredRPM =
        (ballVelocity * 60) / (Math.PI * flywheelDiameterMeters) * FlywheelConstants.SPEED_FACTOR;
    return RPM.of(Math.max(2800, desiredRPM));
  }

  public static double getBallTimeOfFlight() {
    return ballTimeOfFlight;
  }

  /**
   * Gets the desired hood angle for the current shot, as calculated by the most recent call to
   * {@link #setShotParams}.
   *
   * @return The desired hood angle.
   */
  public static Angle getDesiredHoodAngle() {
    return Degrees.of(Math.toDegrees(hoodAngle.magnitude()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates the inputs to be logged by AdvantageKit and writes them to the Logger
    io.updateInputs(inputs, getDesiredFlywheelVelocity());
    Logger.processInputs("Flywheel", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.simIterate();
  }
}
