// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.flywheel;

// import the stuff 
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.motorcontrollers.SmartMotorController;

import org.team157.robot.Constants;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.robot.RobotContainer;
import org.team157.robot.subsystems.hood.HoodConstants;
import org.team157.robot.subsystems.vision.VisionSystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSystem extends SubsystemBase {

  private TalonFX motor = new TalonFX(FlywheelConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
  private TalonFX motor_follower = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);
  public static double ballVelocity = 0; // Meters per second for the ball to be launched
  public static Angle hoodAngle = Radians.of(0);

  private SmartMotorControllerConfig flywheelSystemConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD,
          FlywheelConstants.ANGULAR_VELOCITY, FlywheelConstants.ANGULAR_ACCELERATION)
      .withFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
      .withSimClosedLoopController(FlywheelConstants.SIM_KP, FlywheelConstants.SIM_KI, FlywheelConstants.SIM_KD,
          FlywheelConstants.ANGULAR_VELOCITY, FlywheelConstants.ANGULAR_ACCELERATION)
      .withSimFeedforward(
          new SimpleMotorFeedforward(FlywheelConstants.SIM_KS, FlywheelConstants.SIM_KV, FlywheelConstants.SIM_KA))
      .withTelemetry("FlywheelMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withGearing(FlywheelConstants.GEARING)
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT)
      .withClosedLoopRampRate((FlywheelConstants.RAMP_RATE))
      .withFollowers(Pair.of(motor_follower, true));

  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), flywheelSystemConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(smartMotor)
      // diameter of the flywheel
      .withDiameter((FlywheelConstants.FLYWHEEL_DIAMETER))
      // mass of the flywheel
      .withMass((FlywheelConstants.FLYWHEEL_MASS))
      // maximum speed of the shooter
      .withSoftLimit(FlywheelConstants.FLYWHEEL_RPM_LIMIT_LOWER, FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER)
      // .withUpperSoftLimit(RPM.of(FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER))
      // telemetry name and verbosity
      .withTelemetry("FlywheelDynamics", TelemetryConstants.TELEMETRY_VERBOSITY);

  // flywheel mechanism
  private FlyWheel flywheel = new FlyWheel(flywheelConfig);

  /**
   * 
   * gets the current velocity of the flywheel
   * 
   * @return flywheel velocity
   */
  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  ///////////////////////////////////////////
  /////// DYNAMIC VELOCITY CALCULATION //////
  ///////////////////////////////////////////
  // Function to minimize (all parameters in metric units: 
  // meters, meters/second, radians)
  // Calculates required ball velocity (m/s) 
  // for given distance, height, and launch angle
  // Using projectile motion equations: y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ))
  // Solving for v₀: v₀ = sqrt((g*x²)/(2*cos²(θ)*(x*tan(θ) - y)))
  static double velocityFunction(double distance, double height, double theta) {
    // gravity in m/s²
    double g = 9.81;
    // height difference between target and shooter
    double heightDifference = height - FlywheelConstants.HEIGHT.magnitude();
    double denominator = 2 * Math.cos(theta) * Math.cos(theta) * (distance * Math.tan(theta) - heightDifference);

    if (denominator <= 0) {
      return 157; // Invalid shot parameters, return arbitrary velocity
    }
    return Math.sqrt((g * distance * distance) / denominator);
  }

  double getLowerBound(boolean isUnderTrench){
    if(isUnderTrench){
      return HoodConstants.UPPER_SOFT_LIMIT.in(Radians);
    } else {
      return HoodConstants.LOWER_SOFT_LIMIT.in(Radians);
    }
  }

  public void setShotParams(double height, double distance) {
    double lowerBound = getLowerBound(RobotContainer.drivetrain.isUnderTrench());
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
    SmartDashboard.putNumber("Hood Angle", Math.toDegrees(hoodAngle.magnitude()));
  }

  public AngularVelocity getDesiredFlywheelVelocity() {
    double heightMeters = FieldConstants.positionDetails.getTargetHeight();
    double distanceMeters = VisionSystem.distanceToTargetFromTurret; // Use distance from turret instead of robot center

    setShotParams(heightMeters, distanceMeters);

    // Convert ball velocity (m/s) to flywheel RPM
    // Relationship: ballVelocity = (flywheelRPM / 60) * π * flywheel_radius
    // Therefore: flywheelRPM = (ballVelocity * 60) / (π * flywheel_diameter)
    // desiredRPM is divided by 0.4 to account for 
    // external factors like air resistace and wheel slip.
    double flywheelDiameterMeters = (FlywheelConstants.FLYWHEEL_DIAMETER).in(Meters);
    double desiredRPM = ((ballVelocity) * 60) / (Math.PI * flywheelDiameterMeters) * FlywheelConstants.SPEED_FACTOR;
    return RPM.of(Math.max(2800, desiredRPM));
  }

  public static Angle getDesiredHoodAngle() {
    return Degrees.of(Math.toDegrees(hoodAngle.magnitude()));
  }

  public Command setDynamicVelocity() {
    return flywheel.setSpeed(this::getDesiredFlywheelVelocity);
  }

  /**
   * set the shooter velocity
   * 
   * @param speed speed to set
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return flywheel.setSpeed(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  /**
   * Disable flywheel output when no other commands are running.
   * 
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand} setting the
   *         flywheel to 0 duty cycle
   */
  public Command setDefault() {
    return set(0);
  }

  /** Creates a new FlywheelSystem. */
  public FlywheelSystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber("Distance to Target", VisionSystem.distanceToTargetFromTurret);
      SmartDashboard.putNumber("Target Height", FieldConstants.positionDetails.getTargetHeight());
      SmartDashboard.putNumber("Ball Velocity (m/s)", ballVelocity);
      SmartDashboard.putBoolean("Ball Velocity is NaN", Double.isNaN(ballVelocity));
      SmartDashboard.putNumber("Desired Flywheel Velocity", getDesiredFlywheelVelocity().in(RPM));
    }

    SmartDashboard.putNumber("Flywheel RPM", getVelocity().in(RPM));

    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    flywheel.simIterate();
  }
}
