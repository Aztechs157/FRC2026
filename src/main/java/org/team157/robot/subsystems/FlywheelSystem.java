// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;
// import the stuff 
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.motorcontrollers.SmartMotorController;

import org.team157.robot.Constants;
import org.team157.robot.Constants.FieldConstants;
import org.team157.robot.Constants.FlywheelConstants;
import org.team157.robot.Constants.HoodConstants;

import edu.wpi.first.math.Pair;
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

  private TalonFX motor  = new TalonFX(FlywheelConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
  private TalonFX motor_follower = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);
  public static double ballVelocity = 0; // Meters per second for the ball to be launched
  public static Angle hoodAngle = Radians.of(0);

  private SmartMotorControllerConfig flywheelSystemConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // feedback constant (pid constants)
    .withClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, RPM.of(6000), RotationsPerSecondPerSecond.of(3000)) // TODO: move to constants
    .withSimClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, RPM.of(6000), RotationsPerSecondPerSecond.of(3000))
    // telemetry name and verbosity level
    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
    // gearing from the motor rotor to final shaft
    .withGearing(FlywheelConstants.GEARING)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate((FlywheelConstants.RAMP_RATE))
    .withFollowers(Pair.of(motor_follower, true));

  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),flywheelSystemConfig);
 // private SmartMotorController smartMotorFollower = new TalonFXWrapper(motor_follower, DCMotor.getKrakenX60(1),flywheelSystemConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig(smartMotor)
  // diameter of the flywheel 
  .withDiameter((FlywheelConstants.FLYWHEEL_DIAMETER))
  // mass of the flywheel
  .withMass((FlywheelConstants.FLYWHEEL_MASS))
  // maximum speed of the shooter
  .withSoftLimit(FlywheelConstants.FLYWHEEL_RPM_LIMIT_LOWER, FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER)
  // .withUpperSoftLimit(RPM.of(FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER))
  // telemetry name and verbosity
  .withTelemetry("FlywheelDynamics", TelemetryVerbosity.HIGH);

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

  // Function to minimize (all parameters in metric units: meters, meters/second, radians)
  // Calculates required ball velocity (m/s) for given distance, height, and launch angle
  // Using projectile motion equations: y = x*tan(θ) - (g*x²)/(2*v₀²*cos²(θ))
  // Solving for v₀: v₀ = sqrt((g*x²)/(2*cos²(θ)*(x*tan(θ) - y)))
  static double velocityFunction(double distance, double height, double theta) {
    // gravity in m/s²
    double g = 9.81; 
    // height difference between target and shooter
    double heightDifference = height - FlywheelConstants.HEIGHT.magnitude();
    double denominator = 2 * Math.cos(theta) * Math.cos(theta) * (distance * Math.tan(theta) - heightDifference);
    return Math.sqrt((g * distance * distance) / denominator);
  }

  public static void setShotParams(double height, double distance) {
    double lowerBound = HoodConstants.LOWER_SOFT_LIMIT.in(Radians);
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

  // TODO: evaluate necessity of this method, as the original setShotParams is used in every instance.
  public void setBETTERShotParams(double height, double distance) {
    double lowerBound = HoodConstants.LOWER_SOFT_LIMIT.in(Radians);
    double upperBound = HoodConstants.UPPER_SOFT_LIMIT.in(Radians);
    double steps = 1000;
    double stepSize = (upperBound - lowerBound) / steps;
    
    double theta = lowerBound;
    double velocity = velocityFunction(distance, height, lowerBound);
    double previousVelocityDifference = -1;
    double upperBoundDegrees = Math.toDegrees(upperBound);

    for (int i = 1; i <= steps; i++) {
        double x = lowerBound + i * stepSize;
        double y = velocityFunction(distance, height, x);
        double velocityDifference = Math.abs(ballVelocity - y);
        if (previousVelocityDifference < 0 || (velocityDifference < previousVelocityDifference && Math.toDegrees(x) < upperBoundDegrees)) {
            previousVelocityDifference = velocityDifference;
            velocity = y;
            theta = x;
        }
    }

    ballVelocity = velocity;
    hoodAngle = Radians.of(theta);
  }

  public AngularVelocity getDesiredVelocity() {
    double heightMeters = FieldConstants.positionDetails.getTargetHeight();
    double distanceMeters = VisionSystem.distanceToTarget;
    setShotParams(heightMeters, distanceMeters);
    
    // Convert ball velocity (m/s) to flywheel RPM
    // Relationship: ballVelocity = (flywheelRPM / 60) * π * flywheel_radius
    // Therefore: flywheelRPM = (ballVelocity * 60) / (π * flywheel_diameter)
    // desiredRPM is divided by 0.4 to account for external factors like air resistace and wheel slip.
    double flywheelDiameterMeters = (FlywheelConstants.FLYWHEEL_DIAMETER).in(Meters);
    double desiredRPM = (ballVelocity * 60) / (Math.PI * flywheelDiameterMeters);
    return RPM.of(desiredRPM / FlywheelConstants.RPM_MULTIPLIER);
  }

  public static Angle getDesiredHoodAngle() {
    double heightMeters = FieldConstants.positionDetails.getTargetHeight();
    double distanceMeters = VisionSystem.distanceToTarget;
    setShotParams(heightMeters, distanceMeters);
    return Degrees.of(Math.toDegrees(hoodAngle.magnitude()));
  }

  // TODO: consider removing this, as it's never used and will always return 0.
  public double lossFunction() {
    return 0 * hoodAngle.magnitude();
  }

  // TODO: consider removing this unused method, as getDesiredHoodAngle already provides the hood angle that should be used for the shot.
  public Angle getHoodAngle() {
    return hoodAngle;
  }

  public Command setDynamicVelocity () {
    return flywheel.setSpeed(this::getDesiredVelocity);
  }

  /**
   * set the shooter velocity
   * @param speed speed to set
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity (AngularVelocity speed) {
    return flywheel.setSpeed(speed); 
  }

  
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return flywheel.set(dutyCycle);}

  /** Creates a new FlywheelSystem. */
  public FlywheelSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel Velocity", getVelocity().magnitude());
    SmartDashboard.putNumber("Flywheel RPM", getVelocity().in(RPM));
    SmartDashboard.putNumber("Distance to Target", VisionSystem.distanceToTarget);
    SmartDashboard.putNumber("Target Height", FieldConstants.positionDetails.getTargetHeight());
    SmartDashboard.putNumber("Desired Ball Velocity", getDesiredVelocity().in(RPM));
    SmartDashboard.putNumber("Desired Hood Angle", getDesiredHoodAngle().in(Degrees));
    flywheel.updateTelemetry();
  }

    @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    flywheel.simIterate();
  }
}


