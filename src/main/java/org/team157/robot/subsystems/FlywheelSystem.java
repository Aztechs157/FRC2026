// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems;
// import the stuff 
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import yams.motorcontrollers.SmartMotorController;


import org.team157.robot.Constants.FlywheelConstants;
import org.team157.robot.Constants.HoodConstants;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FlywheelSystem extends SubsystemBase {

  private TalonFX motor  = new TalonFX(FlywheelConstants.MOTOR_ID);
  private static TalonFX motor_follower = new TalonFX(FlywheelConstants.MOTOR_ID_FOLLOWER);
  public double ballVelocity = 0; //Feet per Second for the ball to be launched
  public Angle azimuth = Radians.of(0);

  private SmartMotorControllerConfig flywheelSystemConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    // feedback constant (pid constants)
    .withClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(FlywheelConstants.P, FlywheelConstants.I, FlywheelConstants.D, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    // feedforward constants
    .withFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
    .withSimFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
    // telemetry name and verbosity level
    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
    // gearing from the motor rotor to final shaft
    // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
    // You could also use .withGearing(12) which does the same thing.
    .withGearing(FlywheelConstants.GEARING)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withFollowers(Pair.of(motor_follower, false));
    // vendor motor controller object


  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1),flywheelSystemConfig);
 // private SmartMotorController smartMotorFollower = new TalonFXWrapper(motor_follower, DCMotor.getKrakenX60(1),flywheelSystemConfig);

  private final FlyWheelConfig flyWheelConfig = new FlyWheelConfig(smartMotor)
  // diameter of the flywheel 
  .withDiameter(Inches.of(FlywheelConstants.FLYWHEEL_DIAMETER))
  // mass of the flywheel
  .withMass(Pounds.of(FlywheelConstants.FLYWHEEL_MASS))
  // maximum speed of the shooter
  .withUpperSoftLimit(RPM.of(FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER))
  // telemetry name and verbosity
  .withTelemetry("FlywheelDynamics", TelemetryVerbosity.HIGH);

  // flywheel mechanism
  private FlyWheel flyWheel = new FlyWheel(flyWheelConfig);


    /**
     * 
     * gets the current velocity of the flywheel
     * 
     * @return flywheel velocity
     */
  public AngularVelocity getVelocity() {return flyWheel.getSpeed();}

  // Function to minimize: 
  double velocityFunction(double distance, double height, double theta) {
        return distance / Math.cos(theta) * (Math.sqrt(16 / (distance * Math.tan(theta) - (height - FlywheelConstants.HEIGHT))));
    }

  public void setShotParams(double height, double distance) {
    double lowerBound = HoodConstants.SOFT_MIN_ANGLE;
    double upperBound = HoodConstants.SOFT_MAX_ANGLE;
    double steps = 1000;
    double stepSize = (upperBound - lowerBound) / steps;
    
    double theta = lowerBound;
    double velocity = velocityFunction(distance, height, Math.toRadians(lowerBound));

    System.out.println("Starting optimization with initial velocity: " + velocity);

    for (int i = 1; i <= steps; i++) {
        double x = lowerBound + i * stepSize;
        double y = velocityFunction(distance, height, Math.toRadians(x));
        if (y < velocity) {
            velocity = y;
            theta = x;
        }
    }
    System.out.println("Min found at x = " + theta + ", f(x) = " + velocity);
    ballVelocity = velocity;
    azimuth = Radians.of(theta);
    System.out.println("Ball Velocity: " + ballVelocity);
    System.out.println("Azimuth: " + azimuth);
  }

    public void setBETTERShotParams(double height, double distance) {
    double lowerBound = HoodConstants.SOFT_MIN_ANGLE;
    double upperBound = HoodConstants.SOFT_MAX_ANGLE;
    double steps = 1000;
    double stepSize = (upperBound - lowerBound) / steps;
    
    double theta = lowerBound;
    double velocity = velocityFunction(distance, height, lowerBound);
    double previousVelocityDifference = -1;

    for (int i = 1; i <= steps; i++) {
        double x = lowerBound + i * stepSize;
        double y = velocityFunction(distance, height, x);
        double velocityDifference = Math.abs(ballVelocity - y);
        if (previousVelocityDifference < 0 || (velocityDifference < previousVelocityDifference && x < 157)) {
            previousVelocityDifference = velocityDifference;
            velocity = y;
            theta = x;
        }
    }
    // System.out.println("Min found at x = " + theta + ", f(x) = " + velocity);
    ballVelocity = velocity;
    azimuth = Radians.of(theta);
  }

  public AngularVelocity getDesiredVelocity() {
    setShotParams(0, 10);
    //double desiredVelocity = 2 * ballVelocity / (FlywheelConstants.FLYWHEEL_DIAMETER / 12 * Math.PI) + lossFunction();
    // double desiredVelocity = 2 * ballVelocity + lossFunction();
    double desiredRPM = 60 / (Inches.of(FlywheelConstants.FLYWHEEL_DIAMETER).in(Meters) * Math.PI) * FeetPerSecond.of(ballVelocity).in(MetersPerSecond);
    System.out.println("Desired Ball Velocity (ft/sec): " + ballVelocity);
    System.out.println("Desired Flywheel Velocity (rpm): " + desiredRPM);
    return RPM.of(desiredRPM);
  }

  public double lossFunction() {
    return 0 * azimuth.magnitude();
  }

  public Angle getAzimuth() {
    return azimuth;
  }

  public Command setDynamicVelocity () {
    return flyWheel.setSpeed(this::getDesiredVelocity);
  }

  /**
   * set the shooter velocity
   * @param speed speed to set
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity (AngularVelocity speed) {
    System.out.println("Setting velocity to: " + speed);
    return flyWheel.setSpeed(speed); 
  }

  
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return flyWheel.set(dutyCycle);}

  /** Creates a new FlywheelSystem. */
  public FlywheelSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flyWheel.updateTelemetry();
  }

    @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    flyWheel.simIterate();
  }
}


