// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.team157.robot.Constants;
import org.team157.robot.Constants.ModelConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.robot.subsystems.turret.TurretConstants;
import org.team157.robot.subsystems.vision.VisionSystem;
import org.team157.robot.Robot;
import org.team157.robot.RobotContainer;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSystem extends SubsystemBase {

  private VisionSystem visionSystem;
  private TalonFX motor = new TalonFX(TurretConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);
  public Angle trackingAngle = Degrees.of(0);

  // Configure the turret motor controller for use with YAMS.
  private SmartMotorControllerConfig turretMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD,
          TurretConstants.ANGULAR_VELOCITY, TurretConstants.ANGULAR_ACCELERATION)
      .withSimClosedLoopController(TurretConstants.SIM_KP, TurretConstants.SIM_KI, TurretConstants.SIM_KD,
          TurretConstants.SIM_ANGULAR_VELOCITY, TurretConstants.SIM_ANGULAR_ACCELERATION)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(true)
      .withGearing(TurretConstants.GEARING)
      .withTelemetry("Turret Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withStatorCurrentLimit((TurretConstants.CURRENT_LIMIT))
      .withClosedLoopRampRate((TurretConstants.RAMP_RATE))
      .withSoftLimit((TurretConstants.LOWER_SOFT_LIMIT), (TurretConstants.UPPER_SOFT_LIMIT));

  // Create the turret's motor controller with the above configuration.
  private SmartMotorController smartMotor = new TalonFXWrapper(motor, DCMotor.getKrakenX44(1), turretMotorConfig);

  // Configure the physical characteristics of the turret.
  private PivotConfig turretConfig = new PivotConfig(smartMotor)
      .withStartingPosition(Degrees.of(getScaledPosAngleEncoder()))
      .withHardLimit((TurretConstants.LOWER_HARD_LIMIT), (TurretConstants.UPPER_HARD_LIMIT))
      .withTelemetry("Turret", TelemetryConstants.TELEMETRY_VERBOSITY)
      .withMOI(Meters.of(0.1), Kilograms.of(4));

  // Create the turret pivot system with the above configuration.
  private Pivot turret = new Pivot(turretConfig);

  /** Creates a new TurretSystem. */
  public TurretSystem(VisionSystem visionSystem) {
    this.visionSystem = visionSystem;
    var configurator = motor.getConfigurator();
    configurator
        .refresh(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true));
    configurator.refresh(new ClosedLoopGeneralConfigs().withContinuousWrap(false));
  }

  //////////////////////
  /// TURRET COMMANDS ///
  //////////////////////

  public Command setDefault() {
    return run(() -> set(0));
  }

  /**
   * Set the target angle of the turret.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  public Command setAngleManualOverride(Angle angle) {
    if (RobotContainer.manualOverride) {
      return turret.setAngle(angle);
    } else {
      return Commands.none();
    }
  }

  /**
   * Set the target angle of the turret.
   * 
   * @param angle A supplier to obtain the desired angle to go to.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return turret.setAngle(angle);
  }

  /**
   * Move the arm up and down.
   * 
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return turret.set(dutycycle);
  }

  /**
   * Run sysId on the {@link TurretSystem}.
   * To be used for tuning
   * Default code from YAMS Template
   */
  public Command sysId() {
    return turret.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  /**
   * Track the hub tag using the turret's camera.
   * 
   * @return Command either setting the turret angle to face the tag, or setting
   *         the turret power to 0 if the tag isn't present.
   */
  public Command trackHubTag() {
    return turret.setAngle(this::getAngleToHubFaceTag);
  }

  public Command trackTagGlobalRelative() {
    return turret.setAngle(this::getTrackingAngle);
  }

  /////////////////////
  /// TURRET METHODS //
  /////////////////////

  /**
   * Set the duty cycle output of the turret motor.
   * Primarily used for manual control
   * 
   * @param power The power to be applied to the motor.
   */
  public void runMotor(double power) {
    smartMotor.setDutyCycle(power);
  }

  /**
   * Get the raw position of the turret's encoder.
   * 
   * @return The current position of the turret in encoder rotations.
   */
  public double getPos() {
    return encoder.get();
  }

  public Angle getTrackingAngle() {
    return trackingAngle;
  }

  /**
   * Get the scaled position of the turret from 0 to 1.
   * 
   * @return The position of the turret scaled from 0 to 1.
   */
  public double getScaledPos() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_ENCODER_POSITION, TurretConstants.MAX_ENCODER_POSITION, 0.0,
        1.0);
  }

  /**
   * Get the current angle of the turret, based on the YAMS pivot system.
   * 
   * @return The angle of the turret, in degrees, from -180 to 180, using the YAMS
   *         pivot system.
   */
  public double getScaledPosAngleYAMS() {
    return turret.getAngle().in(Degrees);
  }

  /**
   * Get the current angle of the turret, directly from the encoder value.
   * 
   * @return The angle of the turret, in degrees, from -135 to 135, using the
   *         encoder directly.
   */
  public double getScaledPosAngleEncoder() {
    return PosUtils.mapRange(getPos(), TurretConstants.MIN_ENCODER_POSITION, TurretConstants.MAX_ENCODER_POSITION,
        TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);
  }

  /**
   * Get the current velocity of the turret.
   * 
   * @return The velocity of the turret, in degrees per second.
   */
  public double getVelocity() {
    return smartMotor.getMechanismVelocity().in(DegreesPerSecond);
  }

  /**
   * Calculate the angle the turret needs to turn to face the target tag.
   * 
   * @return The angle the turret needs to rotate to to face the target tag.
   */
  public Angle getAngleToHubFaceTag() {
    // The current angular offset of the tag, relative to the turret camera.
    double tagYaw = visionSystem.getHubTagYawFromTurretCam();
    SmartDashboard.putNumber("Target Yaw", tagYaw);
    // if the target tag is seen, at an arbitrary number indicating no target,
    if (tagYaw != 157357) {
      // Subtract the camera-to-tag angle from the turret angle
      // to find our new setpoint angle to face the tag.
      double finalAngle = getScaledPosAngleYAMS() - tagYaw;
      return Degrees.of(finalAngle);
    } else {
      // If no tag seen, don't move turret.
      return Degrees.of(getScaledPosAngleYAMS());
    }
  }

  /**
   * Calculate the angle the turret needs to turn to face the target tag.
   * 
   * @return The angle the turret needs to rotate to to face the target tag.
   */
  public void updateRelativeAngleToTarget(Pose2d targetPose, Pose2d robotPose) {
    // The current angular offset of the tag, relative to the turret camera.
    visionSystem.setTargetParams(targetPose, robotPose);
    double turretToRobotAngleOffset = VisionSystem.angleToTargetFromTurret + TurretConstants.TURRET_ANGLE_OFFSET;
    if (Robot.isReal()) {
      if (turretToRobotAngleOffset > 180) {
        turretToRobotAngleOffset = turretToRobotAngleOffset - 360;
      }

      if (turretToRobotAngleOffset < -180) {
        turretToRobotAngleOffset = turretToRobotAngleOffset + 360;
      }

      trackingAngle = Degrees.of(turretToRobotAngleOffset);
    } else {
      // Disable turret offset in simulation, as simulated 0 is forward.
      trackingAngle = Degrees.of(VisionSystem.angleToTargetFromTurret);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Send values to NT to display on Elastic.
    /*
     * TODO: look into SmartDashboard alternatives, as it's deprecated,
     * marked for removal along with Shuffleboard for next season.
     * Consider publishing to NT directly.
     */
    if (TelemetryConstants.TELEMETRY_VERBOSITY == TelemetryVerbosity.HIGH) {
      SmartDashboard.putNumber("Turret Pos", getPos());
      SmartDashboard.putNumber("Scaled Turret Pos", getScaledPos());
      SmartDashboard.putNumber("Turret Angle (Encoder)", getScaledPosAngleEncoder());
      SmartDashboard.putNumber("Turret Angle", getScaledPosAngleYAMS());
      SmartDashboard.putNumber("where me going", trackingAngle.magnitude());
    }

    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during 
    // simulation. Updates the turret simulation's values,
    turret.simIterate();
  }

  public Pose3d getBasePose() {
    return new Pose3d(ModelConstants.ORIGIN_TO_TURRET_BASE_OFFSET,
        new Rotation3d(0, 0, Math.toRadians(getScaledPosAngleYAMS())));
  }

  public Transform3d getHoodPivotLocation() {
    return new Transform3d(0.1245 * Math.cos(Math.toRadians(getScaledPosAngleYAMS())),
        0.1245 * Math.sin(Math.toRadians(getScaledPosAngleYAMS())), 0.070,
        new Rotation3d(0, 0, Math.toRadians(getScaledPosAngleYAMS())));
  }

  public Pose3d getHoodPivotPose(Transform3d rotation) {
    return new Pose3d(ModelConstants.ORIGIN_TO_TURRET_BASE_OFFSET, new Rotation3d())
        .transformBy(getHoodPivotLocation())
        .transformBy(rotation);

  }
}
