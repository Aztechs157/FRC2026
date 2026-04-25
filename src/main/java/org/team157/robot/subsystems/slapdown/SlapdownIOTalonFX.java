package org.team157.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.team157.utilities.PosUtils;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class SlapdownIOTalonFX implements SlapdownIO {
  private final Pivot slapdown;
  private final SmartMotorController motor;
  private final DutyCycleEncoder encoder;

  public SlapdownIOTalonFX(SubsystemBase subsystem) {
    this.encoder = new DutyCycleEncoder(SlapdownConstants.PIVOT_ENCODER_ID);
    TalonFX talonfx = new TalonFX(SlapdownConstants.PIVOT_MOTOR_ID);
    // Configure the pivot motor controller for use with YAMS.
    SmartMotorControllerConfig slapdownMotorConfig =
        new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                SlapdownConstants.KP,
                SlapdownConstants.KI,
                SlapdownConstants.KD,
                SlapdownConstants.ANGULAR_VELOCITY,
                SlapdownConstants.ANGULAR_ACCELERATION)
            .withSimClosedLoopController(
                SlapdownConstants.SIM_KP,
                SlapdownConstants.SIM_KI,
                SlapdownConstants.SIM_KD,
                SlapdownConstants.ANGULAR_VELOCITY,
                SlapdownConstants.ANGULAR_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true)
            .withGearing(SlapdownConstants.PIVOT_GEARING)
            .withStatorCurrentLimit(SlapdownConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(SlapdownConstants.RAMP_RATE);

    // Create the pivot's motor controller with the above configuration.
    SmartMotorController smartIntakePivotMotor =
        new TalonFXWrapper(talonfx, DCMotor.getKrakenX44(1), slapdownMotorConfig);

    // Configure the physical characteristics of the pivot.
    PivotConfig slapdownConfig =
        new PivotConfig(smartIntakePivotMotor)
            .withStartingPosition(
                Degrees.of(mapEncoder(SlapdownConstants.MIN_ANGLE, SlapdownConstants.MAX_ANGLE)))
            .withHardLimit(SlapdownConstants.LOWER_HARD_LIMIT, SlapdownConstants.UPPER_HARD_LIMIT)
            .withSoftLimits(SlapdownConstants.LOWER_SOFT_LIMIT, SlapdownConstants.UPPER_SOFT_LIMIT)
            .withMOI(Meters.of(0.75), Kilograms.of(1));

    // Create the pivot system with the above configuration.
    this.slapdown = new Pivot(slapdownConfig);
    this.motor = slapdown.getMotor();
  }
  /**
   * Helper function that maps the pivot's current encoder value to a given range using PosUtils.
   *
   * @param min The minimum value of the remapped range (equivalent to the pivot's minimum encoder
   *     position)
   * @param max The maximum value of the remapped range (equivalent to pivot's maximum encoder
   *     position)
   * @return The current value of the hood's encoder, mapped between 2 numbers based on the
   *     configured minimum and maximum encoder values.
   */
  private double mapEncoder(double min, double max) {
    return PosUtils.mapRange(
        encoder.get(),
        SlapdownConstants.MIN_ENCODER_POSITION,
        SlapdownConstants.MAX_ENCODER_POSITION,
        min,
        max);
  }

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.angleDegrees = slapdown.getAngle().in(Degrees);
    inputs.targetAngleDegrees =
        motor.getMechanismPositionSetpoint().map(a -> a.in(Degrees)).orElse(0.0);
    inputs.encoderPositionRotations = encoder.get();
    inputs.angleFromEncoderDegrees =
        mapEncoder(SlapdownConstants.MIN_ANGLE, SlapdownConstants.MAX_ANGLE);
    inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
    inputs.scaledEncoderPosition = mapEncoder(0, 1);
  }

  // TODO: Evaluate whether or not this method is necessary, as the Hood only needed it for dynamic
  // angle setting.
  @Override
  public Command setTargetAngle(Angle angle) {
    return slapdown.setAngle(angle).finallyDo(() -> stop());
  }

  @Override
  public Command setTargetAngle(Supplier<Angle> angle) {
    return slapdown.setAngle(angle).finallyDo(() -> stop());
  }

  @Override
  public Command stop() {
    return slapdown.set(0);
  }

  @Override
  public Command set(double dutycycle) {
    return slapdown.set(dutycycle);
  }

  @Override
  public void simIterate() {
    slapdown.simIterate();
  }
}
