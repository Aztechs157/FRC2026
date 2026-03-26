package org.team157.robot.subsystems.Slapdown;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.team157.robot.Constants.HoodConstants;
import org.team157.robot.Constants.IntakeConstants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.utilities.PosUtils;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        this.encoder = new DutyCycleEncoder(IntakeConstants.PIVOT_ENCODER_ID);
        TalonFX talonfx = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID);
        // Configure the pivot motor controller for use with YAMS.
        SmartMotorControllerConfig slapdownMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(IntakeConstants.KP, IntakeConstants.KI, IntakeConstants.KD,
                IntakeConstants.ANGULAR_VELOCITY, IntakeConstants.ANGULAR_ACCELERATION)
            .withSimClosedLoopController(IntakeConstants.SIM_KP, IntakeConstants.SIM_KI, IntakeConstants.SIM_KD, IntakeConstants.ANGULAR_VELOCITY, IntakeConstants.ANGULAR_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true)
            .withGearing(IntakeConstants.PIVOT_GEARING)
            .withTelemetry("Intake Pivot Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withStatorCurrentLimit(IntakeConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(IntakeConstants.RAMP_RATE);

        // Create the pivot's motor controller with the above configuration.
        SmartMotorController smartIntakePivotMotor = new TalonFXWrapper(talonfx, DCMotor.getKrakenX44(1), slapdownMotorConfig);

        // Configure the physical characteristics of the pivot.
        PivotConfig slapdownConfig = new PivotConfig(smartIntakePivotMotor)
            .withStartingPosition(Degrees.of(mapEncoder(IntakeConstants.MIN_ANGLE, IntakeConstants.MAX_ANGLE)))
            .withHardLimit(IntakeConstants.LOWER_HARD_LIMIT, IntakeConstants.UPPER_HARD_LIMIT)
            .withSoftLimits(IntakeConstants.LOWER_SOFT_LIMIT, IntakeConstants.UPPER_SOFT_LIMIT)
            .withTelemetry("Intake Pivot", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMOI(Meters.of(0.75), Kilograms.of(1));

        // Create the pivot system with the above configuration.
        this.slapdown = new Pivot(slapdownConfig);
        this.motor = slapdown.getMotor();
    }
    /** Helper function that maps the pivot's current encoder value to a given range using PosUtils.
     * 
     * @param min The minimum value of the remapped range (equivalent to the pivot's minimum encoder position)
     * @param max The maximum value of the remapped range (equivalent to pivot's maximum encoder position)
     * @return The current value of the hood's encoder, mapped between 2 numbers based on the configured minimum and maximum encoder values.
     */
    private double mapEncoder(double min, double max){
        return PosUtils.mapRange(encoder.get(), IntakeConstants.MIN_ENCODER_POSITION, 
                    IntakeConstants.MAX_ENCODER_POSITION, min, max);
    }

    @Override
    public void updateInputs(SlapdownIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.angleDegrees = slapdown.getAngle().in(Degrees);
        inputs.targetAngleDegrees = motor.getMechanismPositionSetpoint().map(a -> a.in(Degrees)).orElse(0.0);
        inputs.encoderPositionRotations = encoder.get();
        inputs.angleFromEncoderDegrees = mapEncoder(HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);
        inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.scaledEncoderPosition = mapEncoder(0, 1);
    }

    // TODO: Evaluate whether or not this method is necessary, as the Hood only needed it for dynamic angle setting.
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
    public void simIterate(){
        slapdown.simIterate();
    }

}
