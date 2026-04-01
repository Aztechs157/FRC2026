package org.team157.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team157.robot.Constants;
import org.team157.robot.Constants.TelemetryConstants;
import org.team157.utilities.PosUtils;

import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Represents an implementation of the Turret running
 * on a TalonFX-based motor (e.g. a Kraken X44).
 */
public class TurretIOTalonFX implements TurretIO {

    private final Pivot turret;
    private final SmartMotorController motor;
    private final DutyCycleEncoder encoder;

    public TurretIOTalonFX(SubsystemBase subsystem) {
        TalonFX talonFX = new TalonFX(TurretConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
        this.encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);

        SmartMotorControllerConfig turretMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD,
                TurretConstants.ANGULAR_VELOCITY, TurretConstants.ANGULAR_ACCELERATION)
            .withSimClosedLoopController(TurretConstants.SIM_KP, TurretConstants.SIM_KI, TurretConstants.SIM_KD,
                TurretConstants.SIM_ANGULAR_VELOCITY, TurretConstants.SIM_ANGULAR_ACCELERATION)
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(true)
            .withGearing(TurretConstants.GEARING)
            .withTelemetry("Turret Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withStatorCurrentLimit(TurretConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(TurretConstants.RAMP_RATE)
            .withSoftLimit(TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);

        SmartMotorController smartMotor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX44(1), turretMotorConfig);

        PivotConfig turretConfig = new PivotConfig(smartMotor)
            .withStartingPosition(Degrees.of(mapEncoder(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE)))
            .withHardLimit(TurretConstants.LOWER_HARD_LIMIT, TurretConstants.UPPER_HARD_LIMIT)
            .withTelemetry("Turret", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMOI(Meters.of(0.1), Kilograms.of(4));

        this.turret = new Pivot(turretConfig);
        this.motor = turret.getMotor();

        var configurator = talonFX.getConfigurator();
        configurator.refresh(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));
        configurator.refresh(new ClosedLoopGeneralConfigs().withContinuousWrap(false));
    }

    /** Helper that maps the encoder position to an angle in degrees using PosUtils. */
    private double mapEncoder(double min, double max) {
        return PosUtils.mapRange(encoder.get(),
            TurretConstants.MIN_ENCODER_POSITION, TurretConstants.MAX_ENCODER_POSITION,
            min, max);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.angleDegrees = turret.getAngle().in(Degrees);
        inputs.encoderPositionRotations = encoder.get();
        inputs.angleFromEncoderDegrees = mapEncoder(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);
        inputs.scaledEncoderPosition = mapEncoder(0, 1);
        inputs.scaledEncoderPosition = Turret.trackingAngle.in(Degrees);
}

    @Override
    public void stop() {
        turret.setDutyCycleSetpoint(0);
    }

    @Override
    public Command setTargetAngle(Angle angle) {
        return turret.setAngle(angle).finallyDo(() -> stop());
    }

    @Override
    public Command setTargetAngle(Supplier<Angle> angle) {
        return turret.setAngle(angle).finallyDo(() -> stop());
    }

    @Override
    public Command set(double dutyCycle) {
        return turret.set(dutyCycle);
    }

    @Override
    public void simIterate() {
        turret.simIterate();
    }
}
