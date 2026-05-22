package org.team157.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
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
 * Represents an implementation of the Turret running on a TalonFX-based motor (e.g. a Kraken X44).
 */
public class TurretIOTalonFX implements TurretIO {

    private final Pivot turret;
    private final SmartMotorController motor;
    private final DutyCycleEncoder encoder;
    // motor object for sysID voltage control
    private final TalonFX talonFX;
    // initial voltage for sysID voltage control
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    public TurretIOTalonFX(SubsystemBase subsystem) {
        this.talonFX = new TalonFX(TurretConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
        this.encoder = new DutyCycleEncoder(TurretConstants.ENCODER_ID);

        SmartMotorControllerConfig turretMotorConfig =
                new SmartMotorControllerConfig(subsystem)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(
                                TurretConstants.KP,
                                TurretConstants.KI,
                                TurretConstants.KD,
                                TurretConstants.ANGULAR_VELOCITY,
                                TurretConstants.ANGULAR_ACCELERATION)
                        .withSimClosedLoopController(
                                TurretConstants.SIM_KP,
                                TurretConstants.SIM_KI,
                                TurretConstants.SIM_KD,
                                TurretConstants.SIM_ANGULAR_VELOCITY,
                                TurretConstants.SIM_ANGULAR_ACCELERATION)
                        .withIdleMode(MotorMode.BRAKE)
                        .withMotorInverted(true)
                        .withGearing(TurretConstants.GEARING)
                        .withTelemetry("Turret Motor", TelemetryConstants.TELEMETRY_VERBOSITY)
                        .withStatorCurrentLimit(TurretConstants.CURRENT_LIMIT)
                        .withClosedLoopRampRate(TurretConstants.RAMP_RATE)
                        .withSoftLimit(
                                TurretConstants.LOWER_SOFT_LIMIT, TurretConstants.UPPER_SOFT_LIMIT);

        SmartMotorController smartMotor =
                new TalonFXWrapper(talonFX, DCMotor.getKrakenX44(1), turretMotorConfig);

        PivotConfig turretConfig =
                new PivotConfig(smartMotor)
                        .withStartingPosition(
                                Degrees.of(
                                        mapEncoder(
                                                TurretConstants.MIN_ANGLE,
                                                TurretConstants.MAX_ANGLE)))
                        .withHardLimit(
                                TurretConstants.LOWER_HARD_LIMIT, TurretConstants.UPPER_HARD_LIMIT)
                        .withTelemetry("Turret", TelemetryConstants.TELEMETRY_VERBOSITY)
                        .withMOI(Meters.of(0.1), Kilograms.of(4));

        this.turret = new Pivot(turretConfig);
        this.motor = turret.getMotor();

        // Refresh-mutate-apply: read whatever YAMS configured during construction, flip only the
        // enable/wrap flags we care about, then write back. Calling apply() on a fresh builder
        // would overwrite thresholds with the default zero and brick the mechanism.
        var configurator = talonFX.getConfigurator();
        var softLimits = new SoftwareLimitSwitchConfigs();
        configurator.refresh(softLimits);
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitEnable = true;
        configurator.apply(softLimits);

        var closedLoopGeneral = new ClosedLoopGeneralConfigs();
        configurator.refresh(closedLoopGeneral);
        closedLoopGeneral.ContinuousWrap = false;
        configurator.apply(closedLoopGeneral);
    }

    /** Helper that maps the encoder position to an angle in degrees using PosUtils. */
    private double mapEncoder(double min, double max) {
        return PosUtils.mapRange(
                encoder.get(),
                TurretConstants.MIN_ENCODER_POSITION,
                TurretConstants.MAX_ENCODER_POSITION,
                min,
                max);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityDegreesPerSecond =
                motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.angleDegrees = turret.getAngle().in(Degrees);
        inputs.encoderPositionRotations = encoder.get();
        inputs.angleFromEncoderDegrees =
                mapEncoder(TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE);
        inputs.scaledEncoderPosition = mapEncoder(0, 1);
        inputs.targetAngleDegrees = Turret.trackingAngle.in(Degrees);
    }

    @Override
    public void stop() {
        turret.setDutyCycleSetpoint(0);
    }

    // Margin in degrees to leave between the current angle and the soft limit before clamping the
    // SysId voltage to zero. Sized to absorb one scheduler tick of motion plus braking distance.
    private static final double SYSID_LIMIT_MARGIN_DEGREES = 5.0;

    @Override
    public void setVoltage(double volts) {
        double angleDegrees = turret.getAngle().in(Degrees);
        double lowerLimit = TurretConstants.LOWER_SOFT_LIMIT.in(Degrees);
        double upperLimit = TurretConstants.UPPER_SOFT_LIMIT.in(Degrees);
        if ((volts > 0 && angleDegrees >= upperLimit - SYSID_LIMIT_MARGIN_DEGREES)
                || (volts < 0 && angleDegrees <= lowerLimit + SYSID_LIMIT_MARGIN_DEGREES)) {
            volts = 0;
        }
        talonFX.setControl(voltageRequest.withOutput(volts));
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
