package org.team157.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team157.robot.Constants;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Represents an implementation of the Flywheel running
 * on TalonFX-based motors (e.g. Kraken X60).
 */
public class FlywheelIOTalonFX implements FlywheelIO {

    private final FlyWheel flywheel;
    private final SmartMotorController motor;

    public FlywheelIOTalonFX(SubsystemBase subsystem) {
        TalonFX talonFX = new TalonFX(FlywheelConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
        TalonFX followerTalonFX = new TalonFX(FlywheelConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);

        SmartMotorControllerConfig flywheelMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(FlywheelConstants.KP, FlywheelConstants.KI, FlywheelConstants.KD,
                FlywheelConstants.ANGULAR_VELOCITY, FlywheelConstants.ANGULAR_ACCELERATION)
            .withFeedforward(new SimpleMotorFeedforward(FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KA))
            .withSimClosedLoopController(FlywheelConstants.SIM_KP, FlywheelConstants.SIM_KI, FlywheelConstants.SIM_KD,
                FlywheelConstants.ANGULAR_VELOCITY, FlywheelConstants.ANGULAR_ACCELERATION)
            .withSimFeedforward(new SimpleMotorFeedforward(FlywheelConstants.SIM_KS, FlywheelConstants.SIM_KV, FlywheelConstants.SIM_KA))
            .withGearing(FlywheelConstants.GEARING)
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(FlywheelConstants.CURRENT_LIMIT)
            .withClosedLoopRampRate(FlywheelConstants.RAMP_RATE)
            .withFollowers(Pair.of(followerTalonFX, true));

        SmartMotorController smartMotor = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), flywheelMotorConfig);

        FlyWheelConfig flywheelConfig = new FlyWheelConfig(smartMotor)
            .withDiameter(FlywheelConstants.FLYWHEEL_DIAMETER)
            .withMass(FlywheelConstants.FLYWHEEL_MASS)
            .withSoftLimit(FlywheelConstants.FLYWHEEL_RPM_LIMIT_LOWER, FlywheelConstants.FLYWHEEL_RPM_LIMIT_UPPER);

        this.flywheel = new FlyWheel(flywheelConfig);
        this.motor = flywheel.getMotor();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs, AngularVelocity flywheelSetpoint) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityRPM = flywheel.getSpeed().in(RPM);
        inputs.targetVelocityRPM = flywheelSetpoint.in(RPM);
    }

    @Override
    public void stop() {
        flywheel.setDutyCycleSetpoint(0);
    }

    @Override
    public Command set(double dutyCycle) {
        return flywheel.set(dutyCycle);
    }

    @Override
    public Command setVelocity(AngularVelocity velocity) {
        return flywheel.setSpeed(velocity);
    }

    @Override
    public Command setVelocity(Supplier<AngularVelocity> velocity) {
        return flywheel.setSpeed(velocity);
    }

    @Override
    public void simIterate() {
        flywheel.simIterate();
    }
}
