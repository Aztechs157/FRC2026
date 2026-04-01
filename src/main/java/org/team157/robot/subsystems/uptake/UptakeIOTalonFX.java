package org.team157.robot.subsystems.uptake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team157.robot.Constants;
import org.team157.robot.Constants.TelemetryConstants;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class UptakeIOTalonFX implements UptakeIO {

    private final FlyWheel uptake;
    private final SmartMotorController motor;

    public UptakeIOTalonFX(SubsystemBase subsystem) {
        TalonFX talonfx = new TalonFX(UptakeConstants.MOTOR_ID, Constants.RIO_CAN_BUS);
        TalonFX followerTalonfx = new TalonFX(UptakeConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);

        SmartMotorControllerConfig uptakeRollerMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("UptakeRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withGearing(1)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(UptakeConstants.CURRENT_LIMIT)
            .withGearing(UptakeConstants.GEARING)
            .withFollowers(Pair.of(followerTalonfx, false));

        SmartMotorController smartRollerMotor = new TalonFXWrapper(talonfx, DCMotor.getKrakenX44(1),
            uptakeRollerMotorConfig);

        FlyWheelConfig uptakeRollerConfig = new FlyWheelConfig(smartRollerMotor)
            .withTelemetry("Uptake", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withMass(Kilograms.of(0.5))
            .withDiameter(Inches.of(2));

        this.uptake = new FlyWheel(uptakeRollerConfig);
        this.motor = uptake.getMotor();
    }

    @Override
    public void updateInputs(UptakeIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.uptakeRunning = !uptake.gte(DegreesPerSecond.of(5)).getAsBoolean();
    }

    @Override
    public Command stop() {
        return uptake.set(0);
    }

    @Override
    public Command set(double dutyCycle) {
        return uptake.set(dutyCycle);
    }

    @Override
    public void simIterate() {
        uptake.simIterate();
    }
}
