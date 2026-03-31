package org.team157.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Constants.TelemetryConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HopperIOTalonFX implements HopperIO {

    private final FlyWheel hopper;
    private final SmartMotorController motor;

    public HopperIOTalonFX(SubsystemBase subsystem) {
        TalonFX talonfx = new TalonFX(HopperConstants.MOTOR_ID, Constants.RIO_CAN_BUS);

        SmartMotorControllerConfig hopperRollerMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("HopperRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withGearing(1)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit((HopperConstants.CURRENT_LIMIT))
            .withGearing(HopperConstants.GEARING);

        SmartMotorController smartRollerMotor = new TalonFXWrapper(talonfx, DCMotor.getKrakenX44(1),
            hopperRollerMotorConfig);

        FlyWheelConfig hopperRollerConfig = new FlyWheelConfig(smartRollerMotor)
            .withMass(Kilograms.of(0.5))
            .withDiameter(Inches.of(1))
            .withTelemetry("Hopper", TelemetryConstants.TELEMETRY_VERBOSITY);

        this.hopper = new FlyWheel(hopperRollerConfig);
        this.motor = hopper.getMotor();

    }
        
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.hopperRunning = !hopper.gte(DegreesPerSecond.of(5)).getAsBoolean();
    }


    @Override
    public Command stop() {
        return hopper.set(0);
    }

    @Override
    public Command set(double dutycycle) {
        return hopper.set(dutycycle);
    }

    @Override
    public void simIterate() {
        hopper.simIterate();
    }



}
