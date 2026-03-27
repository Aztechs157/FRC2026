// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team157.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import org.team157.robot.Constants;
import org.team157.robot.Constants.TelemetryConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
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

/**
 * Represents an implementation of the Intake running 
 * on TalonFX-based motors (e.g. 2 Kraken X60s).
 */
public class IntakeIOTalonFX implements IntakeIO {

    private final FlyWheel intake;
    private final SmartMotorController motor;

    public IntakeIOTalonFX(SubsystemBase subsystem){
        TalonFX talonfx = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, Constants.RIO_CAN_BUS);
        TalonFX talonfxFollower = new TalonFX(IntakeConstants.FOLLOWER_MOTOR_ID, Constants.RIO_CAN_BUS);
       
        SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig(subsystem)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withTelemetry("IntakeRollerMotor", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withGearing(1)
            .withMotorInverted(true)
            .withIdleMode(MotorMode.COAST)
            .withStatorCurrentLimit(Amps.of(40))
            .withFollowers(Pair.of(talonfxFollower, true));
        
        // Create the intake's motor controller with the above configuration.
        SmartMotorController smartIntakeMotor = new TalonFXWrapper(talonfx, DCMotor.getKrakenX60(1), intakeMotorConfig);

        FlyWheelConfig intakeConfig = new FlyWheelConfig(smartIntakeMotor)
            .withTelemetry("Intake", TelemetryConstants.TELEMETRY_VERBOSITY)
            .withDiameter(Inches.of(3))
            //TODO: measure mass of the intake roller and update this constant
            .withMass(Kilograms.of(0.5)); 

        // Create the intake roller system with the above configuration.
        this.intake = new FlyWheel(intakeConfig);
        this.motor = intake.getMotor();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
        inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
        inputs.appliedVolts = motor.getVoltage().in(Volts);
        inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
        inputs.mechanismVelocityDegreesPerSecond = motor.getMechanismVelocity().in(DegreesPerSecond);
        inputs.intakeRunning = !intake.gte(DegreesPerSecond.of(5)).getAsBoolean();

    }

    @Override
    public Command stop() {
        return intake.set(0);
    }

    @Override
    public Command set(double dutycycle) {
        return intake.set(dutycycle);
    }

    @Override
    public void simIterate(){
        intake.simIterate();
    }

    
}
