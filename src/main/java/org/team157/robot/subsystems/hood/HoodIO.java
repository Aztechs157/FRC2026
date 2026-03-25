package org.team157.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;


public interface HoodIO {
    
    @AutoLog
    public static class HoodIOInputs {
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double appliedVolts = 0.0;
        public double temperatureCelsius = 0.0;
        public double targetAngleDegrees = 0.0;
        public double angleDegrees = 0.0;
        public double encoderPositionRotations = 0.0;
        public double scaledEncoderPosition = 0.0;
        public double angleFromEncoderDegrees = 0.0;
        public double mechanismVelocityDegreesPerSecond = 0.0;
    }

    default void updateInputs(HoodIOInputs inputs) {}
    
    default void simIterate() {}

    default Command setTargetAngle(Angle angle) {
        return Commands.none();
    }
    default Command setTargetAngle(Supplier<Angle> angle){
        return Commands.none();
    }

    default Command stop() {
        return Commands.none();
    }

    default Command set(double dutycycle) {
        return Commands.none();
    } 

}
