package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO extends SubsystemDataProcessor.IODataRefresher {
    default void updateInputs(WristIOInputs inputs) {}

    @AutoLog
    class WristIOInputs {
        // Angle is relative to starting position
        public Rotation2d wristAngle = Rotation2d.kZero;

        public double wristAppliedVolts;
        public double wristSupplyCurrentAmps;
        public double wristStatorCurrentAmps;
        public double wristAngularVelocityRadPerSec;
        public double wristAngularAccelerationRadPerSecSquared;
        public double wristMotorTemp;
    }

    default void setTargetAngle(Rotation2d target) {}

    default void resetWristAngle(Rotation2d angle) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
