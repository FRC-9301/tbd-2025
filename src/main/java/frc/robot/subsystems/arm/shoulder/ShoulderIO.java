package frc.robot.subsystems.arm.shoulder;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO extends SubsystemDataProcessor.IODataRefresher {
    default void updateInputs(ShoulderIOInputs inputs) {}

    @AutoLog
    class ShoulderIOInputs {
        // Angle is relative to horizontal rest position as 0 position
        public Rotation2d shoulderAngle = Rotation2d.kZero;

        public double shoulderAppliedVolts;
        public double shoulderSupplyCurrentAmps;
        public double shoulderStatorCurrentAmps;
        public double shoulderAngularVelocityRadPerSec;
        public double shoulderAngularAccelerationRadPerSecSquared;

        public double shoulderLTemp;
        public double shoulderRTemp;
    }

    default void setTargetAngle(Rotation2d target) {}

    default void resetShoulderAngle(Rotation2d angle) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setVoltage(double voltage) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
