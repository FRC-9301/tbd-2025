package frc.robot.subsystems.arm.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.Constants.ArmConstants;

public class ShoulderIOTalonFX implements ShoulderIO {
    public final TalonFX shoulderL;
    public final TalonFX shoulderR;

    VoltageOut voltageOut = new VoltageOut(0.0);
    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0).withEnableFOC(true);

    private final StatusSignal<Angle> shoulderAngle;
    private final StatusSignal<Voltage> shoulderAppliedVolts;
    private final StatusSignal<Current> shoulderSupplyCurrentAmps;
    private final StatusSignal<Current> shoulderStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> shoulderAngularVelocityRadPerSec;
    private final StatusSignal<AngularAcceleration> shoulderAngularAccelerationRadPerSecSquared;
    private final StatusSignal<Temperature> shoulderLTemp;
    private final StatusSignal<Temperature> shoulderRTemp;

    public ShoulderIOTalonFX() {
        Follower followerWithoutInverse = new Follower(21 , false);
        shoulderL = new TalonFX(21);
        shoulderR = new TalonFX(22);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimit = 90.0;

        config.Slot0.kP = 10.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Slot0.kS = 0.0;

        config.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_GEAR_RATIO;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = 4;
        config.MotionMagic.MotionMagicCruiseVelocity = 4;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shoulderL.getConfigurator().apply(config);
        shoulderR.getConfigurator().apply(config);

        shoulderR.setControl(followerWithoutInverse); // Set shoulderR to follow shoulderL without inversion

        shoulderAngle = shoulderL.getPosition();
        shoulderAppliedVolts = shoulderL.getMotorVoltage();
        shoulderSupplyCurrentAmps = shoulderL.getSupplyCurrent();
        shoulderStatorCurrentAmps = shoulderL.getStatorCurrent();
        shoulderAngularVelocityRadPerSec = shoulderL.getRotorVelocity();
        shoulderAngularAccelerationRadPerSecSquared = shoulderL.getAcceleration();
        shoulderLTemp = shoulderL.getDeviceTemp();
        shoulderRTemp = shoulderR.getDeviceTemp();
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        inputs.shoulderAngle =
                Rotation2d.fromRadians(shoulderAngle.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT);

        inputs.shoulderAppliedVolts = shoulderAppliedVolts.getValueAsDouble();

        inputs.shoulderSupplyCurrentAmps = shoulderSupplyCurrentAmps.getValueAsDouble();
        inputs.shoulderStatorCurrentAmps = shoulderStatorCurrentAmps.getValueAsDouble();
        inputs.shoulderAngularVelocityRadPerSec =
                shoulderAngularVelocityRadPerSec.getValueAsDouble() * ArmConstants.SHOULDER_POSITION_COEFFICIENT;
        inputs.shoulderAngularAccelerationRadPerSecSquared =
                shoulderAngularAccelerationRadPerSecSquared.getValueAsDouble()
                        * ArmConstants.SHOULDER_POSITION_COEFFICIENT;

        inputs.shoulderLTemp = shoulderLTemp.getValueAsDouble();
        inputs.shoulderRTemp = shoulderRTemp.getValueAsDouble();
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        shoulderL.setControl(
                motionMagicVoltage.withPosition(target.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT));
    }

    @Override
    public void resetShoulderAngle(Rotation2d angle) {
        shoulderL.setPosition(angle.getRadians() / ArmConstants.SHOULDER_POSITION_COEFFICIENT);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        shoulderL.setControl(dutyCycleOut.withOutput(dutyCycle));
        // Only main motor needs to be controlled, others will follow
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        shoulderL.setNeutralMode(neutralMode);
        shoulderR.setNeutralMode(neutralMode);
    }

    @Override
    public void setVoltage(double volts) {
        shoulderL.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                shoulderAngle,
                shoulderAppliedVolts,
                shoulderSupplyCurrentAmps,
                shoulderStatorCurrentAmps,
                shoulderAngularVelocityRadPerSec,
                shoulderAngularAccelerationRadPerSecSquared,
                shoulderLTemp,
                shoulderRTemp
        );
    }
}
