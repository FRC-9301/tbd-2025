package frc.robot.subsystems.arm.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.Constants.ArmConstants;

public class WristIOTalonFX implements WristIO {
    private TalonFX wristL;
    private TalonFX wristR;

    DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<Voltage> wristVoltage;
    private final StatusSignal<Current> wristSupplyCurrent;
    private final StatusSignal<Current> wristStatorCurrent;
    private final StatusSignal<Temperature> wristTemperature;
    private final StatusSignal<AngularVelocity> wristAngularVelocity;
    private final StatusSignal<AngularAcceleration> wristAngularAcceleration;

    public WristIOTalonFX() {
        Follower followerWithoutInverse = new Follower(23 , false);
        wristL = new TalonFX(23);
        wristR = new TalonFX(24);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimit = 80.0;

        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Slot0.kS = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.WRIST_ACCELERATION_CONSTRAINT;
        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.WRIST_VELOCITY_CONSTRAINT;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        wristL.getConfigurator().apply(config);
        wristR.getConfigurator().apply(config);

        wristR.setControl(followerWithoutInverse);

        wristPosition = wristL.getRotorPosition();
        wristVoltage = wristL.getMotorVoltage();
        wristSupplyCurrent = wristL.getSupplyCurrent();
        wristStatorCurrent = wristL.getStatorCurrent();
        wristTemperature = wristL.getDeviceTemp();
        wristAngularVelocity = wristL.getRotorVelocity();
        wristAngularAcceleration = wristL.getAcceleration();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristAngle =
                Rotation2d.fromRadians(wristPosition.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT);

        inputs.wristAppliedVolts = wristVoltage.getValueAsDouble();
        inputs.wristSupplyCurrentAmps = wristSupplyCurrent.getValueAsDouble();
        inputs.wristStatorCurrentAmps = wristStatorCurrent.getValueAsDouble();
        inputs.wristMotorTemp = wristTemperature.getValueAsDouble();

        inputs.wristAngularVelocityRadPerSec =
                wristAngularVelocity.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT;

        inputs.wristAngularAccelerationRadPerSecSquared =
                wristAngularAcceleration.getValueAsDouble() * ArmConstants.WRIST_POSITION_COEFFICIENT;
    }

    @Override
    public void setTargetAngle(Rotation2d target) {
        wristL.setControl(positionVoltage.withPosition(target.getRadians() / ArmConstants.WRIST_POSITION_COEFFICIENT));
    }

    @Override
    public void resetWristAngle(Rotation2d angle) {
        wristL.setPosition(angle.getRadians() / ArmConstants.WRIST_POSITION_COEFFICIENT);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        wristL.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        wristL.setNeutralMode(neutralMode);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
                wristPosition,
                wristVoltage,
                wristSupplyCurrent,
                wristStatorCurrent,
                wristTemperature,
                wristAngularVelocity,
                wristAngularAcceleration);
    }
}
