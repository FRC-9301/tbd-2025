package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmPoseConstants;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.arm.shoulder.ShoulderIO;
import frc.robot.subsystems.arm.shoulder.ShoulderIO.ShoulderIOInputs;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIO.WristIOInputs;
import frc.robot.util.ArmPosition;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

    private ArmPosition wantedArmPose;

    private final ShoulderIO shoulderIO;
    private final WristIO wristIO;

    private final ShoulderIOInputs shoulderInputs = new ShoulderIOInputs();
    private final WristIOInputs wristInputs = new WristIOInputs();

    private double shoulderHomeTimeStamp = Double.NaN;
    private double wristHomeTimeStamp = Double.NaN;
    private boolean isShoulderHomed = false;
    private boolean isWristHomed = false;

    private boolean hasInitialHomeCompleted = false;

    public enum WantedState {
        HOME,
        IDLE,
        MOVE_TO_POSITION,
    }

    private enum SystemState {
        HOMING_SHOULDER,
        HOMING_WRIST,
        IDLING,
        MOVING_TO_POSITION
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public ArmSubsystem(ShoulderIO shoulderIO, WristIO wristIO) {
        this.shoulderIO = shoulderIO;
        this.wristIO = wristIO;
        wantedArmPose = ArmPoseConstants.ZEROED;
    }


    @Override
    public void periodic() {
        pollIO();

        systemState = handleStateTransitions();
        Logger.recordOutput("Subsystems/Arm/SystemState", systemState);
        Logger.recordOutput("Subsystems/Arm/WantedState", wantedState);
        Logger.recordOutput("Subsystems/Arm/ReachedSetpoint", reachedSetpoint());

        Rotation2d wantedShoulderAngle;
        Rotation2d wantedWristAngle;

        if (wantedArmPose != null) {
            wantedShoulderAngle = wantedArmPose.getShoulderAngleRot2d();
            wantedWristAngle = wantedArmPose.getWristAngleRot2d();
            Logger.recordOutput("Subsystems/Arm/WantedShoulderAngle", wantedShoulderAngle);
            Logger.recordOutput("Subsystems/Arm/WantedWristAngle", wantedWristAngle);

            SmartDashboard.putNumber("Arm/Wanted Shoulder Angle (deg)", wantedShoulderAngle.getDegrees());
            SmartDashboard.putNumber("Arm/Wanted Wrist Angle (deg)", wantedWristAngle.getDegrees());
        }

        SmartDashboard.putNumber("Arm/Actual Shoulder Angle (deg)", shoulderInputs.shoulderAngle.getDegrees());
        SmartDashboard.putNumber("Arm/Actual Wrist Angle (deg)", wristInputs.wristAngle.getDegrees());

        applyStates();

        previousWantedState = this.wantedState;
    }

    private void pollIO() {
        shoulderIO.refreshData();
        wristIO.refreshData();
    
        shoulderIO.updateInputs(shoulderInputs);
        wristIO.updateInputs(wristInputs);
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (previousWantedState != WantedState.HOME) {
                    isShoulderHomed = false;
                    isWristHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(shoulderInputs.shoulderAngularVelocityRadPerSec)
                                    <= ArmConstants.SHOULDER_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                        if (Double.isNaN(shoulderHomeTimeStamp)) {
                            shoulderHomeTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING_SHOULDER;
                        } else if ((Timer.getFPGATimestamp() - shoulderHomeTimeStamp)
                                >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                            if (!hasInitialHomeCompleted) {
                                hasInitialHomeCompleted = true;
                                tareShoulder();
                                return SystemState.HOMING_WRIST;
                            }

                            if (Math.abs(wristInputs.wristAngularVelocityRadPerSec)
                                    <= ArmConstants.WRIST_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                                if (Double.isNaN(wristHomeTimeStamp)) {
                                    wristHomeTimeStamp = Timer.getFPGATimestamp();
                                    return SystemState.HOMING_WRIST;
                                } else if ((Timer.getFPGATimestamp() - wristHomeTimeStamp)
                                        >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                                    isWristHomed = true;
                                    wristHomeTimeStamp = Double.NaN;
                                    isShoulderHomed = true;
                                    shoulderHomeTimeStamp = Double.NaN;

                                    tareWrist();
                                    hasInitialHomeCompleted = false;

                                    setWantedState(WantedState.IDLE);
                                    return SystemState.IDLING;

                                } else {
                                    return SystemState.HOMING_WRIST;
                                }
                            } else {
                                wristHomeTimeStamp = Double.NaN;
                                return SystemState.HOMING_WRIST;
                            }
                        } else {
                            return SystemState.HOMING_SHOULDER;
                        }
                    } else {
                        shoulderHomeTimeStamp = Double.NaN;
                        return SystemState.HOMING_SHOULDER;
                    }
                } else {
                    return SystemState.HOMING_SHOULDER;
                }
            case IDLE:
                return SystemState.IDLING;
            case MOVE_TO_POSITION:
                return SystemState.MOVING_TO_POSITION;
        }
        return SystemState.IDLING;
    }

    public void applyStates() {
        switch (systemState) {
            case HOMING_SHOULDER:
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case HOMING_WRIST:
                wristIO.setDutyCycle(ArmConstants.WRIST_ZEROING_DUTY_CYCLE);
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case IDLING:
                shoulderIO.setDutyCycle(0);
                wristIO.setDutyCycle(0);
                break;
            case MOVING_TO_POSITION:
                if (isShoulderHomed && isWristHomed) {
                    shoulderIO.setTargetAngle(wantedArmPose.getShoulderAngleRot2d());
                    double wristDeg = wantedArmPose.getWristAngleRot2d().getDegrees();
                    double clampedWristDeg = Math.min(wristDeg, 130.0);
                    wristIO.setTargetAngle(Rotation2d.fromDegrees(clampedWristDeg));
                }
                
                break;
        }
    }

    public void tareAllAxesUsingButtonValues() {
        isShoulderHomed = true;
        isWristHomed = true;
        shoulderIO.resetShoulderAngle(Rotation2d.fromDegrees(ArmConstants.SHOULDER_BUTTON_HOME_ANGLE_DEGREES));
        wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_BUTTON_HOME_ANGLE_DEGREES));
    }

    public void tareAllAxes() {
        tareWrist();
        tareShoulder();
    }

    public void tareWrist() {
            isWristHomed = true;
            wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_DRIVEN_HOME_ANGLE_DEGREES));
    }

    public void tareShoulder() {
                isShoulderHomed = true;
                shoulderIO.resetShoulderAngle(Rotation2d.fromDegrees(ArmConstants.SHOULDER_DRIVEN_HOME_ANGLE_DEGREES));
    }

    public void setNeutralMode(NeutralModeValue neutralModeValue) {
                shoulderIO.setNeutralMode(neutralModeValue);
                wristIO.setNeutralMode(neutralModeValue);
    }

    public boolean hasHomeCompleted() {
        return isShoulderHomed && isWristHomed;
    }

    public Rotation2d getCurrentShoulderPosition() {
            return shoulderInputs.shoulderAngle;
    }

    public Rotation2d getCurrentWristPosition() {
            return wristInputs.wristAngle;
    }

    public ArmPosition getWantedArmPose() {
        return wantedArmPose;
    }

    public boolean reachedSetpoint() {
                return MathUtil.isNear(
                                wantedArmPose.getShoulderAngleRot2d().getDegrees(),
                                shoulderInputs.shoulderAngle.getDegrees(),
                                ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                        && MathUtil.isNear(
                                wantedArmPose.getWristAngleRot2d().getDegrees(),
                                wristInputs.wristAngle.getDegrees(),
                                ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
    }

    public boolean reachedSetpoint(ArmPosition armPosition) {
                return MathUtil.isNear(
                                armPosition.getShoulderAngleRot2d().getDegrees(),
                                shoulderInputs.shoulderAngle.getDegrees(),
                                ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                        && MathUtil.isNear(
                                armPosition.getWristAngleRot2d().getDegrees(),
                                wristInputs.wristAngle.getDegrees(),
                                ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        this.wantedArmPose = armPosition;
    }

    public void setOnlyShoulder(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        var wantedWrist = wantedArmPose.getWristAngleRot2d();
        this.wantedArmPose = new ArmPosition(armPosition.getShoulderAngleRot2d(), wantedWrist);
    }

    //_________________COMMANDS_________________//

    public Command cmdHome() {
        double timeoutSec = ArmConstants.HOME_TIMEOUT_SECONDS;
        return Commands.run(() -> setWantedState(WantedState.HOME), this)
                .until(this::hasHomeCompleted)
                .withTimeout(timeoutSec)
                .andThen(Commands.runOnce(() -> setWantedState(WantedState.IDLE), this))
                .withName("ArmHome");
    }

    public Command cmdIdle() {
        return Commands.runOnce(() -> setWantedState(WantedState.IDLE), this).withName("ArmIdle");
    }

    public Command cmdMoveTo(ArmPosition pose) {
        double timeoutSec = ArmConstants.MOVE_TO_TIMEOUT_SECONDS;
        return Commands.run(() -> setWantedState(WantedState.MOVE_TO_POSITION, pose), this)
                .until(this::reachedSetpoint)
                .withTimeout(timeoutSec)
                .andThen(Commands.runOnce(() -> setWantedState(WantedState.IDLE), this))
                .withName("ArmMoveTo");
    }

    public Command cmdMoveOnlyShoulder(ArmPosition pose) {
        double timeoutSec = ArmConstants.MOVE_TO_TIMEOUT_SECONDS;
        return Commands.run(() -> setOnlyShoulder(WantedState.MOVE_TO_POSITION, pose), this)
                .until(() -> reachedSetpoint(new ArmPosition(pose.getShoulderAngleRot2d(), wantedArmPose.getWristAngleRot2d())))
                .withTimeout(timeoutSec)
                .andThen(Commands.runOnce(() -> setWantedState(WantedState.IDLE), this))
                .withName("ArmMoveOnlyExtShoulder");
    }

    public Command cmdTareUsingButtonValues() {
        return Commands.runOnce(this::tareAllAxesUsingButtonValues, this).withName("ArmTareButtons");
    }

    /** Tare using driven/home values (calls tareWrist then tareExtensionAndShoulder). Instant. */
    public Command cmdTareDriven() {
        return Commands.runOnce(this::tareAllAxes, this).withName("ArmTareDriven");
    }

    /** Set neutral mode (Brake/Coast) for all joints. Instant. */
    public Command cmdSetNeutralMode(NeutralModeValue neutral) {
        return Commands.runOnce(() -> setNeutralMode(neutral), this).withName("ArmSetNeutral");
    }

    public Command cmdWaitUntilAtSetpoint() {
        return Commands.waitUntil(this::reachedSetpoint).withName("ArmWaitUntilAtSetpoint");
    }

    public Command cmdShoulderVoltage(double volts) {
        return Commands.runOnce(() -> shoulderIO.setVoltage(volts), this).withName("ArmShoulderVoltage");
    }
}


