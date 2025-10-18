package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ArmPosition;

public final class ArmPoseConstants {
        public static final ArmPosition ZEROED = new ArmPosition(Rotation2d.kZero, Rotation2d.kZero);
        public static final ArmPosition STOWED =
                new ArmPosition(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(0));

        public static final ArmPosition UPRIGHT =         
                new ArmPosition(Rotation2d.fromDegrees(90.0), Rotation2d.kZero);

        public static final ArmPosition ARM_UP =
                new ArmPosition(Rotation2d.fromDegrees(90.0), STOWED.getWristAngleRot2d());

        // Coral Input
        public static final ArmPosition GROUND_CORAL =
                new ArmPosition(Rotation2d.fromDegrees(1.5), Rotation2d.fromDegrees(-144));

        public static final ArmPosition MARK_CORAL = new ArmPosition(Rotation2d.kZero, Rotation2d.fromDegrees(0.0));
        public static final ArmPosition STATION_CORAL =
                new ArmPosition(Rotation2d.fromDegrees(69), Rotation2d.fromDegrees(-31));

        // Coral Output
        public static final ArmPosition L2_CORAL_BACK =
                new ArmPosition(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(-20));

        public static final ArmPosition L1_CORAL_FRONT =
                new ArmPosition(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-195));

        // Algae Input
        public static final ArmPosition L2_REEF_ALGAE_BACK =
                new ArmPosition(Rotation2d.fromDegrees(105), Rotation2d.fromDegrees(100));

        public static final ArmPosition GROUND_ALGAE =
                new ArmPosition(Rotation2d.fromDegrees(32), Rotation2d.fromDegrees(-85));
        public static final ArmPosition HP_ALGAE =
                new ArmPosition(Rotation2d.fromDegrees(85.0), Rotation2d.fromDegrees(-80.0));

        // Algae Output
        public static final ArmPosition PROCESSOR_ALGAE =
                new ArmPosition(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(25.0));

        public static final ArmPosition HOLD_CORAL_TELEOP =
                new ArmPosition(Rotation2d.fromDegrees(70.5), Rotation2d.fromDegrees(80.0));
        public static final ArmPosition HOLD_CORAL_AUTO =
                new ArmPosition(Rotation2d.fromDegrees(100), Rotation2d.fromDegrees(100.0));
        public static final ArmPosition HOLD_ALGAE = new ArmPosition(
                Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-20));
        }
