package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ArmPosition;

public final class ArmPoseConstants {
        public static final ArmPosition ZEROED = new ArmPosition(Rotation2d.kZero, Rotation2d.kZero);
        public static final ArmPosition STOWED =
                new ArmPosition(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(125.0));
        public static final ArmPosition KICKSTAND_POSITION =
                new ArmPosition(Rotation2d.fromDegrees(47.21), Rotation2d.fromDegrees(132.59));

        public static final ArmPosition CLIMB_COLLECT =
                new ArmPosition(Rotation2d.fromDegrees(110), Rotation2d.fromDegrees(-75.0));
        public static final ArmPosition CLIMB_INTERMEDIATE_POSE =
                new ArmPosition(Rotation2d.fromDegrees(-5.0), Rotation2d.fromDegrees(15.0));
        public static final ArmPosition CLIMB_LOCK = new ArmPosition(
                CLIMB_INTERMEDIATE_POSE.getShoulderAngleRot2d(),
                CLIMB_INTERMEDIATE_POSE.getWristAngleRot2d());

        public static final ArmPosition ARM_UP =
                new ArmPosition(Rotation2d.fromDegrees(90.0), STOWED.getWristAngleRot2d());

        // Coral Input
        public static final ArmPosition GROUND_CORAL =
                new ArmPosition(Rotation2d.kZero, Rotation2d.fromDegrees(-2.0));

        public static final ArmPosition MARK_CORAL = new ArmPosition(Rotation2d.kZero, Rotation2d.fromDegrees(0.0));
        public static final ArmPosition STATION_CORAL =
                new ArmPosition(Rotation2d.fromDegrees(69), Rotation2d.fromDegrees(-31));
        public static final ArmPosition MARK_PUNCH =
                new ArmPosition(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(20.0));

        // Coral Output
        public static final ArmPosition L2_CORAL_BACK =
                new ArmPosition(Rotation2d.fromDegrees(107), Rotation2d.fromDegrees(122));

        public static final ArmPosition L1_CORAL_FRONT =
                new ArmPosition(Rotation2d.fromDegrees(37), Rotation2d.fromDegrees(0));

        public static final ArmPosition L1_CORAL_BASE =
                new ArmPosition(Rotation2d.fromDegrees(35.0), Rotation2d.fromDegrees(-8.0));

        public static final ArmPosition L1_CORAL_TOP =
                new ArmPosition(Rotation2d.fromDegrees(43), Rotation2d.fromDegrees(-20));

        public static final ArmPosition L2_CORAL_FRONT =
                new ArmPosition(Rotation2d.fromDegrees(38), Rotation2d.fromDegrees(118));

        // Algae Input
        public static final ArmPosition L2_REEF_ALGAE_BACK =
                new ArmPosition(Rotation2d.fromDegrees(105), Rotation2d.fromDegrees(100));
        public static final ArmPosition L2_REEF_ALGAE_FRONT =
                new ArmPosition(Rotation2d.fromDegrees(57.6), Rotation2d.fromDegrees(-65));
        public static final ArmPosition L2_REEF_ALGAE_BACK_INTERMEDIATE = new ArmPosition(
                L2_REEF_ALGAE_BACK.getShoulderAngleRot2d().minus(Rotation2d.fromDegrees(10.0)),
                L2_REEF_ALGAE_BACK.getWristAngleRot2d());
        public static final ArmPosition L2_REEF_ALGAE_FRONT_INTERMEDIATE = new ArmPosition(
                L2_REEF_ALGAE_FRONT.getShoulderAngleRot2d(),
                L2_REEF_ALGAE_FRONT.getWristAngleRot2d());

        public static final ArmPosition GROUND_ALGAE =
                new ArmPosition(Rotation2d.fromDegrees(32), Rotation2d.fromDegrees(-85));
        public static final ArmPosition MARK_ALGAE =
                new ArmPosition(Rotation2d.fromDegrees(49.31), Rotation2d.fromDegrees(-81.38));
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
