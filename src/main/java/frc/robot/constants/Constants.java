package frc.robot.constants;

import edu.wpi.first.units.Units;


public final class Constants {
    public static final class SuperstructureConstants {
        public static final double X_OFFSET_FROM_TAG_FOR_L1_BASE_SCORING_INCHES = 21.0;
        public static final double X_OFFSET_FROM_TAG_FOR_L1_TOP_SCORING_INCHES = 19.25;

        public static final double X_OFFSET_FROM_TAG_FOR_SCORING_INCHES = 22.0;
        public static final double X_OFFSET_FROM_TAG_FOR_INTAKING_ALGAE_INCHES = 18.0;
        public static final double X_OFFSET_FROM_TAG_FOR_INTERMEDIATE_INTAKING_ALGAE_INCHES = 30.0;
        public static final double X_OFFSET_FROM_TAG_FOR_BACKOUT_INTAKING_ALGAE_INCHES = 50.0;
        public static final double X_OFFSET_FROM_TAG_FOR_L1_BACKOUT_INCHES = 10.0;

        public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES = 6.5;
        public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_L1_INCHES = 9.0;
    }


    public static final class ArmConstants {
        public static final double ZERO_VELOCITY_TIME_PERIOD = 0.5;

        public static final double HOME_TIMEOUT_SECONDS = 3;

        public static final double MOVE_TO_TIMEOUT_SECONDS = 3;

        public static final double SHOULDER_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND = 0.08;
        public static final double WRIST_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND = 0.03;

        public static final double SHOULDER_SETPOINT_TOLERANCE_DEGREES = 1.0;
        public static final double WRIST_SETPOINT_TOLERANCE_DEGREES = 2.0;

        public static final double SHOULDER_GEAR_RATIO = (7.75) * (58 / 18) * (32 / 14);
        public static final double SHOULDER_POSITION_COEFFICIENT = 2 * Math.PI / SHOULDER_GEAR_RATIO;

        public static final double SHOULDER_ACCELERATION = Units.Radians.convertFrom(600, Units.Degree);
        public static final double SHOULDER_VELOCITY = Units.Radian.convertFrom(1000, Units.Degree);

        public static final double WRIST_ACCELERATION = Units.Radian.convertFrom(4500, Units.Degree);
        public static final double WRIST_VELOCITY = Units.Radian.convertFrom(2000, Units.Degree);

        public static final double WRIST_ACCELERATION_CONSTRAINT =
                WRIST_ACCELERATION / ArmConstants.WRIST_POSITION_COEFFICIENT;
        public static final double WRIST_VELOCITY_CONSTRAINT = WRIST_VELOCITY / ArmConstants.WRIST_POSITION_COEFFICIENT;

        public static final double SHOULDER_ACCELERATION_CONSTRAINT =
                SHOULDER_ACCELERATION / SHOULDER_POSITION_COEFFICIENT;
        public static final double SHOULDER_VELOCITY_CONSTRAINT = SHOULDER_VELOCITY / SHOULDER_POSITION_COEFFICIENT;

        public static final double WRIST_GEAR_RATIO = (7.75) * (22 / 14);
        public static final double WRIST_POSITION_COEFFICIENT = 2 * Math.PI / WRIST_GEAR_RATIO;

        public static final double SHOULDER_ZEROING_DUTY_CYCLE = -0.05;
        public static final double WRIST_ZEROING_DUTY_CYCLE = 0.07;

        public static final double SHOULDER_DRIVEN_HOME_ANGLE_DEGREES = -1.647;
        public static final double WRIST_DRIVEN_HOME_ANGLE_DEGREES = 136.0;
        public static final double SHOULDER_BUTTON_HOME_ANGLE_DEGREES = -0.5;
        public static final double WRIST_BUTTON_HOME_ANGLE_DEGREES = 134.74;
    }

    public static final class ClimberConstants {
        public static final double INTAKE_MOTOR_VOLTAGE = 12.0;
        public static final double REJECT_MOTOR_VOLTAGE = -12.0;

        public static final double CLIMBER_PULLEY_DIAMETER = Units.Meter.convertFrom(0.25 * 12 / Math.PI, Units.Inch);
        public static final double CLIMBER_GEAR_RATIO = (66.0 / 9.0) * (50.0 / 14.0);
        public static final double CLIMBER_ACCELERATION = Units.Meter.convertFrom(40.0, Units.Inch);
        public static final double CLIMBER_VELOCITY = Units.Meter.convertFrom(10.0, Units.Inch);
        public static final double CLIMBER_CARRIAGE_INTAKE_POSITION = Units.Meter.convertFrom(7.5, Units.Inch);
        public static final double CLIMBER_CARRIAGE_SETPOINT_TOLERANCE = Units.Meter.convertFrom(0.1, Units.Inch);
        public static final double CLIMBER_CARRIAGE_CAGE_SUPPLY_CURRENT_THRESHOLD = 15.0;
        public static final double CLIMBER_CARRIAGE_CAGE_VELOCITY_RPS_THRESHOLD = 70.0;
        public static final double CLIMBER_DRIVEN_HOME_RESET_POSITION_METERS =
                Units.Meter.convertFrom(-0.04, Units.Inch);

        public static final double CLIMBER_BUTTON_HOME_RESET_POSITION_METERS = 0.0;

        public static final double CLIMBER_ZERO_VELOCITY_THRESHOLD_METERS_PER_SECOND =
                Units.Meter.convertFrom(0.1, Units.Inch);
        public static final double CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 0.15;
        public static final double CLIMBER_DUTY_CYCLE_FOR_ZEROING = -0.07;

        public static final double CLIMBER_POSITION_COEFFICIENT =
                Math.PI * CLIMBER_PULLEY_DIAMETER / CLIMBER_GEAR_RATIO;

        public static final double CLIMBER_ACCELERATION_CONSTRAINT =
                CLIMBER_ACCELERATION / ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
        public static final double CLIMBER_VELOCITY_CONSTRAINT =
                CLIMBER_VELOCITY / ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
    }

    public static final class IntakeConstants {
        public static final class CollectingVoltages {
            public static final double COLLECTING_VERT_CORAL_VOLTAGE = 5.0;
            public static final double COLLECTING_TOP_ALGAE_VOLTAGE = 4.0;
        }

        public static final class EjectingVoltages {
            public static final double EJECTING_VERTICAL_CORAL_VOLTAGE_L2 = -2;
            public static final double EJECTING_VERTICAL_CORAL_VOLTAGE_L1 = -2;
            public static final double EJECTING_PROCESSOR_TOP_ALGAE_VOLTAGE = -1.0;
        }

        public static final class HoldingVoltages {
            public static final double HOLDING_TOP_ALGAE_VOLTAGE = 1.0;
        }

        // public static final class IndexingVoltages {
        //         public static final double INDEXING_CORAL_FORWARD_VERTICAL_VOLTAGE = 0;
        //         public static final double INDEXING_CORAL_BACKWARD_VERTICAL_VOLTAGE = 0;
        // }

        public static final double TOP_ROLLER_CURRENT_THRESHOLD_FOR_ALGAE_DETECTION = 9.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_ALLOWANCE = -80.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_INTAKING = -20.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_HOLDING = -70.0;
     }
}