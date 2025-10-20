// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.ArmPoseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainSysId;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.shoulder.ShoulderIO;
import frc.robot.subsystems.arm.shoulder.ShoulderIOTalonFX;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOTalonFX;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric dPadStaright = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController= new CommandXboxController(0);

    private final CommandXboxController operatorController = new CommandXboxController(1);

    //private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final DrivetrainSysId drivetrainSysId = new DrivetrainSysId(drivetrain);

    public final Intake intake = new Intake();

    public final ShoulderIO ShoulderIOTalonFX = new ShoulderIOTalonFX();

    public final WristIO WristIOTalonFX = new WristIOTalonFX();

    public final ArmSubsystem armSubsystem = new ArmSubsystem(ShoulderIOTalonFX, WristIOTalonFX);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Coral L1 Front", Commands.sequence(
            armSubsystem.cmdMoveTo(ArmPoseConstants.L1_CORAL_FRONT),
			intake.coralL1().withTimeout(.1)
		));

        NamedCommands.registerCommand("Coral L2 Back", Commands.sequence(
			armSubsystem.cmdMoveTo(ArmPoseConstants.L2_CORAL_BACK),
            intake.coralL2().withTimeout(.1)
		));

		

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        // Field centric driving with d-pad
        driveController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            dPadStaright.withVelocityX(MaxSpeed * 0.6).withVelocityY(0.0)
        ));
        driveController.pov(90).whileTrue(drivetrain.applyRequest(() ->
            dPadStaright.withVelocityX(0.0).withVelocityY(-MaxSpeed * 0.6)
        ));
        driveController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            dPadStaright.withVelocityX(-MaxSpeed * 0.6).withVelocityY(0.0)
        ));
        driveController.pov(270).whileTrue(drivetrain.applyRequest(() ->
            dPadStaright.withVelocityX(0.0).withVelocityY(MaxSpeed * 0.6)
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrainSysId.sysIdTranslationDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrainSysId.sysIdTranslationDynamic(Direction.kReverse));
        driveController.back().and(driveController.a()).whileTrue(drivetrainSysId.sysIdRotationDynamic(Direction.kForward));
        driveController.back().and(driveController.b()).whileTrue(drivetrainSysId.sysIdRotationDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrainSysId.sysIdTranslationQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrainSysId.sysIdTranslationQuasistatic(Direction.kReverse));
        driveController.start().and(driveController.a()).whileTrue(drivetrainSysId.sysIdRotationQuasistatic(Direction.kForward));
        driveController.start().and(driveController.b()).whileTrue(drivetrainSysId.sysIdRotationQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.resetPigeon());

        drivetrain.registerTelemetry(logger::telemeterize);

        operatorController.rightBumper().onTrue(armSubsystem.cmdTareDriven());

        operatorController.a()
             .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.UPRIGHT))
             .onFalse(armSubsystem.cmdWaitUntilAtSetpoint().andThen(armSubsystem.cmdHome().alongWith(intake.stopMotors())));

        // operatorController.a()
        //      .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.GROUND_CORAL).alongWith(intake.coralCollectGround()))
        //      .onFalse(armSubsystem.cmdWaitUntilAtSetpoint.andThen(cmdHome().alongWith(intake.stopMotors())));
        // operatorController.b()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.L1_CORAL_FRONT))
        //     .whileTrue(intake.coralL1())
        //     .onFalse(armSubsystem.cmdHome());
        // operatorController.x()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.L2_CORAL_BACK));
        //     //.whileTrue(intake.coralL2())
        //     //.onFalse(armSubsystem.cmdHome());
        // operatorController.y()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.L2_REEF_ALGAE_BACK))
        //     .whileTrue(intake.algaeCollect())
        //     .onFalse(armSubsystem.cmdHome());

        
        // operatorController.cross()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.GROUND_CORAL).alongWith(intake.coralCollectGround()))
        //     .onFalse(armSubsystem.cmdHome().alongWith(intake.stopMotors()));
        // operatorController.circle()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.GROUND_ALGAE))
        //     .whileTrue(intake.algaeCollect())
        //     .onFalse(armSubsystem.cmdHome());
        // operatorController.triangle()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.STATION_CORAL))
        //     .whileTrue(intake.coralCollectStation())
        //     .onFalse(armSubsystem.cmdHome());
        // operatorController.square()
        //     .onTrue(armSubsystem.cmdMoveTo(ArmPoseConstants.L2_REEF_ALGAE_BACK))
        //     .whileTrue(intake.algaeCollect())
        //     .onFalse(armSubsystem.cmdHome());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
