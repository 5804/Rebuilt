// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commandfactories.ScoringFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                     // desired top
    // speed
    private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation
                                                                                                   // per second
    // max angular velocity
    public static double speedMultiplier = 1;
    public static double rotationEnabled = 1;

    public boolean turretAutoLock = false;

    static Optional<Alliance> alliance = DriverStation.getAlliance();
    public static boolean isRedAlliance = alliance.get() == Alliance.Red;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.005) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final ButtonBoard xKeys = new ButtonBoard(64, 1);
    // private final Joystick joystick = new Joystick(2);

    public int climbTriggerHeld = 0;

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static TurretMath turretMath = new TurretMath();
    public static final Turret turret = new Turret(drivetrain, turretMath, isRedAlliance);
    public final Elevator elevator = new Elevator();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    // public Climber climber = new Climber(() -> { return -1 *
    // xKeys.getAxis(2) * climbTriggerHeld; });

    ScoringFactory scoringFactory = new ScoringFactory(shooter, elevator, indexer, drivetrain, isRedAlliance,
            turretMath);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public Command aimTurretStop() {
        return Commands.runOnce(() -> {
            turret.setYaw(0);
        }, turret);
    }

    public RobotContainer() {
        configureBindings();
        RobotController.setBrownoutVoltage(6.2);
        NamedCommands.registerCommand("AimTurret", turret.aimTurret());
        NamedCommands.registerCommand("StopAimTurret", turret.stopAiming());
        NamedCommands.registerCommand("RunShooter", shooter.runShooter().andThen(scoringFactory.runShooter()));
        NamedCommands.registerCommand("StopShooter", shooter.stopShooter().andThen(scoringFactory.stopShooter()));
        NamedCommands.registerCommand("RunIntake", intake.runIntake());
        NamedCommands.registerCommand("StopIntake", intake.stopIntake());
        NamedCommands.registerCommand("RunOuttake", new ParallelCommandGroup(intake.reverseIntake(),
                indexer.reverseIndexer(), elevator.reverseElevator(Constants.ElevatorConstants.ELEVATOR_SPEED)));
        NamedCommands.registerCommand("StopOuttake",
                new ParallelCommandGroup(intake.stopIntake(), indexer.stopIndexer(), elevator.stopElevator()));

        autoChooser.addOption("Middle Left", VABLAmL());
        autoChooser.addOption("Middle Right", VABLAmR());
        autoChooser.addOption("Right Side", VABLARight());
        autoChooser.addOption("Right Passing", VABLARightPassing());
        autoChooser.addOption("TestPath", TestPath());
        autoChooser.addOption("Right Bump", rightBump());
        autoChooser.addOption("Right Bump Return", rightBumpReturn());
        autoChooser.addOption("Backup Left Trench", backupLeftTrench());
        autoChooser.addOption("Shoot Left Trench", shootLeftTrench());
        autoChooser.addOption("Insta Left Trench", instaLeftTrench());
        autoChooser.addOption("Backup Left Bump", BackupLeftBump());
        autoChooser.addOption("Shoot Left Bump", shootLeftBump());



        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);
        LimelightHelpers.setCameraPose_RobotSpace("limelight-front", 0.28, 0.28, 0.21, 0, 30, 0);
        LimelightHelpers.setPipelineIndex("limelight-front", 0);

        LimelightHelpers.setCameraPose_RobotSpace("limelight-back", -0.32, 0, 0.22, 0, 16, 180);
        LimelightHelpers.setPipelineIndex("limelight-back", 0);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // if (climbTriggerHeld == 0) {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-xboxController.getLeftY() * MaxSpeed * speedMultiplier) // Drive forward with
                        // negative Y
                        // (forward)
                        .withVelocityY(-xboxController.getLeftX() * MaxSpeed * speedMultiplier) // Drive left with
                                                                                                // negative X (left)
                        .withRotationalRate(
                                -xboxController.getRightX() * MaxAngularRate * speedMultiplier * rotationEnabled) // Drive
                                                                                                                  // counterclockwise
                                                                                                                  // with
                // negative X (left)
                /*
                 * ? (climbTriggerHeld == 1) : () ->
                 * drive.withVelocityX(-Math.sqrt(joystick.getRawAxis(1)))
                 */
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // xboxController
        xboxController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        xboxController.b().whileTrue(new ParallelCommandGroup(intake.reverseIntake(), indexer.reverseIndexer()));
        xboxController.b().onFalse(new ParallelCommandGroup(intake.stopIntake(), indexer.stopIndexer()));

        // xboxController.rightTrigger().whileTrue(scoringFactory.runShooter());
        // xboxController.rightTrigger().onFalse(scoringFactory.stopShooter());

        xboxController.rightTrigger().whileTrue(turret.aimTurret().andThen(scoringFactory.runShooter()));
        xboxController.rightTrigger().onFalse(scoringFactory.stopShooter().andThen(turret.stopAiming()));

        xboxController.leftTrigger().whileTrue(new ParallelCommandGroup(intake.runIntake(), indexer.runIndexer()));
        xboxController.leftTrigger().onFalse(new ParallelCommandGroup(intake.stopIntake(), indexer.stopIndexer()));

        xboxController.povDown().onTrue(turret.aimTurret());
        xboxController.povRight().onTrue(shooter.runShooter());
        xboxController.povLeft().onTrue(shooter.stopShooter());

        /*
         * // Testing Commands
         * xboxController.y().onTrue(new InstantCommand(() -> {
         * Constants.ShooterConstants.SHOOTER_SPEED += .25 ;}));
         * xboxController.x().onTrue(new InstantCommand(() -> {
         * Constants.ShooterConstants.SHOOTER_SPEED -= .25 ;}));
         * 
         * // Climber Commands (Temp)
         * xboxController.povRight().whileTrue(climber.extendActuator());
         * xboxController.povLeft().whileTrue(climber.retractActuator());
         */

        // X-Keys
        // xKeys.getButton(14).onTrue(turret.aimTurret());
        // xKeys.getButton(15).onTrue(aimTurretStop().andThen(turret.stopAiming()));

        // xKeys.getButton(19).onTrue(new ParallelCommandGroup(intake.reverseIntake(),
        // indexer.reverseIndexer()));
        // xKeys.getButton(19).onFalse(new ParallelCommandGroup(intake.stopIntake(),
        // indexer.stopIndexer()));

        // xKeys.getButton(21).onTrue(new InstantCommand(() ->
        // {CommandScheduler.getInstance().cancelAll();}));

        // xKeys.getButton(3).onTrue(indexer.runIndexer());
        // xKeys.getButton(3).onFalse(indexer.stopIndexer());

        // xKeys.getButton(2).onTrue(elevator.runElevator(Constants.ElevatorConstants.ELEVATOR_SPEED));
        // xKeys.getButton(2).onFalse(elevator.stopElevator());

        // xKeys.getButton(1).onTrue(shooter.runShooter());
        // xKeys.getButton(1).onFalse(shooter.stopShooter());

        // xKeys.getButton(16).onTrue(scoringFactory.reverseSystem());
        // xKeys.getButton(16).onFalse(scoringFactory.stopShooter());

        // X-Keys

        for (int i = 1; i <= 18; i++) {
            if (i <= 8) {
                xKeys.getButton(i).onTrue(turret.changeTurretTarget(i));
            } else if (i <= 10) {
                xKeys.getButton(i).onTrue(turret.changeTurretTarget(i + 1));
            } else {
                xKeys.getButton(i).onTrue(turret.changeTurretTarget(i + 2));
            }
        }

        xKeys.getButton(22).onTrue(intake.runIntake());
        xKeys.getButton(22).onFalse(intake.stopIntake());

        xKeys.getButton(23).onTrue(indexer.runIndexer());
        xKeys.getButton(23).onFalse(indexer.stopIndexer());

        xKeys.getButton(24).onTrue(elevator.runElevator(Constants.ElevatorConstants.ELEVATOR_SPEED));
        xKeys.getButton(24).onFalse(elevator.stopElevator());

        // xKeys.getButton(25).whileTrue(turret.aimTurret().andThen(scoringFactory.runShooter()));
        // xKeys.getButton(25).onFalse(scoringFactory.stopShooter().andThen(turret.stopAiming()));

        // xKeys.getButton(26).whileTrue(shooter.runShooter());
        // xKeys.getButton(26).onFalse(shooter.stopShooter());

        xKeys.getButton(27).onTrue(turret.aimTurret());
        xKeys.getButton(28).onTrue(aimTurretStop().andThen(turret.stopAiming()));

        xKeys.getButton(29).onTrue(new ParallelCommandGroup(intake.reverseIntake(), indexer.reverseIndexer()));
        xKeys.getButton(29).onFalse(new ParallelCommandGroup(intake.stopIntake(), indexer.stopIndexer()));

        xKeys.getButton(30).onTrue(turret.aimTurret().andThen(scoringFactory.runShooter()));
        xKeys.getButton(30).onFalse(scoringFactory.stopShooter().andThen(turret.stopAiming()));

        xKeys.getButton(31).onTrue(new InstantCommand(() -> { CommandScheduler.getInstance().cancelAll(); }));

        
        // Climber
        // climber.setDefaultCommand(climber.setClimberSpeed());
        
        // xKeys.getButton(19).whileTrue(new InstantCommand(() -> { climbTriggerHeld = 1; }));
        // xKeys.getButton(19).whileFalse(new InstantCommand(() -> { climbTriggerHeld = 0; }));

        // xKeys.getButton(20).whileTrue(climber.extendActuator());
        // xKeys.getButton(21).whileTrue(climber.retractActuator());
         

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command VABLAmR() {
        return new PathPlannerAuto("VABLAmR");
    }

    public Command VABLAmL() {
        return new PathPlannerAuto("VABLAmL");
    }

    public Command VABLARight() {
        return new PathPlannerAuto("VABLARight");
    }

    public Command VABLARightPassing() {
        return new PathPlannerAuto("VABLARightPassing");
    }

    public Command TestPath() {
        return new PathPlannerAuto("2MeterTest");
    }

    public Command rightBump() {
        return new PathPlannerAuto("Right Bump");
    }
    public Command rightBumpReturn() {
        return new PathPlannerAuto("Right Bump Return");
    }
    
    public Command shootLeftBump() {
        return new PathPlannerAuto("Shoot Left Bump");
    }
    
    public Command backupLeftTrench() {
        return new PathPlannerAuto("Backup Left Trench");
    }
    
    public Command shootLeftTrench() {
        return new PathPlannerAuto("Shoot Left Trench");
    }
    
    public Command BackupLeftBump() {
        return new PathPlannerAuto("Backup Left Bump");
    }
    
    public Command instaLeftTrench() {
        return new PathPlannerAuto("Insta Left Trench");
    }
}