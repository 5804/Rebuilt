// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commandfactories.ScoringFactory;
import frc.robot.TurretMath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    public boolean turretAutoLock = false;

    static Optional<Alliance> alliance = DriverStation.getAlliance();
    public static boolean isRedAlliance = alliance.get() == Alliance.Red;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static TurretMath turretMath = new TurretMath();
    public static final Turret turret = new Turret(drivetrain, turretMath, isRedAlliance);
    public final Elevator elevator = new Elevator();
    public final Indexer indexer = new Indexer();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    public Shooter shooterSpeed = new Shooter();

    ScoringFactory scoringFactory = new ScoringFactory(shooter, elevator, indexer, drivetrain, isRedAlliance,
            turretMath);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private ShuffleboardTab tab1 = Shuffleboard.getTab("Tab1");

    public Command aimTurretStop() {
        return Commands.run(() -> {
            turret.setYaw(0);
        }, turret);
    }

    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("AimTurret", turret.aimTurret());

        NamedCommands.registerCommand("ScoreFor3.5", new ParallelCommandGroup(
                turret.aimTurret(),
                scoringFactory.runShooter()).withTimeout(3.5)
                .andThen(new InstantCommand(() -> {
                    shooter.leftShooterMotor.set(0);
                    elevator.elevatorMotor.set(0);
                    indexer.indexerMotor.set(0);
                })));

        NamedCommands.registerCommand("StopTurretAim", aimTurretStop());
        NamedCommands.registerCommand("RunIntake", intake.runIntake().repeatedly());
        NamedCommands.registerCommand("StopIntake", intake.stopIntake());
        NamedCommands.registerCommand("Shoot", scoringFactory.runShooter().repeatedly());
        NamedCommands.registerCommand("StopShoot", scoringFactory.stopShooter());

        autoChooser.addOption("Two Meter Test", TwoMeterTest());
        autoChooser.addOption("90 Degree Rotation Test", RotationTest());
        autoChooser.addOption("Left Side Auto", LeftSideAuto());
        autoChooser.addOption("Right Side Auto", RightSideAuto());
        autoChooser.addOption("Right Middle Auto", RightMiddleAuto());
        autoChooser.addOption("Right Middle U-Auto", RightMiddleUAuto());

        SmartDashboard.putData("Auto choices", autoChooser);
        tab1.add("Auto Chooser", autoChooser);

        LimelightHelpers.setCameraPose_RobotSpace("limelight-front", 0.305, 0.395, 0.26, 0, 30, 0);
        LimelightHelpers.setPipelineIndex("limelight-front", 0);

        LimelightHelpers.setCameraPose_RobotSpace("limelight-back", -0.305, -0.305, 0.26, 0, 30, 180);
        LimelightHelpers.setPipelineIndex("limelight-back", 0);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.rightBumper().whileTrue(elevator.runElevator());
        joystick.rightBumper().onFalse(elevator.stopElevator());

        joystick.leftBumper().whileTrue(indexer.runIndexer());
        joystick.leftBumper().onFalse(indexer.stopIndexer());

        joystick.a().whileTrue(shooter.runShooter());
        joystick.a().onFalse(shooter.stopShooter());

        joystick.b().whileTrue(new ParallelCommandGroup(intake.reverseIntake(), indexer.reverseIndexer()));
        joystick.b().onFalse(new ParallelCommandGroup(intake.stopIntake(), indexer.stopIndexer()));

        joystick.rightTrigger().whileTrue(scoringFactory.runShooter());
        joystick.rightTrigger().onFalse(scoringFactory.stopShooter());

        joystick.leftTrigger().whileTrue(intake.runIntake());
        joystick.leftTrigger().whileFalse(intake.stopIntake());

        joystick.povUp().onTrue(turret.aimTurret());
        joystick.povDown().onTrue(aimTurretStop());

        joystick.y().onTrue(Commands.runOnce(() -> { Constants.ShooterConstants.SHOOTER_SPEED += 0.2; } ));
        joystick.x().onTrue(Commands.runOnce(() -> { Constants.ShooterConstants.SHOOTER_SPEED -= 0.2; } ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() { return autoChooser.getSelected(); }

    public Command LeftSideAuto() { return new PathPlannerAuto("Left Side Auto"); }

    public Command RightSideAuto() { return new PathPlannerAuto("Right Side Auto"); }

    public Command RightMiddleAuto() { return new PathPlannerAuto("Right Middle Auto"); }

    public Command RightMiddleUAuto() { return new PathPlannerAuto("Right Middle U-Auto"); }

    public Command TwoMeterTest() { return new PathPlannerAuto("2MeterTest"); }

    public Command RotationTest() { return new PathPlannerAuto("90DegreeTest"); }
}