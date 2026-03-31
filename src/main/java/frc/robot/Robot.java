// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.StateEnums.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commandfactories.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public final Turret turret;
    public final Elevator elevator;
    public final Indexer indexer;
    public final Shooter shooter;
    public final Intake intake;
    public final CommandSwerveDrivetrain drivetrain;
    public final SystemFactory systemFactory;

    public Robot() {
        m_robotContainer = new RobotContainer();
        this.turret = RobotContainer.turret;
        this.elevator = m_robotContainer.elevator;
        this.indexer = m_robotContainer.indexer;
        this.shooter = m_robotContainer.shooter;
        this.intake = m_robotContainer.intake;
        this.drivetrain = RobotContainer.drivetrain;
        this.systemFactory = m_robotContainer.systemFactory;
    }

    private final VelocityVoltage shooterVelReq = new VelocityVoltage(0).withSlot(0);

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        if (!ShooterState.ReachedSpeed && shooter.leftShooterMotor.getVelocity()
                .getValueAsDouble() > TurretMath.turretRPS - (TurretMath.turretRPS * 0.03))
            ShooterState.ReachedSpeed = true;

        if (ShooterState.Running) {
            shooter.leftShooterMotor.setControl(shooterVelReq.withVelocity(TurretMath.turretRPS).withFeedForward(.5));
        } else if (ShooterState.Reversing) {
            shooter.leftShooterMotor.setControl(shooterVelReq.withVelocity(-TurretMath.turretRPS).withFeedForward(.5));
        }

        double elevatorVel = Math.abs(elevator.elevatorMotor.getVelocity().getValueAsDouble());
        if (ElevatorState.Running) {
            if (ElevatorState.Manual) {
                elevator.elevatorMotor.set(ElevatorConstants.ELEVATOR_SPEED);
            } else if (ShooterState.ReachedSpeed) {
                if (ElevatorState.Unjamming) {
                    elevator.elevatorMotor.set(TurretMath.turretRPS / 100);
                    if (elevatorVel >= TurretMath.turretRPS / 100
                            * 0.90) {
                        ElevatorState.Unjamming = false;
                        ElevatorState.ReachedSpeed = false;
                    }
                } else if (!ElevatorState.ReachedSpeed) {
                    elevator.elevatorMotor.set(-TurretMath.turretRPS / 100);
                    if (elevatorVel < TurretMath.turretRPS / 100
                            * 0.50) {
                        ElevatorState.Unjamming = true;
                    } else if (elevatorVel >= TurretMath.turretRPS
                            / 100 * 0.90) {
                        ElevatorState.ReachedSpeed = true;
                    }
                } else {
                    if (elevatorVel < TurretMath.turretRPS / 100
                            * 0.30) {
                        ElevatorState.Unjamming = true;
                    } else {
                        elevator.elevatorMotor.set(-TurretMath.turretRPS / 100);
                    }
                }
            }
        } else if (ElevatorState.Reversing) {
            elevator.elevatorMotor.set(-ElevatorConstants.ELEVATOR_SPEED);
        }

        double indexerVel = Math.abs(indexer.indexerMotor.getVelocity().getValueAsDouble());
        if (IndexerState.Running) {
            if (IndexerState.Manual) {
                indexer.indexerMotor.set(IndexerConstants.INDEXER_SPEED);
            } else if (ShooterState.ReachedSpeed) {
                if (IndexerState.Unjamming) {
                    indexer.indexerMotor.set(-IndexerConstants.INDEXER_SPEED);
                    if (indexerVel >= Math
                            .abs(IndexerConstants.INDEXER_SPEED) * 0.95) {
                        IndexerState.Unjamming = false;
                    }
                } else {
                    indexer.indexerMotor.set(IndexerConstants.INDEXER_SPEED);
                    if (indexerVel < Math
                            .abs(IndexerConstants.INDEXER_SPEED) * 0.30) {
                        IndexerState.Unjamming = true;
                    }
                }
            }
        } else if (IndexerState.Reversing) {
            indexer.indexerMotor.set(-IndexerConstants.INDEXER_SPEED);
        }

        if (IntakeState.Running) {
            intake.intakeMotor.set(IntakeConstants.INTAKE_SPEED);
        } else if (IntakeState.Reversing) {
            intake.intakeMotor.set(-IntakeConstants.INTAKE_SPEED);
        }

        Pose2d robotPose = drivetrain.getState().Pose;
        if (TurretState.Aiming) {
            TurretMath.calculateTarget(RobotContainer.isRedAlliance, drivetrain);
            turret.setYaw(-(robotPose.getRotation().getDegrees() + 90) + TurretMath.turretAngle);
        }

        Pose2d turretPose = robotPose.transformBy(new Transform2d(new Translation2d(-.25, 0), new Rotation2d()));
        int teamNum = RobotContainer.isRedAlliance ? 0 : 1;
        SmartDashboard.putNumber("Dist from Hub",
                Math.hypot(turretPose.getX() - Constants.GlobalConstants.hubPositions[teamNum][0],
                        turretPose.getY() - Constants.GlobalConstants.hubPositions[teamNum][1]));
        TurretMath.calculateTurretMath(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians(),
                drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) { CommandScheduler.getInstance().schedule(m_autonomousCommand); }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        StateEnums.TurretState.Aiming = false;
        systemFactory.stopSystem();
        CommandScheduler.getInstance().cancelAll();
        shooter.runShooter(false);
        turret.aimTurret();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() { CommandScheduler.getInstance().cancelAll(); }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
