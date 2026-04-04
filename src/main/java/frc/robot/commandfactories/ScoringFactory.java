package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.TurretMath;

public class ScoringFactory extends SubsystemBase {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;
  CommandSwerveDrivetrain drivetrain;
  boolean isRedAlliance;
  TurretMath turretMath;

  boolean reachedSpeed = false;
  public boolean isShooting = false;
  public boolean isReversing = false;
  boolean elevatorToSpeed = false;
  boolean elevatorUnjamming = false;
  boolean indexerUnjamming = false;

  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public ScoringFactory(Shooter s, Elevator e, Indexer i, CommandSwerveDrivetrain d, boolean a, TurretMath t) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.drivetrain = d;
    this.isRedAlliance = a;
    this.turretMath = t;
  }

  @Override
  public void periodic() {
    if (isShooting) {
      double rps = TurretMath.turretRPS;
      SmartDashboard.putNumber("Shooter Velocity (RPS)", rps);
      // shooter.leftShooterMotor.setControl(m_request.withVelocity(rps).withFeedForward(.5));

      if (reachedSpeed || shooter.leftShooterMotor.getVelocity().getValueAsDouble() > rps - (rps * 0.03)) {
        reachedSpeed = true;

        double elevatorVel = Math.abs(elevator.elevatorMotor.getVelocity().getValueAsDouble());
        double targetElevatorRps = rps / 100;
        double indexerVel = Math.abs(indexer.indexerMotor.getVelocity().getValueAsDouble());
        double targetIndexerRps = Math.abs(Constants.IndexerConstants.INDEXER_SPEED);

        if (elevatorUnjamming) {
          elevator.elevatorMotor.set(rps / 100);
          if (elevatorVel >= targetElevatorRps * 0.90) {
            elevatorUnjamming = false;
            elevatorToSpeed = false;
          }
        } else if (!elevatorToSpeed) {
          elevator.elevatorMotor.set(-rps / 100);
          if (elevatorVel < targetElevatorRps * 0.50) {
            elevatorUnjamming = true;
          } else if (elevatorVel >= targetElevatorRps * 0.90) {
            elevatorToSpeed = true;
          }
        } else {
          if (elevatorVel < targetElevatorRps * 0.30) {
            elevatorUnjamming = true;
          } else {
            elevator.elevatorMotor.set(-rps / 100);
          }
        }

        if (indexerUnjamming) {
          indexer.indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED);
          if (indexerVel >= targetIndexerRps * 0.95) {
            indexerUnjamming = false;
          }
        } else {
          indexer.indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
          if (indexerVel < targetIndexerRps * 0.30) {
            indexerUnjamming = true;
          }
        }
      }
    } else if (isReversing) {
      double rps = -TurretMath.turretRPS;
      SmartDashboard.putNumber("Shooter Velocity (RPS)", rps);
      // shooter.leftShooterMotor.setControl(m_request.withVelocity(rps).withFeedForward(.5));
      elevator.elevatorMotor.set(-rps / 100);
      indexer.indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED);
    }
  }

  public Command runShooter() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      isReversing = false;
      isShooting = true;
    }, shooter, elevator, indexer);
  }

  public Command reverseSystem() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      isReversing = true;
      isShooting = false;
    }, shooter, elevator, indexer);
  }

  public Command stopShooter() {
    return Commands.runOnce(() -> {
      reachedSpeed = false;
      elevatorToSpeed = false;
      elevatorUnjamming = false;
      indexerUnjamming = false;
      isShooting = false;
      isReversing = false;
      // shooter.leftShooterMotor.set(0);
      elevator.elevatorMotor.set(0);
      indexer.indexerMotor.set(0);
    }, shooter, elevator, indexer);
  }
}