package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.TurretMath;

public class ScoringFactory {
  Shooter shooter;
  Turret turret;
  Elevator elevator;
  Indexer indexer;
  Intake intake;
  CommandSwerveDrivetrain drivetrain;
  boolean isRedAlliance;
  TurretMath turretMath;

  public ScoringFactory(Shooter s, Elevator e, Indexer i, Intake in, CommandSwerveDrivetrain d, boolean a, TurretMath t) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.intake = in;
    this.drivetrain = d;
    this.isRedAlliance = a;
    this.turretMath = t;
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    return Commands.run(() -> {
      double rps = turretMath.turretRPS;
      SmartDashboard.putNumber("Shooter Velocity (RPS)", rps);
      shooter.leftShooterMotor.setControl(m_request.withVelocity(rps).withFeedForward(.5)); // .5
      if (shooter.leftShooterMotor.getVelocity().getValueAsDouble() > rps - (rps * 0.03)) { // 97
        elevator.elevatorMotor.set(-rps / 100);
        indexer.indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
        intake.intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
      } else {
        elevator.elevatorMotor.set(0);
        indexer.indexerMotor.set(0);
        intake.intakeMotor.set(0);
      }
    }, shooter, elevator, indexer, intake);
  }

  public Command reverseSystem() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    return Commands.run(() -> {
      double rps = -50;
      SmartDashboard.putNumber("Shooter Velocity (RPS)", rps);
      shooter.leftShooterMotor.setControl(m_request.withVelocity(rps).withFeedForward(.5));
      elevator.elevatorMotor.set(-rps / 100);
      indexer.indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED);
      intake.intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
    }, shooter, elevator, indexer, intake);
  }

  public Command stopShooter() {
    return Commands.run(() -> {
      shooter.leftShooterMotor.set(0);
      elevator.elevatorMotor.set(0);
      indexer.indexerMotor.set(0);
      intake.intakeMotor.set(0);
    }, shooter, elevator, indexer, intake);
  }
}