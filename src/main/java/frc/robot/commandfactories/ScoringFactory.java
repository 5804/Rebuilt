package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class ScoringFactory {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;

  public ScoringFactory(Shooter s, Elevator e, Indexer i) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    return Commands.run(() -> {
      shooter.leftShooterMotor.setControl(m_request.withVelocity(Constants.ShooterConstants.SHOOTER_SPEED).withFeedForward(.5)); // .5
      if (shooter.leftShooterMotor.getVelocity().getValueAsDouble() > Constants.ShooterConstants.SHOOTER_SPEED-(Constants.ShooterConstants.SHOOTER_SPEED * 0.03)) { // 97
        elevator.elevatorMotor.set(-Constants.ShooterConstants.SHOOTER_SPEED/100);
        indexer.indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
      } else {
        elevator.elevatorMotor.set(0);
        indexer.indexerMotor.set(0);
      }
    }, shooter, elevator, indexer);
  }

  public Command stopShooter() {
    return Commands.run(() -> {
      shooter.leftShooterMotor.set(0);
      elevator.elevatorMotor.set(0);
      indexer.indexerMotor.set(0);
    }, shooter, elevator, indexer);
  }
}