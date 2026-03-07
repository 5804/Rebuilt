package frc.robot.commandfactories;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class ScoringFactory {
  Shooter shooter;
  Elevator indexer;

  public ScoringFactory(Shooter s, Elevator i) {
    this.shooter = s;
    this.indexer = i;
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    double bot_dist = 59; // ~98 is the max velocity for 1:1
    double shoot_set = (bot_dist * .68 + 13) / 2 + 6;

    return Commands.run(() -> {
      shooter.leftShooterMotor.setControl(m_request.withVelocity(bot_dist).withFeedForward(.5)); // .5
      if (shooter.leftShooterMotor.getVelocity().getValueAsDouble() > bot_dist-(bot_dist * 0.03)) { // 97
        indexer.elevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
        
      } else {
        indexer.elevatorMotor.set(0);
      }
    }, shooter, indexer);
  }

  public Command stopShooter() {
    return Commands.run(() -> {
      shooter.leftShooterMotor.set(0);
      indexer.elevatorMotor.set(0);
    }, shooter, indexer);
  }
}