package frc.robot.commandfactories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SystemFactory extends SubsystemBase {
  Shooter shooter;
  Elevator elevator;
  Indexer indexer;
  Intake intake;
  CommandSwerveDrivetrain drivetrain;

  public SystemFactory(Shooter s, Elevator e, Indexer i, Intake in, CommandSwerveDrivetrain d) {
    this.shooter = s;
    this.elevator = e;
    this.indexer = i;
    this.intake = in;
    this.drivetrain = d;
  }

  public Command runScore() { return elevator.runElevator(false).andThen(indexer.runIndexer(false)); }

  public Command stopScore() { return elevator.stopElevator().andThen(indexer.stopIndexer()); }

  public Command runOuttake() {
    return 
      elevator.reverseElevator(false)
      .andThen(indexer.reverseIndexer(false))
      .andThen(intake.reverseIntake());
  }

  public Command stopOuttake() {
    return
      elevator.stopElevator()
      .andThen(indexer.stopIndexer())
      .andThen(intake.stopIntake());
  }

  public Command reverseSystem() {
    return
      shooter.reverseShooter(false)
      .andThen(elevator.reverseElevator(false))
      .andThen(indexer.reverseIndexer(false))
      .andThen(intake.reverseIntake());
  }

  public Command stopSystem() {
    return
      shooter.stopShooter()
      .andThen(elevator.stopElevator())
      .andThen(indexer.stopIndexer())
      .andThen(intake.stopIntake());
  }
}