package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Indexer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public TalonFX indexerMotor;
  public Indexer() {
       indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID);
  }

  public Command runIndexer() {
    return run(() -> {
        indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
      // Code to run the indexer motor at a certain speed
    });
  }

  public Command stopIndexer() {
    return run(() -> {
        indexerMotor.set(0);
      // Code to stop the indexer motor
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}