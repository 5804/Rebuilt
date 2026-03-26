package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  public TalonFX indexerMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;

  public Indexer() { indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID); }

  @Override
  public void periodic() {
    if (isRunning) indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
    else if (isReversing) indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED);
  }

  public Command runIndexer() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseIndexer() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopIndexer() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; indexerMotor.set(0); }, this); }
}