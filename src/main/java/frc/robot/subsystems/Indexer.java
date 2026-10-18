package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  public TalonFX indexerMotor;
  public TalonFX agitatorMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public double defaultSupplyLimit = 30;
  public double previousIndexerSupplyLimit = -1;
  public double previousAgitatorSupplyLimit = -1;
  public TalonFXConfiguration indexerMotorConfig;
  public TalonFXConfiguration agitatorMotorConfig;


  public Indexer() {
    indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID);
    indexerMotorConfig = new TalonFXConfiguration();
    indexerMotor.getConfigurator().apply(indexerMotorConfig);
    SmartDashboard.putNumber("IndexerSupplyLimit", defaultSupplyLimit);

    agitatorMotor = new TalonFX(Constants.IndexerConstants.AGITATOR_MOTOR_ID);
    agitatorMotorConfig = new TalonFXConfiguration();
    agitatorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
    SmartDashboard.putNumber("AgitatorSupplyLimit", defaultSupplyLimit);
  }

  @Override
  public void periodic() {
    if (isRunning) {
      indexerMotor.set(IndexerConstants.INDEXER_SPEED);
      agitatorMotor.set(IndexerConstants.AGITATOR_SPEED);
    }
    else if (isReversing) {
      indexerMotor.set(-IndexerConstants.INDEXER_SPEED);
      agitatorMotor.set(-IndexerConstants.AGITATOR_SPEED);
    }

    double indexerSupplyLimit = SmartDashboard.getNumber("IndexerSupplyLimit", defaultSupplyLimit) > IndexerConstants.MIN_INDEXER_SUPPLY_LIMIT ? SmartDashboard.getNumber("IndexerSupplyLimit", defaultSupplyLimit) : IndexerConstants.MIN_INDEXER_SUPPLY_LIMIT; // The elevator motors will not stop if the current limit is set to 0
    double agitatorSupplyLimit = SmartDashboard.getNumber("AgitatorSupplyLimit", defaultSupplyLimit) > IndexerConstants.MIN_AGITATOR_SUPPLY_LIMIT ? SmartDashboard.getNumber("AgitatorSupplyLimit", defaultSupplyLimit) : IndexerConstants.MIN_AGITATOR_SUPPLY_LIMIT;
    
    if (previousIndexerSupplyLimit != indexerSupplyLimit) { // Jonathan thinks this is stupid and should be rewritten
      indexerMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(indexerSupplyLimit).withSupplyCurrentLimitEnable(true));
      indexerMotor.getConfigurator().apply(indexerMotorConfig);
      previousIndexerSupplyLimit = indexerSupplyLimit;
    }

    if (previousAgitatorSupplyLimit != agitatorSupplyLimit) {
      agitatorMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(agitatorSupplyLimit).withSupplyCurrentLimitEnable(true));
      agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
      previousAgitatorSupplyLimit = agitatorSupplyLimit;
    }
  }

  public Command runIndexer() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseIndexer() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopIndexer() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; indexerMotor.set(0); agitatorMotor.set(0); }, this); }
}