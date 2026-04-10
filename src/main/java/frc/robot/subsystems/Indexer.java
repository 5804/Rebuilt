package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  public TalonFX indexerMotor;
  public TalonFXS agitatorMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public double defaultSupplyLimit = 30;
  public double previousSupplyLimit = -1;
  public TalonFXConfiguration indexerMotorConfig;
  public TalonFXSConfiguration agitatorMotorConfig;



  public Indexer() {
    indexerMotor = new TalonFX(Constants.IndexerConstants.INDEXER_MOTOR_ID);
    indexerMotorConfig = new TalonFXConfiguration();
    indexerMotor.getConfigurator().apply(indexerMotorConfig);
    SmartDashboard.putNumber("IndexerSupplyLimit", defaultSupplyLimit);

    agitatorMotor = new TalonFXS(Constants.IndexerConstants.AGITATOR_MOTOR_ID);
    agitatorMotorConfig = new TalonFXSConfiguration();
    // agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
    SmartDashboard.putNumber("AgitatorSupplyLimit", defaultSupplyLimit);
  }

  @Override
  public void periodic() {
    if (isRunning) {
      indexerMotor.set(Constants.IndexerConstants.INDEXER_SPEED);
      agitatorMotor.set(-Constants.IndexerConstants.AGITATOR_SPEED);
    }
    else if (isReversing) {
      indexerMotor.set(-Constants.IndexerConstants.INDEXER_SPEED);
      agitatorMotor.set(Constants.IndexerConstants.AGITATOR_SPEED);
    }

    double indexerSupplyLimit = SmartDashboard.getNumber("IndexerSupplyLimit", defaultSupplyLimit) > 20 ? SmartDashboard.getNumber("IndexerSupplyLimit", defaultSupplyLimit) : 20; // The elevator motors will not stop if the current limit is set to 0
    if (previousSupplyLimit != indexerSupplyLimit) {
        indexerMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(indexerSupplyLimit).withSupplyCurrentLimitEnable(true));
        indexerMotor.getConfigurator().apply(indexerMotorConfig);
        previousSupplyLimit = indexerSupplyLimit;
    }

    double agitatorSupplyLimit = SmartDashboard.getNumber("AgitatorSupplyLimit", defaultSupplyLimit) > 20 ? SmartDashboard.getNumber("AgitatorSupplyLimit", defaultSupplyLimit) : 20; // The elevator motors will not stop if the current limit is set to 0
    if (previousSupplyLimit != agitatorSupplyLimit) {
        agitatorMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(agitatorSupplyLimit).withSupplyCurrentLimitEnable(true));
        agitatorMotor.getConfigurator().apply(agitatorMotorConfig);
        previousSupplyLimit = agitatorSupplyLimit;
    }
  }

  public Command runIndexer() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseIndexer() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopIndexer() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; indexerMotor.set(0); agitatorMotor.set(0); }, this); }

}