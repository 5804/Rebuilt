package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public TalonFXS intakeMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;

  public Intake() {
    intakeMotor = new TalonFXS(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    var intakeMotorConfig = new TalonFXSConfiguration();
    // intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    intakeMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true));
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  @Override
  public void periodic() {
    if (isRunning) intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
    else if (isReversing) intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
  }

  public Command runIntake() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseIntake() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopIntake() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; intakeMotor.set(0); }, this); }
}