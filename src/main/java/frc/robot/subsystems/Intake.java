package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  public TalonFXS intakeMotor;
  public boolean isRunning = false;
  public boolean isReversing = false;
  public TalonFXSConfiguration intakeMotorConfig;
  public double defaultSupplyLimit = 25;
  public double previousSupplyLimit = -1;

  public Intake() {
    intakeMotor = new TalonFXS(Constants.IntakeConstants.INTAKE_MOTOR_ID);

    intakeMotorConfig = new TalonFXSConfiguration();
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
    SmartDashboard.putNumber("IntakeSupplyLimit", defaultSupplyLimit);
  }

  @Override
  public void periodic() {
    if (isRunning) intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
    else if (isReversing) intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED);
    double intakeSupplyLimit = SmartDashboard.getNumber("IntakeSupplyLimit", defaultSupplyLimit); // The Intake motors will not stop if the current limit is set to 0
    if (previousSupplyLimit != intakeSupplyLimit) {
        intakeMotorConfig.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(intakeSupplyLimit).withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLowerLimit(intakeSupplyLimit / 2)
        .withSupplyCurrentLowerTime(0.25));
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
        previousSupplyLimit = intakeSupplyLimit;
    }
  }

  public Command runIntake() { return Commands.runOnce(() -> { isRunning = true; isReversing = false; }, this); }
  public Command reverseIntake() { return Commands.runOnce(() -> { isReversing = true; isRunning = false; }, this); }
  public Command stopIntake() { return Commands.runOnce(() -> { isRunning = false; isReversing = false; intakeMotor.set(0); }, this); }
}