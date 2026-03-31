package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateEnums.IntakeState;

public class Intake extends SubsystemBase {
  public TalonFXS intakeMotor;

  public Intake() {
    intakeMotor = new TalonFXS(IntakeConstants.INTAKE_MOTOR_ID);
    var intakeMotorConfig = new TalonFXSConfiguration();
    intakeMotorConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true));
    intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  public Command runIntake() { return Commands.runOnce(() -> { IntakeState.Reversing = false; IntakeState.Running = true; }, this); }
  public Command reverseIntake() { return Commands.runOnce(() -> { IntakeState.Running = false; IntakeState.Reversing = true; }, this); }
  public Command stopIntake() { return Commands.runOnce(() -> { IntakeState.Running = false; IntakeState.Reversing = false; intakeMotor.set(0); }, this); }
}