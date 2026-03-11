package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public TalonFXS intakeMotor;

  public Intake() { intakeMotor = new TalonFXS(Constants.IntakeConstants.INTAKE_MOTOR_ID); }

  public Command runIntake() { return run(() -> { intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED); }); }

  public Command reverseIntake() { return run(() -> { intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED * 1.5); }); }

  public Command stopIntake() { return run(() -> { intakeMotor.set(0); }); }
}
