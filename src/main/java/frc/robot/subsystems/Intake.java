package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public TalonFXS intakeMotor;

  public Intake() {
    intakeMotor = new TalonFXS(Constants.IntakeConstants.INTAKE_MOTOR_ID);

  }
  public Command runIntake() {
    return run(() -> {
        intakeMotor.set(Constants.IntakeConstants.INTAKE_SPEED);
    });
  }
  public Command reverseIntake() {
    return run(() -> {
        intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPEED * 1.5);
    });
  }
public Command stopIntake() {
    return run(() -> {
        intakeMotor.set(0);
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
