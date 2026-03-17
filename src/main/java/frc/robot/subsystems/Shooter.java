package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public TalonFX rightShooterMotor;
  public TalonFX leftShooterMotor;
  public TalonFX shooterMotor;
  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    
    rightShooterMotor.setControl(new Follower(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

    var rightShooterMotorConfig = new TalonFXConfiguration();
    var leftShooterMotorConfig = new TalonFXConfiguration();

    var leftShooterMotorSlot0Configs = leftShooterMotorConfig.Slot0;

    leftShooterMotorSlot0Configs.kS = 0;
    leftShooterMotorSlot0Configs.kV = .11;
    leftShooterMotorSlot0Configs.kP = .3;
    leftShooterMotorSlot0Configs.kI = 0;
    leftShooterMotorSlot0Configs.kD = 0;
    
    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
  }

  public Command runShooter() {
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    return run(() -> { leftShooterMotor.setControl(m_request.withVelocity(Constants.ShooterConstants.SHOOTER_SPEED).withFeedForward(.5)); });
  }

  public Command stopShooter() { return run(() -> { leftShooterMotor.set(0); }); }

  public Command increaseShooterSpeed(){ return run(() -> { Constants.ShooterConstants.SHOOTER_SPEED +=0.1; }); }
  
  public Command decreaseShooterSpeed(){ return run(() -> { Constants.ShooterConstants.SHOOTER_SPEED -=0.1; }); }
}