package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateEnums.ShooterState;

public class Shooter extends SubsystemBase {
  public TalonFX rightShooterMotor;
  public TalonFX leftShooterMotor;

  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);

    rightShooterMotor
        .setControl(new Follower(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID, MotorAlignmentValue.Opposed));

    var rightShooterMotorConfig = new TalonFXConfiguration();
    var leftShooterMotorConfig = new TalonFXConfiguration();

    var leftShooterMotorSlot0Configs = leftShooterMotorConfig.Slot0;
    var rightShooterMotorSlot0Configs = rightShooterMotorConfig.Slot0;

    leftShooterMotorSlot0Configs.kP = ShooterConstants.kP;
    leftShooterMotorSlot0Configs.kI = ShooterConstants.kI;
    leftShooterMotorSlot0Configs.kD = ShooterConstants.kD;
    leftShooterMotorSlot0Configs.kS = ShooterConstants.kS;
    leftShooterMotorSlot0Configs.kV = ShooterConstants.kV;
    leftShooterMotorSlot0Configs.kA = ShooterConstants.kA;

    rightShooterMotorSlot0Configs.kP = ShooterConstants.kP;
    rightShooterMotorSlot0Configs.kI = ShooterConstants.kI;
    rightShooterMotorSlot0Configs.kD = ShooterConstants.kD;
    rightShooterMotorSlot0Configs.kS = ShooterConstants.kS;
    rightShooterMotorSlot0Configs.kV = ShooterConstants.kV;
    rightShooterMotorSlot0Configs.kA = ShooterConstants.kA;

    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftShooterMotorConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true));
    rightShooterMotorConfig
        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true));
    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
  }

  public Command runShooter(boolean manual) { return runOnce(() -> { ShooterState.Manual = manual; ShooterState.ReachedSpeed = false; ShooterState.Reversing = false; ShooterState.Running = true; }); }
  public Command reverseShooter(boolean manual) { return runOnce(() -> { ShooterState.Manual = manual; ShooterState.ReachedSpeed = false; ShooterState.Running = false; ShooterState.Reversing = true; }); }
  public Command stopShooter() { return runOnce(() -> { ShooterState.Manual = false; ShooterState.ReachedSpeed = false; ShooterState.Running = false; ShooterState.Reversing = false; leftShooterMotor.set(0); }); }

  public Command increaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED += 0.1); }
  public Command decreaseShooterSpeed() { return run(() -> Constants.ShooterConstants.SHOOTER_SPEED -= 0.1); }
}