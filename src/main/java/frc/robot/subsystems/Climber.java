package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class Climber extends SubsystemBase {

    private DoubleSupplier climberSup;

    public TalonFX leftClimberMotor;
    public TalonFX rightClimberMotor;
    public TalonFX linearActuator;

    public TalonFXConfiguration climberTalonFXConfigs = new TalonFXConfiguration();
    public Slot0Configs climberSlot0Configs = climberTalonFXConfigs.Slot0;
    public MotionMagicConfigs climberMotionMagicConfigs = climberTalonFXConfigs.MotionMagic;
    public MotorOutputConfigs climberMotorOutputFXConfigs = climberTalonFXConfigs.MotorOutput;

    public TalonFXConfiguration actuatorTalonFXConfigs = new TalonFXConfiguration();
    public Slot0Configs actuatorSlot0Configs = actuatorTalonFXConfigs.Slot0;
    public MotionMagicConfigs actuatorMotionMagicConfigs = actuatorTalonFXConfigs.MotionMagic;
    public MotorOutputConfigs actuatorMotorOutputFXConfigs = actuatorTalonFXConfigs.MotorOutput;

    double initialActuatorPos;

    public Climber(DoubleSupplier climberSup) {
        leftClimberMotor = new TalonFX(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID);
        rightClimberMotor = new TalonFX(Constants.ClimberConstants.RIGHT_CLIMBER_MOTOR_ID);
        rightClimberMotor.setControl(new Follower(Constants.ClimberConstants.LEFT_CLIMBER_MOTOR_ID, MotorAlignmentValue.Aligned));

        linearActuator = new TalonFX(Constants.ClimberConstants.LINEAR_ACTUATOR_MOTOR);
        leftClimberMotor.setPosition(0);

        climberSlot0Configs.kS = 0.25;
        climberSlot0Configs.kV = 0.12;
        climberSlot0Configs.kA = 0.01;
        climberSlot0Configs.kP = 15;
        climberSlot0Configs.kI = 0;
        climberSlot0Configs.kD = 0.1;

        climberMotionMagicConfigs.MotionMagicCruiseVelocity = 100;
        climberMotionMagicConfigs.MotionMagicAcceleration = 200;
        climberMotionMagicConfigs.MotionMagicJerk = 1600;

        climberMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;
        actuatorMotorOutputFXConfigs.NeutralMode = NeutralModeValue.Brake;

        leftClimberMotor.getConfigurator().apply(climberTalonFXConfigs);

        initialActuatorPos = linearActuator.getPosition().getValueAsDouble();

        this.climberSup = climberSup;
    }

    public Command setClimberPosition(double position, double tolerance) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        return run(() -> {
            leftClimberMotor.setControl(request.withPosition(position));
        })
                .until(() -> {
                    return Math.abs(getClimberPosition() - position) < tolerance;
                });
    }

    public Command extendActuator() {
        return run(() -> { linearActuator.set(-.1); })
            .until(() -> {
                return Math.abs(linearActuator.getPosition().getValueAsDouble()) >=
                (Math.abs(initialActuatorPos - Constants.ClimberConstants.ACTUATOR_DELTA_POS
                ) - Constants.ClimberConstants.ACTUATOR_POS_TOLERANCE);
            })
            .finallyDo(() -> { linearActuator.set(0); });
    }

    public Command retractActuator() {
        return run(() -> { linearActuator.set(.1); })
            .until(() -> {
                return Math.abs(linearActuator.getPosition().getValueAsDouble()) <=
                (Math.abs(initialActuatorPos) - Constants.ClimberConstants.ACTUATOR_POS_TOLERANCE);
            })
            .finallyDo(() -> { linearActuator.set(0); });
    }

    public Command setActuatorPosition(double pos) {
        MotionMagicVoltage request = new MotionMagicVoltage(0);
        return run(() -> { leftClimberMotor.setControl(request.withPosition(pos)); });
    }

    public Command setClimberSpeed() { return run(() -> { leftClimberMotor.set(climberSup.getAsDouble()); }); }

    public double getClimberPosition() { return leftClimberMotor.getPosition().getValueAsDouble(); }
}