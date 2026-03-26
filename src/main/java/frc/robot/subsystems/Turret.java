package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TurretMath;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

public class Turret extends SubsystemBase {
  public TalonFX yawMotor = new TalonFX(Constants.TurretConstants.TURRET_MOTOR_ID);
  public Pose2d turretPose;
  public TurretMath turretMath;
  public boolean isRedAlliance;
  public CommandSwerveDrivetrain drivetrain;
  public boolean isAiming = false;

  public Turret(CommandSwerveDrivetrain d, TurretMath t, boolean a) {
    this.drivetrain = d;
    this.turretMath = t;
    this.isRedAlliance = a;
    turretMath.calculateTarget(isRedAlliance, drivetrain);

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 12;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.35;
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.117 / (2 * Math.PI);
    slot0Configs.kA = 0;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 100;
    motionMagicConfigs.MotionMagicAcceleration = 1000;
    motionMagicConfigs.MotionMagicJerk = 10000;

    yawMotor.getConfigurator().apply(talonFXConfigs);
    yawMotor.setPosition(0 / 360);
  }

  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double TURRET_GEAR_RATIO = 6.4;
  double[][] hubPos = {{ 4.625594, 4 }, { 11.665394, 4 }};

  private double normalizeAngle(double deg) { return MathUtil.inputModulus(deg, -180.0, 180.0); }

  public void setYaw(double angleDeg) { yawMotor.setControl(new MotionMagicExpoVoltage(TURRET_GEAR_RATIO * normalizeAngle(angleDeg) / 360)); }

  public Command setYawCommand(double angleDeg) {
    double normalized = normalizeAngle(angleDeg);
    double targetAngle = TURRET_GEAR_RATIO * normalized / 360;
    return run(() -> yawMotor.setControl(new MotionMagicExpoVoltage(targetAngle)));
  }

  public Command aimTurret() { return Commands.runOnce(() -> isAiming = true, this); }
  public Command stopAiming() { return Commands.runOnce(() -> isAiming = false, this); }

  @Override
  public void periodic() {
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d turretPose = robotPose.transformBy(new Transform2d(new Translation2d(-.25, 0), new Rotation2d()));

    int teamNum = 0;
    SmartDashboard.putNumber("Dist from Hub", Math.sqrt(Math.pow((turretPose.getX() - hubPos[teamNum][0]), 2) + Math.pow((turretPose.getY() - hubPos[teamNum][1]), 2)));

    turretMath.calculateTurretMath(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians(),
        drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);

    if (isAiming) {
      turretMath.calculateTarget(isRedAlliance, drivetrain);
      setYaw(-(robotPose.getRotation().getDegrees() + 90) + turretMath.turretAngle);
    }
  }
}