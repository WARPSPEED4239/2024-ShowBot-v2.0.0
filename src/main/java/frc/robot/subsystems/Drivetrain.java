package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final int CURRENT_LIMIT = 45;
	private final double RAMP_RATE = 0.05;
	
	private SparkMax leftMotor1 = new SparkMax(Constants.DRIVETRAIN_MOTOR_LEFT_1, MotorType.kBrushless);
	private SparkMax leftMotor2 = new SparkMax(Constants.DRIVETRAIN_MOTOR_LEFT_2, MotorType.kBrushless);
	private SparkMax rightMotor1 = new SparkMax(Constants.DRIVETRAIN_MOTOR_RIGHT_1, MotorType.kBrushless);
	private SparkMax rightMotor2 = new SparkMax(Constants.DRIVETRAIN_MOTOR_RIGHT_2, MotorType.kBrushless);
  
  private SparkMaxConfig leftMotor1Config = new SparkMaxConfig();
  private SparkMaxConfig leftMotor2Config = new SparkMaxConfig();
  private SparkMaxConfig rightMotor1Config = new SparkMaxConfig();
  private SparkMaxConfig rightMotor2Config = new SparkMaxConfig();
  private DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);
	
	public Drivetrain() {
    leftMotor1Config.inverted(false);
    leftMotor2Config.inverted(false);
    rightMotor1Config.inverted(true);
    rightMotor2Config.inverted(true);

    leftMotor2Config.follow(leftMotor1);
    rightMotor2Config.follow(rightMotor1);

    leftMotor1Config.idleMode(IdleMode.kBrake);
    leftMotor2Config.idleMode(IdleMode.kBrake);
    rightMotor1Config.idleMode(IdleMode.kBrake);
    rightMotor2Config.idleMode(IdleMode.kBrake);

    leftMotor1Config.smartCurrentLimit(CURRENT_LIMIT);
    leftMotor2Config.smartCurrentLimit(CURRENT_LIMIT);
    rightMotor1Config.smartCurrentLimit(CURRENT_LIMIT);
    rightMotor2Config.smartCurrentLimit(CURRENT_LIMIT);

    leftMotor1Config.openLoopRampRate(RAMP_RATE);
    leftMotor2Config.openLoopRampRate(RAMP_RATE);
    rightMotor1Config.openLoopRampRate(RAMP_RATE);
    rightMotor2Config.openLoopRampRate(RAMP_RATE);

    leftMotor1.configure(leftMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor2.configure(leftMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor1.configure(rightMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor2.configure(rightMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}
	
	public void arcadeDrive(double move, double rotate) {
		final double MIN_MOVE_THRESHOLD = 0.05;
		final double MIN_ROTATE_THRESHOLD = 0.20;
		if (Math.abs(move) < MIN_MOVE_THRESHOLD) {
			move = 0.0;
    }
		if (Math.abs(rotate) < MIN_ROTATE_THRESHOLD) {
			rotate = 0.0;
    }
		
    drive.arcadeDrive(move, rotate);
	}
}
