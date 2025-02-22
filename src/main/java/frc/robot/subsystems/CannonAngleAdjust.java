package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CannonAngleAdjust extends SubsystemBase {
  private final int CURRENT_LIMIT = 45;
  private final double RAMP_RATE = 0.05;

  private final SparkMax angleAdjustMotor = new SparkMax(Constants.CANNON_ANGLE_ADJUST_MOTOR, MotorType.kBrushed);
  private final SparkMaxConfig angleAdjustConfig = new SparkMaxConfig();

  public CannonAngleAdjust() {
    angleAdjustConfig.inverted(true);
    angleAdjustConfig.idleMode(IdleMode.kBrake);
    angleAdjustConfig.smartCurrentLimit(CURRENT_LIMIT);
    angleAdjustConfig.openLoopRampRate(RAMP_RATE);
    angleAdjustMotor.configure(angleAdjustConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void periodic() {
  }

  public void setPercentOutput(double output) {
    if (output > 1.0) {
      output = 1.0;
    } else if (output < -1.0) {
      output = -1.0;
    }
    
    angleAdjustMotor.set(output);
  }
}
