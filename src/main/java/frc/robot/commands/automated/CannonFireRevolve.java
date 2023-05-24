package frc.robot.commands.automated;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.CannonRevolve;

public class CannonFireRevolve extends CommandBase {

  private final Cannon mCannon;
  private final CannonRevolve mCannonRevolve;
  private Timer mTimer;
  private Timer mCorrectionTimer;

  private boolean mCorrectionNeeded;
  private boolean mWaitingForLimit;
  private boolean mRotationStep;
  private boolean mEnd;

  private int mNumberOfBarrelsAdvanced;
  private double mPercentOutput;
  private double mMinFiringPressure;
  private double mStartTime;
  private double mCorrectionStartTime;

  // private final RGBController mRGBController;
  // private final Color[] colors = {Color.Red, Color.Black};

  public CannonFireRevolve(Cannon cannon, CannonRevolve cannonRevolve, double percentOutput/*, RGBController RGBController*/) {
    mCannon = cannon;
    mCannonRevolve = cannonRevolve;
    mPercentOutput = percentOutput;
    // mRGBController = RGBController;

    addRequirements(mCannon, mCannonRevolve);
  }

  @Override
  public void initialize() {
    mEnd = false;
    mRotationStep = true;
    mNumberOfBarrelsAdvanced = 0;
    mMinFiringPressure = mCannon.getMinFiringPressure();

    mTimer = new Timer();
    mCorrectionTimer = new Timer();
    if (!mCannonRevolve.getRevolveLimitSwitch()) {
      mCorrectionNeeded = true;
      mWaitingForLimit = true;
      mPercentOutput = -mPercentOutput;
    } else {
      mCorrectionNeeded = false;
      mWaitingForLimit = false;
      mTimer.start();
      mStartTime = mTimer.get();
    }
  }

  @Override
  public void execute() {
    if (mCorrectionNeeded) { // Correction to barrel is needed, do not fire
      mCannon.setLoadingSolenoidState(false);
      mCannon.setFiringSolenoidState(false);
      if (mRotationStep) {
        initialRotation();
      } else {
        correction();
      }
    } else if (mCannon.getFiringTankPressure() < mMinFiringPressure) { // Cannon not at min pressure, do not fire
      mCannon.setLoadingSolenoidState(false);
      mCannon.setFiringSolenoidState(false);
      mEnd = true;
    } else { // All conditions are met, fire
      double mElapsedTime = mTimer.get() - mStartTime;

      if (mElapsedTime < 1.0) {
        mCannon.setLoadingSolenoidState(false);
        mCannon.setFiringSolenoidState(false);
        // mRGBController.setColors(colors, 0.1);
      } else if (mElapsedTime >= 1.0 && mElapsedTime < 1.75) {
        mCannon.setLoadingSolenoidState(false);
        mCannon.setFiringSolenoidState(true);
        // mRGBController.setColor(Color.White);
      } else {
        mCannon.setLoadingSolenoidState(false);
        mCannon.setFiringSolenoidState(false);
        // mRGBController.setColor(Color.Black);
        if (mRotationStep) {
          initialRotation();
        } else {
          correction();
        }
      }
    }
  }

  public void initialRotation() {
    mCannonRevolve.setPercentOutput(mPercentOutput);

    if (mWaitingForLimit) {
      if (mCannonRevolve.getRevolveLimitSwitch()) {
        mNumberOfBarrelsAdvanced++;
        mWaitingForLimit = false;
      }
    } else {
      if (!mCannonRevolve.getRevolveLimitSwitch()) {
        mWaitingForLimit = true;
      }
    }

    if (mNumberOfBarrelsAdvanced == 1) {
      mCannonRevolve.setPercentOutput(0.0);
      System.out.println("RAN1");
      mCorrectionTimer.restart();
      System.out.println("RAN2");
      mCorrectionStartTime = mTimer.get();
      System.out.println("Start Time" + mCorrectionStartTime);
      mRotationStep = false;
    }
  }

  public void correction() {
    if ((mCorrectionTimer.get() - mCorrectionStartTime) > 0.5) {
      System.out.println("Time: " + mCorrectionTimer.get());
      if (mCannonRevolve.getRevolveLimitSwitch()) {
        mCannonRevolve.setPercentOutput(0.0);
        mEnd = true;
      } else {
        mCannonRevolve.setPercentOutput(-mPercentOutput);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    mCannonRevolve.setPercentOutput(0.0);
    mCannon.setLoadingSolenoidState(false);
    mCannon.setFiringSolenoidState(false);

    mTimer.stop();
    mCorrectionTimer.stop();
    mTimer.reset();
    mCorrectionTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return mEnd;
  }
}
