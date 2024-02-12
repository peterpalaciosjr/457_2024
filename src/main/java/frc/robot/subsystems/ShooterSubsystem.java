package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MOTOR_IDS.SHOOTER;

public class ShooterSubsystem extends SubsystemBase
{

  CANSparkMax        shooter;
  SparkPIDController shooterPID;
  RelativeEncoder    shooterEncoder;

  public ShooterSubsystem()
  {
    shooter = new CANSparkMax(SHOOTER.SHOOTER_CAN_ID, MotorType.kBrushless);
    shooterEncoder = shooter.getEncoder();
    shooterPID = shooter.getPIDController();

    shooterPID.setP(0.01);
    shooterPID.setI(0);
    shooterPID.setD(0);
    shooterPID.setFF(0);

    shooterPID.setFeedbackDevice(shooterEncoder);

    shooter.burnFlash();
    Timer.delay(1);
//    CANSparkMax shooterFollower = new CANSparkMax(SHOOTER.SHOOTER_FOLLOWER_CAN_ID, MotorType.kBrushless);
//
//    shooterFollower.follow(shooter);
  }

  /**
   * Run the shooter with duty cycle percentage of power.
   *
   * @param dutycycle Percentage of power.
   */
  public void setShooter(double dutycycle)
  {
    shooter.set(dutycycle);
  }

  /**
   * Run the shooter as a dutycycle percentage. [0, 1]
   *
   * @param dutycycle Percentage to run the shooter motor at.
   * @return Instant command to run the shooter.
   */
  public Command setShooterCmd(double dutycycle)
  {
    return runOnce(() -> shooter.set(dutycycle));
  }


  /**
   * Set the shooter PID.
   *
   * @param velocity Velocity in RPM.
   */
  public void runShooter(double velocity)
  {
    shooterPID.setReference(velocity, ControlType.kVelocity);
  }



  /**
   * Run the shooter command
   *
   * @param velocity Velocity in RPM.
   * @return Instant command to set the shooter PID.
   */
  public Command runShooterCmd(double velocity)
  {
    return runOnce(() -> shooterPID.setReference(velocity, ControlType.kVelocity));
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Shooter velocity RPM", shooterEncoder.getVelocity());
  }
}

