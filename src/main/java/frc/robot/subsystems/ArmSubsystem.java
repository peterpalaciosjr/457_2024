package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MOTOR_IDS.ARM;
import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase
{

  CANSparkMax        arm;
  SparkPIDController armPID;
  RelativeEncoder    armEncoder;
  DutyCycleEncoder   armThroughbore;

  public ArmSubsystem()
  {
    arm = new CANSparkMax(ARM.ARM_CAN_ID, MotorType.kBrushless);
    armThroughbore = new DutyCycleEncoder(2);
    armPID = arm.getPIDController();
    armEncoder = arm.getEncoder();
    armPID.setFeedbackDevice(armEncoder);

    armPID.setP(0.015);
    armPID.setI(0);
    armPID.setD(0.001);
    armPID.setFF(0);

    // 1 Throughbore rotation = 360deg
    armEncoder.setPositionConversionFactor(360 / 21.5);
    arm.setInverted(false);
    arm.burnFlash();

    Timer.delay(1);
    SmartDashboard.putNumber("Arm Position Set", 0);
  }

  public void setArm(double degrees)
  {
    degrees = degrees < 240.0 ? degrees + 180 : degrees;
    armPID.setReference(MathUtil.clamp(degrees, 240, 340), ControlType.kPosition);
  }

  public Command setArmLocation(int degrees)
  {
    return run(() -> armPID.setReference(degrees, ControlType.kPosition));
  }

  public Command armLocation(Supplier<Boolean> down)
  {
    return run(() ->
                   armPID.setReference(down.get() ? 20 : 230, ControlType.kPosition));
  }

  /**
   * Get the throughbore angle in degrees.
   *
   * @return Angle from throughbore in degrees.
   */
  public double getThroughboreAngle()
  {
    return (armThroughbore.getAbsolutePosition() * 360.0);
  }

  boolean encoderSynced = false;

  /**
   * Periodic function
   */
  public void periodic()
  {
    // If the throughbore is 3 degrees more than the arm encoder, reseed the arm encoder.
    if (!encoderSynced)
    {
      if (((getThroughboreAngle()) - armEncoder.getPosition()) > 3)
      {
        armEncoder.setPosition(getThroughboreAngle());
        encoderSynced = true;
      }
    }

    SmartDashboard.putNumber("Arm Position Abs", getThroughboreAngle());
//    setArm(SmartDashboard.getNumber("Arm Position Set", 0));
    SmartDashboard.putNumber("Arm Position Reading", armEncoder.getPosition());
  }
}

