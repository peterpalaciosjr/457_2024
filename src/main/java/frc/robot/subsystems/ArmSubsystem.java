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
import frc.robot.Constants.MISC;
import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase
{
  public CANSparkMax arm;
  CANSparkMax armSecond;
  SparkPIDController armPID;
  // SparkPIDController armPIDTwo;
  RelativeEncoder    armEncoder;
  // RelativeEncoder armEncoderTwo;
  DutyCycleEncoder   armThroughbore;

  public ArmSubsystem()
  {
    arm = new CANSparkMax(ARM.ARM_CAN_ID, MotorType.kBrushless);
    armSecond = new CANSparkMax(ARM.ARM_CAN_ID_TWO, MotorType.kBrushless);

    arm.setInverted(false);

    // Arm smart current limit and arm speed
    int armSmartCurrentLimit = 60;
    double rampRate = 0.255;

    arm.setSmartCurrentLimit(armSmartCurrentLimit);
    arm.setClosedLoopRampRate(rampRate);

    armSecond.follow(arm, true);

    armThroughbore = new DutyCycleEncoder(2);

    armPID = arm.getPIDController();
    //armPIDTwo = armSecond.getPIDController();

    armEncoder = arm.getEncoder();
    //armEncoderTwo = armSecond.getEncoder();

    armPID.setFeedbackDevice(armEncoder);
   // armPIDTwo.setFeedbackDevice(armEncoderTwo);

    armPID.setP(0.015*5);
    armPID.setI(0);
    armPID.setD(0.001*5);
    armPID.setFF(0);

   // armPIDTwo.setP(0.015);
   // armPIDTwo.setI(0);
   // armPIDTwo.setD(0.001);
   // armPIDTwo.setFF(0);

    // 1 Throughbore rotation = 360deg
    // Conversion factor set in Arm class of Constants.java 
    armEncoder.setPositionConversionFactor(MISC.ARM_CONVERSION_FACTOR);
   // armEncoderTwo.setPositionConversionFactor(MISC.ARM_CONVERSION_FACTOR);

    arm.setInverted(false);
    armSecond.setInverted(true);

    arm.burnFlash();
    armSecond.burnFlash();

    Timer.delay(1);
    SmartDashboard.putNumber("Arm Position Set", 0);
  }

  public void setArm(double degrees)
  {
    degrees = degrees < 240.0 ? degrees + 180 : degrees;
    armPID.setReference(MathUtil.clamp(degrees, 90, 0), ControlType.kPosition);
    // armPIDTwo.setReference(MathUtil.clamp(degrees, 240, 340), ControlType.kPosition);

  }

  public Command setArmLocation(double degrees)
  {
    return run(() -> armPID.setReference(degrees, ControlType.kPosition));
  }

  public Command setManualArmLocation(String movement, double speed)
  {

    arm.set(0.3);
    if (movement == "up" && armEncoder.getPosition() > 0)
    {
      return run(() -> armPID.setReference(armEncoder.getPosition() - 8, ControlType.kPosition));
    }
    else if (movement == "down" && armEncoder.getPosition() < 90)
    {
      return run(() -> armPID.setReference(armEncoder.getPosition() + 8, ControlType.kPosition));
    }
    else if (movement == "down" && armEncoder.getPosition() >= 90)
    {
      return run(() -> armPID.setReference(90, ControlType.kPosition));
    }
    else if (movement == "up" && armEncoder.getPosition() <= 0)
    {
      return run(() -> armPID.setReference(0, ControlType.kPosition));
    }
    else 
    {
      return run(() -> armPID.setReference(armEncoder.getPosition(), ControlType.kPosition));
    }


    // if (armEncoder.getPosition() < 90 && armEncoder.getPosition() > 0)
    // {
    //   return run(() -> armPID.setReference(armEncoder.getPosition() + rightY, ControlType.kPosition));
    // }



    // if (rightY < (Supplier <Double> 0))
    // {
    //   return run(() -> armPID.setReference(armEncoder.getPosition() - 1, ControlType.kPosition));
    // }
    // else if (movement == "down" && armEncoder.getPosition() < 90)
    // {
    //   return run(() -> armPID.setReference(armEncoder.getPosition() + 1, ControlType.kPosition));
    // }
    // else
    // {
    //   return run(() -> armPID.setReference(armEncoder.getPosition(), ControlType.kPosition));
    // }

  }

  public Command setArmWithSpeedControl(double degrees, double speed1, double speed2)
  {
    arm.set(speed1);
    armPID.setReference(42, ControlType.kPosition);
    Timer.delay(8);
    arm.set(speed2);
    return run(() -> armPID.setReference(degrees, ControlType.kPosition));
  }


  // public Command setArmSecondLocation(int degrees)
  // {
  //   // return run(() -> armPIDTwo.setReference(degrees, ControlType.kPosition));
  // }


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
      if ((getThroughboreAngle() - armEncoder.getPosition()) > 3)
      {
        armEncoder.setPosition(getThroughboreAngle());
        encoderSynced = true;
      }
    }

    // double currentDegrees = armEncoder.getPosition();

    SmartDashboard.putNumber("Arm Position Abs", getThroughboreAngle());
//    setArm(SmartDashboard.getNumber("Arm Position Set", 0));
    SmartDashboard.putNumber("Arm Position Reading", armEncoder.getPosition());
  }
}

