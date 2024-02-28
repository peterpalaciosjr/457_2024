// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MOTOR_IDS.DRIVE;
import com.kauailabs.navx.frc.AHRS;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase {
  DifferentialDrive drive;
  SparkPIDController leftPID, rightPID;
  RelativeEncoder leftEncoder, rightEncoder;

  AHRS navx;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    CANSparkMax left_main, left_follower;
    CANSparkMax right_main, right_follower;
    left_main = new CANSparkMax(DRIVE.LEFT_CAN_ID, MotorType.kBrushless);
    left_follower = new CANSparkMax(DRIVE.LEFT_FOLLOWER_CAN_ID, MotorType.kBrushless);
    right_main = new CANSparkMax(DRIVE.RIGHT_CAN_ID, MotorType.kBrushless);
    right_follower = new CANSparkMax(DRIVE.RIGHT_FOLLOWER_CAN_ID, MotorType.kBrushless);

  

    // Set the inversion states of the drive train.
    left_main.setInverted(false);
    left_follower.setInverted(false);
    right_main.setInverted(true);
    right_follower.setInverted(true);

    int driveCurrentLimit = 40;

    left_main.setSmartCurrentLimit(driveCurrentLimit);
    left_follower.setSmartCurrentLimit(driveCurrentLimit);
    right_main.setSmartCurrentLimit(driveCurrentLimit);
    right_follower.setSmartCurrentLimit(driveCurrentLimit);

  
    leftEncoder = left_main.getEncoder();
    rightEncoder = right_main.getEncoder();

    leftEncoder.setPositionConversionFactor(6 * Units.inchesToMeters(6));
    rightEncoder.setPositionConversionFactor(6 * Units.inchesToMeters(6));
    leftPID = left_main.getPIDController();
    rightPID = right_main.getPIDController();
    leftPID.setFeedbackDevice(leftEncoder);
    rightPID.setFeedbackDevice(rightEncoder);

    leftPID.setP(0);
    leftPID.setI(0);
    leftPID.setD(0);

    rightPID.setP(0);
    rightPID.setI(0);
    rightPID.setD(0);

    navx = new AHRS(Port.kMXP);

    // Set follower motors.
    left_follower.follow(left_main);
    right_follower.follow(right_main);

    drive = new DifferentialDrive(left_main, right_main);
  }

  /**
   * Drive command to drive the robot in teleop.
   *
   * @return a drive command
   */
  public Command driveCommand(Supplier<Double> leftStick, Supplier<Double> rightStick) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          drive.tankDrive(leftStick.get()*.8, rightStick.get()*.75);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
