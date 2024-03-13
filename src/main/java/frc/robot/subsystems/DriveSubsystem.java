// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MOTOR_IDS.DRIVE;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase
{

  public static DifferentialDrive  drive;
  SparkPIDController leftPID, rightPID;
  RelativeEncoder leftEncoder, rightEncoder;
  DifferentialDrivePoseEstimator odometry;
  DifferentialDriveKinematics    kinematics;

  AHRS navx;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem()
  {
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

    // circumference / gear ratio
    double positionConversionFactor = (Math.PI * Units.inchesToMeters(6)) / 6;
    double trackWidth               = Units.inchesToMeters(72);
    Pose2d startingPose = new Pose2d(new Translation2d(Units.feetToMeters(0), Units.feetToMeters(0)),
                                     Rotation2d.fromDegrees(0));

    leftEncoder.setPositionConversionFactor(positionConversionFactor);
    rightEncoder.setPositionConversionFactor(positionConversionFactor);

    leftEncoder.setVelocityConversionFactor(positionConversionFactor / 60);
    rightEncoder.setVelocityConversionFactor(positionConversionFactor / 60);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftPID = left_main.getPIDController();
    rightPID = right_main.getPIDController();
    leftPID.setFeedbackDevice(leftEncoder);
    rightPID.setFeedbackDevice(rightEncoder);

    leftPID.setP(0.01);
    leftPID.setI(0);
    leftPID.setD(0);

    rightPID.setP(0.01);
    rightPID.setI(0);
    rightPID.setD(0);

    navx = new AHRS(Port.kMXP);

    // Set follower motors.
    left_follower.follow(left_main);
    right_follower.follow(right_main);

    drive = new DifferentialDrive(left_main, right_main);
    kinematics = new DifferentialDriveKinematics(trackWidth);
    odometry = new DifferentialDrivePoseEstimator(kinematics,
                                                  navx.getRotation2d(),
                                                  0,
                                                  0,
                                                  startingPose);

  }

  /**
   * Drive command to drive the robot in teleop.
   *
   * @return a drive command
   */
  public Command driveCommand(Supplier<Double> leftStick, Supplier<Double> rightStick)
  {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          drive.tankDrive(leftStick.get()*.7, rightStick.get()*.7);
        });
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public ChassisSpeeds getVelocity()
  {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void setChassisSpeeds(ChassisSpeeds velocity)
  {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(velocity);
    leftPID.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity);
    rightPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
  }


  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    odometry.update(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
  }

  public void resetPose(Pose2d pose)
  {
    odometry.resetPosition(navx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  public void setupPathPlanner()
  {
    // Configure AutoBuilder last
    AutoBuilder.configureRamsete(
        odometry::getEstimatedPosition, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getVelocity, // Current ChassisSpeeds supplier
        this::setChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent())
          {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
                                );
  }
}
