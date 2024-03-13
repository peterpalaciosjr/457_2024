package frc.robot;

import edu.wpi.first.wpilibj.DriverStation; // Added
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {
   private Command m_autonomousCommand;
   double initTime;
   private RobotContainer m_robotContainer;

   // Class instantiations for autonomous organizational purposes
   private final ShooterSubsystem a_shooterSubsystem = new ShooterSubsystem(); // Added
   private final IntakeSubsystem a_intakeSubsystem = new IntakeSubsystem(); // Added
   private final DriveSubsystem a_driveSubsystem = new DriveSubsystem(); // Added

   public Robot() {
   }

   public void robotInit() {
      this.m_robotContainer = new RobotContainer();
   }

   public void robotPeriodic() {
      CommandScheduler.getInstance().run();
   }

   public void disabledInit() {
   }

   public void disabledPeriodic() {
   }

   public void autonomousInit() {
      this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand();

      if (this.m_autonomousCommand != null) {
         this.m_autonomousCommand.schedule();
      }

   }

   public void autonomousPeriodic() {
      if (Timer.getFPGATimestamp() < 1) // Modified
      {
        a_shooterSubsystem.setShooter(0.42); // Added
      }
      else if (Timer.getFPGATimestamp() < 2.5)  // Modified
      {
         a_intakeSubsystem.setIntake(0.6); // Added
      } 
      else if (Timer.getFPGATimestamp() < 3) // Modified
      {
         a_shooterSubsystem.setShooter(0); // Added
         a_intakeSubsystem.setIntake(0); // Added
      } 
      else
      {
         DriverStation.startDataLog("Autonomous Complete") // Added
         return; // Added
      }

   }

   public void teleopInit() {
      if (this.m_autonomousCommand != null) {
         this.m_autonomousCommand.cancel();
      }

   }

   public void teleopPeriodic() {
   }

   public void testInit() {
      CommandScheduler.getInstance().cancelAll();
   }

   public void testPeriodic() {
   }

   public void simulationInit() {
   }

   public void simulationPeriodic() {
   }
}
