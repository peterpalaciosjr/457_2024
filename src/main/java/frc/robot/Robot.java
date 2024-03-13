package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
   private Command m_autonomousCommand;
   double initTime;
   private RobotContainer m_robotContainer;

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
      if (Timer.getFPGATimestamp() - this.initTime < 0.5) {
         DriveSubsystem.drive.tankDrive(-0.3, -0.3);
      } else if (Timer.getFPGATimestamp() - this.initTime < 1.2) {
         DriveSubsystem.drive.tankDrive(-0.3, -0.3);
      } else if (Timer.getFPGATimestamp() - this.initTime < 2.5) {
         DriveSubsystem.drive.tankDrive(-0.3, -0.3);
      } else if (Timer.getFPGATimestamp() - this.initTime < 5.5) {
         DriveSubsystem.drive.tankDrive(-0.3, -0.3);
      } else {
         DriveSubsystem.drive.tankDrive(0.0, 0.0);
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
