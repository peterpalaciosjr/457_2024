package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MOTOR_IDS.INTAKE;

public class IntakeSubsystem extends SubsystemBase
{

  CANSparkMax  intake;
  DigitalInput noteInput;


  public IntakeSubsystem()
  {
    intake = new CANSparkMax(INTAKE.INTAKE_CAN_ID, MotorType.kBrushless);
    noteInput = new DigitalInput(5);

    intake.setInverted(true);
  }

  public void setIntake(double percentage)
  {
  }

  /**
   * Set the intake power.
   *
   * @param percentage Power as a dutycycle percentage.
   * @return Instant command.
   */
  public Command setIntakeCmd(double percentage)
  {
    return run(() -> intake.set(percentage));
  }


  /**
   * Intake a note until the sensor detects it.
   *
   * @param percentage Power to intake note.
   * @return Command that runs until a note is intaken.
   */
  public Command intakeCommand(double percentage)
  {
    return run(() -> intake.set(percentage)).onlyIf(() -> !this.noteIntaked()).until(this::noteIntaked).andThen(runOnce(
        () -> intake.set(0)));
  }

  /**
   * Feed the note to the shooter until the note is not detected.
   *
   * @param percentage Power as dutycycle percentage.
   * @return Command that runs until the note is not detected.
   */
  public Command feedCommand(double percentage)
  {
    return run(() -> intake.set(percentage)).onlyIf(this::noteIntaked).until(() -> !this.noteIntaked()).andThen(runOnce(
        () -> intake.set(0)));
  }

  /**
   * Whether or not the note is detected.
   *
   * @return True if note is detected.
   */
  public boolean noteIntaked()
  {
    return !noteInput.get();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Intake power", intake.get());
    SmartDashboard.putBoolean("Note detected", noteIntaked());
  }

}

