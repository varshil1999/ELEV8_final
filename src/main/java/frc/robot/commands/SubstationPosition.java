package frc.robot.commands;



import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ElementSelector;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SubstationPosition extends CommandBase {
  private Arm arm;
  private ElementSelector mode;
  /** Creates a new IntakePosition. */
  public SubstationPosition(Arm arm, ElementSelector mode) {
    this.arm = arm;
    this.mode = mode;
    addRequirements(arm,mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  if(this.mode.Mode == false){
    this.arm.InputSUBORGRO(false);
    // this.arm.SetOperatorArmCancoderValues(164.80);
    // this.arm.SetOperatorELbowCancoderValues(-37);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
