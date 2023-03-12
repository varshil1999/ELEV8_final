package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElementSelector extends SubsystemBase {
  public boolean Mode;

  public ElementSelector() {}

  @Override
  public void periodic() {}

  public void mode(boolean InputType) {
    /*if(InputType == false) { Mode = false; SmartDashboard.putString("Input type", "Cone"); } 
    else*/ if(InputType == true) { Mode = true; SmartDashboard.putString("Input type", "Cube"); } 
  }

  public boolean Type() {
    return Mode;
  }
}
