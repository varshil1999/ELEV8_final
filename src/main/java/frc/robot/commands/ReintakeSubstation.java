// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class ReintakeSubstation extends CommandBase {
  private Arm arm;
  private Intake intake;
  private Gripper grip;
  int GroundCount = 0;
  boolean flag,returnflag,GroundOnce = false;
  /** Creates a new ReintakeSubstation. */
  public ReintakeSubstation(Arm arm, Intake intake,Gripper grip) {
    this.arm = arm;
    this.intake = intake;
    this.grip = grip;

    addRequirements(arm,intake,grip);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    returnflag = false;
    GroundOnce = true;
    GroundCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            if(GroundOnce == true && GroundCount<1){
            this.grip.IsGrip(false);
            this.grip.Grip();
            Timer.delay(0.25);
            this.arm.GripperDegrees(-6);
            this.arm.GripperRotate();
            GroundCount = 5;
            GroundOnce = false;
            }
            else if(GroundCount==5&&GroundOnce == false){
            this.arm.SetOperatorArmCancoderValues(208.125);
            this.arm.SetOperatorELbowCancoderValues(-17.84);
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=6;
            }
            else if(GroundCount==6&& flag == true){
            this.arm.SetOperatorArmCancoderValues(208.125);
            this.arm.SetOperatorELbowCancoderValues(-55.84);
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=7;
            }
            else if(GroundCount==7&& flag==true){
            this.arm.SetOperatorArmCancoderValues(169);
            this.arm.SetOperatorELbowCancoderValues(-112.67);
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=8;
            }
            else if(GroundCount==8 && flag==true){
            this.intake.OperatorCubeDegrees(15);
            this.intake.DriverCubeDegrees();
            returnflag=true;
            }
            if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
    flag = true;
    SmartDashboard.putString("Check Loop","13");
    
    } else {
  flag = false;
  SmartDashboard.putString("Check Loop","14");
       
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnflag;
  }
}
