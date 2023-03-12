package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

    //Subsystems

    private final Swerve swerve = new Swerve();
    public static final Intake intake = new Intake();
    public static final Arm arm = new Arm();
    private final Gripper grip = new Gripper();
    private final ElementSelector elementSelected = new ElementSelector();

    //Controllers

    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    //Drive Controls and Buttons

    private final JoystickButton ZeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton RobotCentric = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton IntakePosition = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton RetractButtoon = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton OutputPosition = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton DropGamePieces = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton PrecisionButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton BoostButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    public static double speed = 0.7;

    //Operator Controls

    private final JoystickButton ConeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CubeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final POVButton GroundPosition = new POVButton(operator, 180);
    private final POVButton SubstationPosition = new POVButton(operator, 0);
    private final JoystickButton HybridPosition = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton HighPosition = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton MediumPosition = new JoystickButton(operator, XboxController.Button.kB.value);

    public RobotContainer() {
        
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(1)*speed, 
                () -> -driver.getRawAxis(0)*speed, 
                () -> -driver.getRawAxis(4)*speed,
                () -> RobotCentric.getAsBoolean()
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //driver
        ZeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        IntakePosition.onTrue(new frc.robot.commands.IntakePosition(intake, elementSelected,arm,grip));
        RetractButtoon.onTrue(new frc.robot.commands.ReintakeSubstation(arm, intake, grip));
        OutputPosition.onTrue(new frc.robot.commands.OutputPosition(intake, elementSelected,arm,grip));
        DropGamePieces.onTrue(new frc.robot.commands.DropGamePieces(intake, elementSelected,arm,grip));
        PrecisionButton.whileTrue(new PrecisionCommand(0.5));
        BoostButton.whileTrue(new PrecisionCommand(0.95));

        //operator
        ConeButton.toggleOnTrue(new InstantCommand(()->this.elementSelected.mode(false)));
        CubeButton.toggleOnTrue(new InstantCommand(()->this.elementSelected.mode(true)));
        GroundPosition.onTrue(new frc.robot.commands.GroundPosition(intake, elementSelected,arm));
        SubstationPosition.onTrue(new frc.robot.commands.SubstationPosition(arm, elementSelected));
        HybridPosition.onTrue(new frc.robot.commands.HybridPosition(intake, elementSelected));
        MediumPosition.onTrue(new frc.robot.commands.MediumPosition(intake, elementSelected,arm));
        HighPosition.onTrue(new frc.robot.commands.HighPosition(intake, elementSelected,arm));
        
    }
    
    public Command getAutonomousCommand() {
        return new AutoCone(swerve,intake,arm,grip);
        // return null;
    }

    //Commented code

    {

        // private final int translationAxis = XboxController.Axis.kLeftY.value;
// private final int strafeAxis = XboxController.Axis.kLeftX.value;
// private final int rotationAxis = XboxController.Axis.kRightX.value;


    //    // private final JoystickButton IntakeUp = new JoystickButton(driver, XboxController.Button.kX.value);
// private final JoystickButton IntakeDown = new JoystickButton(driver, XboxController.Button.kB.value);
// private final JoystickButton IntakeInside = new JoystickButton(driver, XboxController.Button.kY.value);
// private final JoystickButton IntakeOutside = new JoystickButton(driver, XboxController.Button.kA.value);
// private final JoystickButton SolGripperOutside = new JoystickButton(driver, XboxController.Button.kStart.value);
// private final JoystickButton SolGripperInside = new JoystickButton(driver, XboxController.Button.kBack.value);
// private final POVButton HOMEPosition = new POVButton(driver, 180);
// private final POVButton INTAKEPosition  = new POVButton(driver, 90);
// private final JoystickButton Solenoid_On = new JoystickButton(Joy2, XboxController.Button.kStart.value);
// private final JoystickButton Solenoid_OFF = new JoystickButton(Joy2, XboxController.Button.kBack.value);

// private final POVButton IntakeJoy2 = new POVButton(Joy2, 270);
// // private final POVButton MediumPosition = new POVButton(Joy2%, 90);
// private final POVButton ThrowPosition= new POVButton(Joy2, 0);
// private final POVButton ThrowPositionHybrid= new POVButton(driver, 0);
// private final POVButton throwPower  = new POVButton(Joy2, 180);

    // private final POVButton GripperUp = new POVButton(driver, 90);
// private final POVButton GripperDown = new POVButton(driver, 270);
// private final JoystickButton Docking = new JoystickButton(driver, XboxController.Button.kY.value);
// private final JoystickButton Resting = new JoystickButton(driver, XboxController.Button.kX.value);


// private final POVButton MediumPosition = new POVButton(driver, 0);


// GripperUp.onTrue(new InstantCommand(()-> arm.manualGripperUpOrDown(2)));
    // GripperDown.onTrue(new InstantCommand(()-> arm.manualGripperUpOrDown(-2)));
    // Docking.onTrue(new SequentialCommandGroup(new DockBalance3(s_Swerve),new DockBalanceRest(s_Swerve)));
    // Resting.onTrue(new DockBalanceRest(s_Swerve));

    // SubstationPo sition.onTrue(new frc.robot.commands.GroundPosition(intake, inputMode));
    /* Driver Buttons */
    
    // IntakeJoy2.whileTrue(new InstantCommand(()-> intake.LifterDegrees(117)));
    // ThrowPosition.whileTrue(new InstantCommand(()-> intake.LifterDegrees(75)));
    // throwPower.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-1)));
    // throwPower.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));
    // // IntakeDown.whileTrue(new InstantCommand(()-> intake.ThrowPosition(000)));
    // IntakeUp.whileTrue(new frc.robot.commands.ManualIntakeUp(this.intake));
    // IntakeDown.whileTrue(new frc.robot.commands.ManualIntakeDown(this.intake));
    // INTAKEPosition.whileTrue(new InstantCommand(() -> arm.IntakePosition()));
    // HOMEPosition.whileTrue(new InstantCommand(() -> arm.HomePostion()));
    // IntakeInside.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-0.38)));
    // IntakeOutside.whileTrue(new InstantCommand(() -> intake.OuttakeCube(-0.25)));
    // IntakeInside.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));
    // IntakeOutside.whileFalse(new InstantCommand(() -> intake.OuttakeCube(0)));

    // // SolGripperOutside.whileTrue(new InstantCommand(()-> intake.ThrowPosition(211000)));
    // SolGripperInside.toggleOnTrue(new frc.robot.commands.IntakeOnBeamBreaked(this.intake));
    // // SolGripperInside.whileFalse(new InstantCommand(()->intake.IntakeCube(0)));
    // // MediumPosition.whileTrue(new InstantCommand(()-> arm.MediumPosition()));
    // Solenoid_On.onTrue(new InstantCommand(()-> arm.Grip(false)));
    // Solenoid_OFF.onTrue(new InstantCommand(()-> arm.Grip(true)));
    // MoveArmUp.whileTrue(new InstantCommand(() -> arm.MoveArm(2)));
    // MoveArmDown.whileTrue(new InstantCommand(() -> arm.MoveArm(-2)));
    // MoveElbowUp.whileTrue(new InstantCommand(() -> arm.MoveElbow(2)));
    // MoveElbowDown.whileTrue(new InstantCommand(() -> arm.MoveElbow(-2)));
    // IntakeCOne.whileTrue((new InstantCommand(()-> arm.IntakeCOne())));
    // MediumPosition.whileTrue(new InstantCommand(()-> arm.MediumPosition()));
}
}


