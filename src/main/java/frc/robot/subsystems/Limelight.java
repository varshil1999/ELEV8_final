package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  private final Swerve swerveSubsystem;
  //private final Pigeon2Subsystem pigeon2Subsystem;
  
  
  
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  static NetworkTableEntry valueOfPoses = table.getEntry("botpose_wpiblue");

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
  // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence on the final pose estimate.
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2));
  private static SwerveDrivePoseEstimator poseEstimator;
  
  //private final Field2d field2d = new Field2d();

  public Limelight(Swerve swerveSubsystem, Pose2d initPose) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(5);
    this.swerveSubsystem = swerveSubsystem;
    
    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics, 
      swerveSubsystem.getYaw(), 
      swerveSubsystem.getModulePositions(), 
      initPose, 
      stateStdDevs, 
      visionMeasurementStdDevs);

    //SmartDashboard.putData("Field", field2d);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update pose estimator with visible targets
    // latest pipeline result
    double[] temp = {0.0,0.0,0.0,0.0,0.0,0.0};//Defult getEntry
    double[] result = valueOfPoses.getDoubleArray(temp);
    double timestamp = Timer.getFPGATimestamp();
    if((result[0]<2.5 && Math.abs(result[3])>90) || (result[0]>10.0 && Math.abs(result[3])<90) /* || result[0]>5*/)
    {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5)));
    }
    else
    {
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10)));
    }
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0)==1.0 && NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0)>0) {
      SmartDashboard.putBoolean("Camera Has Target", true);
      Translation3d translation3d = new Translation3d(result[0]/*+Units.feetToMeters(27.0)*/, result[1]/*+Units.feetToMeters(13.5)*/, result[2]);
      SmartDashboard.putString("translation", translation3d.toString());
      Rotation3d rotation3d = new Rotation3d(Units.degreesToRadians(result[3]), Units.degreesToRadians(result[4]), Units.degreesToRadians(result[5]));
      SmartDashboard.putString("rotation", rotation3d.toString());
      Pose3d pose3d = new Pose3d(translation3d, rotation3d);
      poseEstimator.addVisionMeasurement(pose3d.toPose2d(), timestamp);
    }else{
      SmartDashboard.putBoolean("Camera Has Target", false);
    }

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), swerveSubsystem.getYaw(), swerveSubsystem.getModulePositions());
    //field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putString("Pose", poseEstimator.getEstimatedPosition().toString());
    // This method will be called once per scheduler run
  }

  public static Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }

  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }

  public double getPoseTheta() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public Rotation2d getPoseRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(swerveSubsystem.getYaw(), swerveSubsystem.getModulePositions(), pose);
  }

  /*public void setTrajectoryField2d(Trajectory trajectory) {
    field2d.getObject("traj").setTrajectory(trajectory);
  }*/
}