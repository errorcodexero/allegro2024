package frc.robot.subsystems.target_tracker;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem; 
import org.xero1425.base.subsystems.swerve.SDSSwerveDriveSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024; 
import org.xero1425.base.subsystems.RobotSubsystem;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;




public class TargetTrackerSubsystem extends Subsystem {

    public TargetTrackerSubsystem(Subsystem parent) {
        super(parent, "targettracker") ;
        Pose2d target_pos_ = new Pose2d() ;
        Pose2d robot_pos_ = new Pose2d() ;
        AllegroRobot2024 robot_= (AllegroRobot2024)getRobot().getRobotSubsystem(); 

        robot_pos_ = robot_.getSwerve().getPose(); 
    }
}
  
