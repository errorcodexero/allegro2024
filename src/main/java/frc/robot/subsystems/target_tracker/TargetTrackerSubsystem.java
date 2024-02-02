package frc.robot.subsystems.target_tracker;
import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetTrackerSubsystem extends Subsystem {

    Pose3d target_pos_ ;

    public TargetTrackerSubsystem(Subsystem parent) {
        super(parent, "targettracker") ;
    
    }

     @Override
     public void init(LoopType prev, LoopType current) {
        super.init(prev, current);
    
        AprilTagFieldLayout field_layout = getRobot().getAprilTags(); 

        if (DriverStation.getAlliance().get() == Alliance.Red) { 
            target_pos_ = field_layout.getTagPose(4).orElse(null); 
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            target_pos_ = field_layout.getTagPose(7).orElse(null); 
        }

    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        AllegroRobot2024 robot_= (AllegroRobot2024)getRobot().getRobotSubsystem();
        Pose2d robot_pos_ = robot_.getSwerve().getPose();
    
    }

}

