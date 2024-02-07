package frc.robot.subsystems.target_tracker;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetTrackerSubsystem extends Subsystem {

    private Pose2d target_pos_;
    private double distance_between_robot_and_target_;
    private double angle_to_target_;

    public TargetTrackerSubsystem(Subsystem parent) {
        super(parent, "targettracker");

    }

    @Override
    public void init(LoopType prev, LoopType current) {
        super.init(prev, current);

        AprilTagFieldLayout field_layout = getRobot().getAprilTags();

        Pose3d target_pos_3d = null;
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            target_pos_3d = field_layout.getTagPose(4).orElse(null);
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            target_pos_3d = field_layout.getTagPose(7).orElse(null);
        }
        if (target_pos_3d != null) {
            target_pos_ = target_pos_3d.toPose2d();
        }

    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        AllegroRobot2024 robotSubsystem = (AllegroRobot2024) getRobot().getRobotSubsystem();
        Pose2d robot_pos_ = robotSubsystem.getSwerve().getPose();
        distance_between_robot_and_target_ = calculateDistanceBetweenPoses(robot_pos_, target_pos_);
        angle_to_target_ = calculateAngleBetweenPoses(robot_pos_, target_pos_);
    
        putDashboard("tt_distance", DisplayType.Always, distance_between_robot_and_target_);
        putDashboard("tt_rotation", DisplayType.Always, angle_to_target_);   
    
    }

    private double calculateDistanceBetweenPoses(
            Pose2d robot,
            Pose2d target) {
        double dist = robot.getTranslation().getDistance(target.getTranslation());
        return dist;
    }

    public double getDistance() {
        return distance_between_robot_and_target_;
    }

    private double calculateAngleBetweenPoses(
            Pose2d robot,
            Pose2d target) {
        Transform2d diff = target.minus(robot);
        double rotation = diff.getRotation().getDegrees();
        return rotation;
    }

    public double getRotation() {
        return angle_to_target_;
    }

}
