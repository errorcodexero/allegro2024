package frc.robot.subsystems.target_tracker;

import java.util.Optional;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.XeroMath;

import frc.robot.AprilTags;
import frc.robot.subsystems.toplevel.AllegroRobot2024;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetTrackerSubsystem extends Subsystem {

    private Pose2d target_pos_;
    private double distance_between_robot_and_target_;
    private double angle_to_target_;
    private boolean sees_target_ ;
    private int target_number_ ;
    private boolean feed_angle_to_db_ ;

    public TargetTrackerSubsystem(Subsystem parent) {
        super(parent, "targettracker");
        sees_target_ = false ;
        feed_angle_to_db_ = false ;
    }

    public void feedTargetToDB(boolean b) {
        feed_angle_to_db_ = b ;
    }

    @Override
    public void init(LoopType prev, LoopType current) {
        super.init(prev, current);

        AprilTagFieldLayout field_layout = getRobot().getAprilTags();

        Pose3d target_pos_3d = null;
        Optional<Alliance> value = DriverStation.getAlliance() ;
        if (value.isPresent()) {
            if (value.get() == Alliance.Red) {
                target_number_ = AprilTags.RED_SPEAKER_CENTER ;
                target_pos_3d = field_layout.getTagPose(target_number_).orElse(null);
            } else if (value.get() == Alliance.Blue) {
                target_number_ = AprilTags.BLUE_SPEAKER_CENTER ;
                target_pos_3d = field_layout.getTagPose(target_number_).orElse(null);
            }

            if (target_pos_3d != null) {
                target_pos_ = target_pos_3d.toPose2d();
            }
        }
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        if (target_pos_ != null) {
            AllegroRobot2024 robotSubsystem = (AllegroRobot2024) getRobot().getRobotSubsystem();
            Pose2d robot_pos_ = robotSubsystem.getSwerve().getPose();
            distance_between_robot_and_target_ = calculateDistanceBetweenPoses(robot_pos_, target_pos_);
            angle_to_target_ = calculateAngleBetweenPoses(robot_pos_, target_pos_);
        
            putDashboard("tt_distance", DisplayType.Always, distance_between_robot_and_target_);
            putDashboard("tt_rotation", DisplayType.Always, angle_to_target_);

            putDashboard("tt_target_x", DisplayType.Always, target_pos_.getX()) ;
            putDashboard("tt_target_y", DisplayType.Always, target_pos_.getY()) ;
            putDashboard("tt_target_ang", DisplayType.Always, target_pos_.getRotation().getDegrees()) ;

            putDashboard("tt_robotpos_x", DisplayType.Always, target_pos_.getX()) ;
            putDashboard("tt_robotpos_y", DisplayType.Always, target_pos_.getY()) ;
            putDashboard("tt_robotpos_ang", DisplayType.Always, target_pos_.getRotation().getDegrees()) ;            

            if (feed_angle_to_db_) {
                //
                // Tell the drive base the angle we want you to have.  This is only honored if it is enabled on the
                // drivebase.  The driver will enable or disable this feature via the gamepad.
                //
                robotSubsystem.getSwerve().setSWRotationAngle(XeroMath.normalizeAngleDegrees(angle_to_target_ + robot_pos_.getRotation().getDegrees()));
            }

            LimeLightSubsystem ll = robotSubsystem.getLimelight() ;
            sees_target_ = ll.validTargets() && ll.hasAprilTag(target_number_) ;

            putDashboard("ll:dist", DisplayType.Always, distance_between_robot_and_target_);
            putDashboard("ll:angle", DisplayType.Always, angle_to_target_);
            putDashboard("ll:tag", DisplayType.Always, sees_target_);
        }
    }

    public boolean seesTarget() {
        return sees_target_ ;
    }

    public double getDistance() {
        return distance_between_robot_and_target_;
    }    

    public double getRotation() {
        return angle_to_target_;
    }    

    private static double calculateDistanceBetweenPoses(Pose2d robot, Pose2d target) {        
        double dist = Double.POSITIVE_INFINITY ;

        if (target != null) {
            dist = robot.getTranslation().getDistance(target.getTranslation());
        }

        return dist;
    }

    private static double calculateAngleBetweenPoses(Pose2d robot, Pose2d target) {
        Translation2d diff = robot.getTranslation().minus(target.getTranslation()) ;
        Rotation2d rotation = new Rotation2d(diff.getX(), diff.getY()) ;
        return XeroMath.normalizeAngleDegrees(rotation.getDegrees() - robot.getRotation().getDegrees()) ;
    }
}
