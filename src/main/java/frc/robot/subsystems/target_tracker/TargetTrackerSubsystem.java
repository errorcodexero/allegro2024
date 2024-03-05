package frc.robot.subsystems.target_tracker;

import java.util.Optional;

import org.xero1425.base.LoopType;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroMath;

import frc.robot.AprilTags;
import frc.robot.subsystems.toplevel.AllegroRobot2024;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetTrackerSubsystem extends Subsystem {

    private static final double kCameraOffset = 0.4;
    private Pose2d target_pos_;
    private double distance_between_robot_and_target_;
    private double angle_to_target_;
    private boolean sees_target_ ;
    private int target_number_ ;
    private LimeLightSubsystem ll_ ;

    private double camera_angle_ ;
    private double camera_height_ ;
    private double target_height_ ;

    public TargetTrackerSubsystem(Subsystem parent, LimeLightSubsystem ll) throws BadParameterTypeException, MissingParameterException {
        super(parent, "targettracker");
        sees_target_ = false ;
        ll_ = ll ;

        camera_angle_ = getSettingsValue("camera-angle").getDouble() ;
        camera_height_ = getSettingsValue("camera-height").getDouble() ;
        target_height_ = getSettingsValue("target-height").getDouble() ;        
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

            MessageLogger logger = getRobot().getMessageLogger() ;

            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            if (ll_.validTargets() && ll_.hasAprilTag(target_number_)) {
                logger.add("apriltag", true) ;
                sees_target_ = true ;
                angle_to_target_ = -ll_.getTX(target_number_);
                distance_between_robot_and_target_ = (target_height_ - camera_height_) / Math.tan(Math.toRadians(camera_angle_ + ll_.getTY(target_number_))) + kCameraOffset ;
                // distance_between_robot_and_target_ = calculateDistanceBetweenPoses(robot_pos_, target_pos_);                
            }
            else {
                logger.add("apriltag", false) ;
                sees_target_ = false ;
                distance_between_robot_and_target_ = calculateDistanceBetweenPoses(robot_pos_, target_pos_);
                angle_to_target_ = calculateAngleBetweenPoses(robot_pos_, target_pos_);
            }
            logger.add("distance", distance_between_robot_and_target_) ;
            logger.add("angle", angle_to_target_) ;
            logger.endMessage();

            putDashboard("tt_distance", DisplayType.Always, distance_between_robot_and_target_);
            putDashboard("tt_distance(no LL)", DisplayType.Verbose, calculateDistanceBetweenPoses(robot_pos_, target_pos_));
            putDashboard("tt_rotation", DisplayType.Verbose, angle_to_target_);           
            putDashboard("tt_tag", DisplayType.Verbose, sees_target_);
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
