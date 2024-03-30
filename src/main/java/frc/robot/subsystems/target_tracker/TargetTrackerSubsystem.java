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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TargetTrackerSubsystem extends Subsystem {

    private static final double kCameraOffset = 0.3575 ;
    private Pose2d target_pos_;
    private double distance_between_robot_and_target_;
    private double angle_to_target_;
    private boolean sees_target_ ;
    private int target_number_ ;
    private LimeLightSubsystem ll_ ;
    private Pose2d robot_pos_ ;

    private double camera_angle_ ;
    private double camera_height_ ;
    private double target_height_ ;

    private double offset_ ;

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

    public double getOffset() {
        return offset_ ;
    }

    public void clearOffset() {
        offset_ = 0 ;
    }

    private double getTargetAngle() {
        double ret = 0.0 ;

        if (target_pos_ != null) {
            AllegroRobot2024 robotSubsystem = (AllegroRobot2024) getRobot().getRobotSubsystem();        
            robot_pos_ = robotSubsystem.getSwerve().getPose();            

            if (target_number_ == AprilTags.RED_SPEAKER_CENTER) {
            }
            else {
                double angle = Math.atan2(target_pos_.getY() - robot_pos_.getY(), target_pos_.getX() - robot_pos_.getX()) ;
                ret = XeroMath.normalizeAngleDegrees(Math.toDegrees(angle) + 180) ;
            }
        }
        return ret ;
    }

    public boolean setOffset() {
        if (!ll_.validTargets() || !ll_.hasAprilTag(target_number_))
            return false;

        MessageLogger logger = getRobot().getMessageLogger() ;

        double effective = getTargetAngle() ;

        logger.startMessage(MessageType.Info) ;
        if (effective <= 20 && effective >= -20) {
            logger.add(" case 1") ;
            offset_ = 0 ;
        }
        else if (effective < -20 && effective >= -40) {
            logger.add(" case 2") ;                
            offset_ = -5.0 ;
        }
        else if (effective < -40 && effective >= -60) {
            logger.add(" case 3") ;
            offset_ = -5.0 ;
        }
        else if (effective > 20 && effective <= 40) {
            logger.add(" case 4") ;                
            offset_ = 0.0 ;
        }
        else if (effective > 40 && effective <= 60) { 
            logger.add(" case 5") ;                
            offset_ = -15 ;
        } else {
            logger.add(" case 6") ;                
            offset_ = -7.5 ;
        }

        logger.add("effective", effective) ;
        logger.add("offset", offset_) ;
        logger.endMessage();
        
        return true ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        double angle = getTargetAngle() ;
        putDashboard("targetangle", DisplayType.Always, angle);

        if (target_pos_ != null) {
            AllegroRobot2024 robotSubsystem = (AllegroRobot2024) getRobot().getRobotSubsystem();
            robot_pos_ = robotSubsystem.getSwerve().getPose();
            MessageLogger logger = getRobot().getMessageLogger() ;

            logger.startMessage(MessageType.Debug, getLoggerID()) ;
            if (ll_.validTargets() && ll_.hasAprilTag(target_number_)) {
                logger.add("apriltag", true) ;
                sees_target_ = true ;
                angle_to_target_ = -ll_.getTX(target_number_) + offset_ ;
                distance_between_robot_and_target_ = (target_height_ - camera_height_) / Math.tan(Math.toRadians(camera_angle_ + ll_.getTY(target_number_))) + kCameraOffset ;
            }
            else {
                sees_target_ = false ;                
                logger.add("pose", false) ;
                distance_between_robot_and_target_ = calculateDistanceBetweenPoses(robot_pos_, target_pos_) ;

                if (distance_between_robot_and_target_ < 3.0) {
                    sees_target_ = true ;
                }

                angle_to_target_ = calculateAngleBetweenPoses(robot_pos_, target_pos_) + offset_ ;
            }
            logger.add("distance", distance_between_robot_and_target_) ;
            logger.add("angle", angle_to_target_) ;
            logger.endMessage();

            try {
                ShuffleboardTab tab = Shuffleboard.getTab("TargetTracking");
                tab.addDouble("tt_dist-tri", ()->distance_between_robot_and_target_) ;
                tab.addDouble("tt_dist-pose", ()->calculateDistanceBetweenPoses(robot_pos_, target_pos_)) ;
                tab.addDouble("tt_rotation", ()->angle_to_target_);
                tab.addBoolean("tt_tag", ()->sees_target_) ;
            }
            catch(Exception ex) {
            }
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
            dist = robot.getTranslation().getDistance(target.getTranslation()) ;
            dist = dist * 0.8925 + 0.0236 ;
        }

        return dist;
    }

    private static double calculateAngleBetweenPoses(Pose2d robot, Pose2d target) {
        Translation2d diff = robot.getTranslation().minus(target.getTranslation()) ;
        Rotation2d rotation = new Rotation2d(diff.getX(), diff.getY()) ;
        return XeroMath.normalizeAngleDegrees(rotation.getDegrees() - robot.getRotation().getDegrees()) ;
    }
}
