package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.IVisionLocalization;
import org.xero1425.base.IVisionLocalization.LocationData;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

public class SwerveVisionProcessing {

    private static final double kZeroPosThreshold = 0.2 ;

    private enum VisionParamsType {
        SingleNear,
        SingleFar,
        MultiNear,
        MultiFar
    };

    private int logger_id_ ;
    private int forced_cycles_ ;

    private SwerveBaseSubsystem sub_ ;
    private IVisionLocalization vision_ ;

    private VisionParamsType params_type_ ;

    private double single_tag_threshold_;
    private Vector<N3> single_tag_near_params_ ;
    private Vector<N3> single_tag_far_params_ ;

    private double multi_tag_threshold_;
    private Vector<N3> multi_tag_near_params_ ;
    private Vector<N3> multi_tag_far_params_ ;

    private double vision_reject_threshold_;

    private Pose2d vision_pose_ ;

    public SwerveVisionProcessing(SwerveBaseSubsystem sub, IVisionLocalization vision) throws BadParameterTypeException, MissingParameterException {
        vision_ = vision ;
        sub_ = sub ;

        single_tag_threshold_ = sub_.getSettingsValue("estimator:single-threshold").getDouble();
        multi_tag_threshold_ = sub_.getSettingsValue("estimator:single-threshold").getDouble();
        vision_reject_threshold_ = sub_.getSettingsValue("estimator:vision-reject-threshold").getDouble();

        single_tag_near_params_ = getParams(sub, "vision:single-near");
        single_tag_far_params_ = getParams(sub, "vision:single-far");
        multi_tag_near_params_ = getParams(sub, "vision:multi-near");
        multi_tag_far_params_ = getParams(sub, "vision:multi-far");

        logger_id_ = sub.getRobot().getMessageLogger().registerSubsystem("vision");
    }

    public Pose2d getCurrentPose() {
        Pose2d ret = null ;

        LocationData lc = vision_.getLocation(sub_.getPose());
        if (lc != null) { 
            ret = lc.location.toPose2d();
        }

        return ret;
    }

    public boolean hasTargets() {
        return vision_.getTagCount() > 0 ;
    }

    public void processVision() {
        MessageLogger logger = sub_.getRobot().getMessageLogger();
        boolean ignore = false;

        LocationData lc = vision_.getLocation(sub_.getPose()) ;
        setVisionParams();
        if (lc != null) {
            vision_pose_ = lc.location.toPose2d();

            Translation2d rpose = sub_.getPose().getTranslation() ;
            if (Math.abs(rpose.getX()) < kZeroPosThreshold && Math.abs(rpose.getY()) < kZeroPosThreshold) {
                forced_cycles_ = 50 ;
            }
            else if (forced_cycles_ > 0) {
                forced_cycles_-- ;
            }
            else {
                //
                // There are two strategies for rejection vision samples if they do not seem to be valid.  The simple
                // strategy is that if the samples are more than 1 meter from the current pose of the robot, we ignore
                // them.  The advanced strategy is that if we see more than one april tag, we always trust the vision
                // sample data.  If we see only one april tag, it must be within a given distance (single_tag_distance_threshold_).
                //
                double dist = vision_pose_.getTranslation().getDistance(sub_.getPose().getTranslation());
                if (dist >= vision_reject_threshold_) {
                    ignore = true ;
                }
            }

            if (!ignore) {
                sub_.getEstimator().addVisionMeasurement(vision_pose_, lc.when) ; 
            }
        }      

        logger.startMessage(MessageType.Debug, logger_id_);
        logger.add("Vision: ");
        logger.add("params", params_type_.toString());
        logger.add("dbx", sub_.getPose().getX());
        logger.add("dby", sub_.getPose().getY());
        logger.add("dbheading", sub_.getPose().getRotation().getDegrees());
        if (vision_pose_ != null) {
            logger.add("vsx", vision_pose_.getX());
            logger.add("vsy", vision_pose_.getY());
            logger.add("vsheading", vision_pose_.getRotation().getDegrees());
        }
        logger.endMessage();
    }

    private void setVisionParams(VisionParamsType vtype)
    {
        MessageLogger logger = sub_.getRobot().getMessageLogger();
        if (params_type_ != vtype) {
            switch(vtype) {
                case SingleNear:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Single Near").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(single_tag_near_params_);
                    break ;

                case SingleFar:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Single Far").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(single_tag_far_params_);
                    break; 

                case MultiNear:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Multi Near").endMessage();
                    sub_.getEstimator().setVisionMeasurementStdDevs(multi_tag_near_params_);
                    break ;

                case MultiFar:
                    logger.startMessage(MessageType.Info).add("Changed vision parameters to Multi Far").endMessage();                
                    sub_.getEstimator().setVisionMeasurementStdDevs(multi_tag_far_params_);
                    break; 
            }
            params_type_ = vtype;
        }
    }

    private void setVisionParams() {
        if (vision_.getTagCount() == 1) {
            if (vision_.getDistance() < single_tag_threshold_) {
                setVisionParams(VisionParamsType.SingleNear);
            }
            else {
                setVisionParams(VisionParamsType.SingleFar);
            }
        }
        else if (vision_.getTagCount() > 1) {
            if (vision_.getDistance() < multi_tag_threshold_) {
                setVisionParams(VisionParamsType.MultiNear);
            }
            else {
                setVisionParams(VisionParamsType.MultiFar);
            }                
        }
        else if (vision_.getTagCount() == 0) {
            setVisionParams(VisionParamsType.SingleFar);
        }
    }

    public static Vector<N3> getParams(Subsystem sub, String str) throws BadParameterTypeException, MissingParameterException {
        double px = sub.getSettingsValue("estimator:" + str + ":x").getDouble();
        double py = sub.getSettingsValue("estimator:" + str + ":y").getDouble();
        double ph = sub.getSettingsValue("estimator:" + str + ":heading").getDouble();

        return VecBuilder.fill(px, py, ph);
    }
}
