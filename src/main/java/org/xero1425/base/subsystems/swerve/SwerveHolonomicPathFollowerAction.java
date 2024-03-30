package org.xero1425.base.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.MissingPathException;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveHolonomicPathFollowerAction extends SwerveHolonomicControllerAction {

    public interface Executor {
        void doit() ;
    }

    private class DistanceBasedAction {
        public final double Distance ;
        public final Executor Function ;
        public boolean Executed ;

        public DistanceBasedAction(double dist, Executor fun) {
            Distance = dist ;
            Function = fun ;
        }
    }

    private List<DistanceBasedAction> actions_ ;

    private String pathname_ ;
    private XeroPath path_;
    private int index_ ;
    private boolean setpose_ ;

    private double start_ ;
    private int plot_id_ ;
    private Double[] plot_data_ ;

    private boolean end_phase_;
    private XeroTimer end_timer_;

    private boolean disable_vision_ ;
    private double distance_ ;

    private boolean mirror_ ;
    private double mvalue_ ;

    private String method_ ;

    private static final String [] columns_ = {
        "time", "index",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)"
    } ;

    public SwerveHolonomicPathFollowerAction(SwerveBaseSubsystem sub, String pathname, boolean setpose, double endtime, boolean mirror, double mvalue) throws BadParameterTypeException, MissingParameterException, MissingPathException {
        super(sub) ;

        pathname_ = pathname ;
        setpose_ = setpose ;
        mirror_ = mirror ;
        mvalue_ = mvalue ;

        method_ = "NONE" ;

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot(pathname_) ;

        end_timer_ = new XeroTimer(sub.getRobot(), "holonomicpath", endtime);
        disable_vision_ = true ;

        actions_ = new ArrayList<DistanceBasedAction>() ;

        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);
        if (mirror_) {
            path_.mirrorX(mvalue_);
        }        
    }

    public double getDistance() {
        return distance_ ;
    }

    public double getPathLength() {
        XeroPathSegment seg = path_.getSegment(0, path_.getTrajectoryEntryCount() - 1) ;
        return seg.getPosition() ;
    }

    public void addDistanceBasedAction(double dist, Executor action) {
        DistanceBasedAction act = new DistanceBasedAction(dist, action) ;
        actions_.add(act) ;
    }

    public String getPathName() {
        return pathname_ ;
    }

    public void disableVision(boolean b) {
        disable_vision_ = b ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        //
        // Initialize the actions that are executed based on distance
        //
        for(DistanceBasedAction item : actions_) {
            item.Executed = false ;
        }

        if (disable_vision_) {
            getSubsystem().enableVision(false);
        }
        getSubsystem().startPlot(plot_id_, columns_);

        start_ = getSubsystem().getRobot().getTime() ;


        end_phase_ = false ;

        if (setpose_) {
            Pose2d pose = getPoseFromPath(0) ;
            getSubsystem().setPose(pose);

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("SwerveHolonomicPathFollower: Set Initial Pose ", getSubsystem().getPose());
            logger.endMessage();
        }
        else {
            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("SwerveHolonomicPathFollower: Did not set pose") ;
            logger.endMessage();            
        }

        index_ = 0 ;
    }

    private void checkActions(double distance) {
        for(DistanceBasedAction item : actions_) {
            if (distance > item.Distance && !item.Executed) {
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info);
                logger.add("PathFollowing executing lambda") ;
                logger.add("target", item.Distance);
                logger.add("actual", distance) ;
                logger.endMessage();       
                
                item.Function.doit() ;
                item.Executed = true ;
            }
        }
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {
        double velocity ;
        Pose2d target ;

        if (index_ < path_.getTrajectoryEntryCount())
        {
            target = getPoseFromPath(index_);
            velocity = getVelocityFromPath(index_) ;
            
            distance_ = getDistanceFromPath(index_);
            checkActions(distance_);
        }
        else {
            target = getPoseFromPath(index_ - 1);
            velocity = 0.0 ;
        }
        
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), target, velocity, target.getRotation()) ;
        getSubsystem().drive(speed) ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("SwerveHolonomicPathFollower Target:").add("index", index_) ;
        logger.add(", target ") ;
        logger.add(target.getX()).add(" ").add(target.getY()) ;
        logger.add(" ").add(target.getRotation().getDegrees()) ;

        Pose2d actual = getSubsystem().getPose() ;
        logger.add(",actual ") ;
        logger.add(actual.getX()).add(" ").add(actual.getY()) ;
        logger.add(" ").add(actual.getRotation().getDegrees()) ;

        logger.endMessage();

        int i = 0 ;
        plot_data_[i++] = getSubsystem().getRobot().getTime() - start_ ;
        plot_data_[i++] = (double)index_ ;
        plot_data_[i++] = target.getX() ;
        plot_data_[i++] = target.getY() ;
        plot_data_[i++] = target.getRotation().getDegrees() ;
        plot_data_[i++] = actual.getX() ;
        plot_data_[i++] = actual.getY() ;
        plot_data_[i++] = actual.getRotation().getDegrees() ;
        getSubsystem().addPlotData(plot_id_, plot_data_) ;   
        
        
        if (index_ < path_.getTrajectoryEntryCount()) {
            index_++ ;            
        }
        else {
            if (!end_phase_) {
                end_phase_ = true ;
                end_timer_.start() ;
            }

            if (index_ == path_.getTrajectoryEntryCount() && (controller().atReference() || end_timer_.isExpired())) {
                if (controller().atReference())
                    method_ = "endpath" ;
                else
                    method_ = "timer" ;

                getSubsystem().endPlot(plot_id_);
                getSubsystem().drive(new ChassisSpeeds()) ;
                getSubsystem().enableVision(true);
                setDone();

                logger.startMessage(MessageType.Info) ;
                logger.add("finished path") ;
                logger.addQuoted(pathname_);
                logger.add("pose", actual);
                logger.add("method", method_) ;
                logger.endMessage();
            }
        }
    }

    @Override
    public void cancel() {
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicPathFollower " + pathname_ ;
    }

    private double getDistanceFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return main.getPosition();
    }

    private Pose2d getPoseFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return new Pose2d(main.getX(), main.getY(), Rotation2d.fromDegrees(main.getRotation())) ;
    }

    private double getVelocityFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return main.getVelocity() ;
    }
}
