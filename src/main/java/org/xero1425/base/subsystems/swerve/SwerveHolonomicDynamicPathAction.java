package org.xero1425.base.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.utils.Pose2dWithRotation;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class SwerveHolonomicDynamicPathAction extends SwerveHolonomicControllerAction {

    private Trajectory traj_ ;
    private TrajectoryConfig config_ ;
    private double start_ ;
    private String pathname_ ;

    private XeroTimer timer_ ;
    private boolean waiting_ ;

    private double rot_travel_ ;
    private double rot_start_ ;
    private double rot_pre_ ;
    private double rot_post_ ;

    private int plot_id_ ;
    private Double[] plot_data_ ;    
    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "th (deg)",
        "ax (m)", "ay (m)", "ah (deg)",
        "tr (deg)", "ar (deg)"
    } ;    

    public interface Executor {
        void doit() ;
    } ;   

    private class LocationBasedAction {
        public final Translation2d Location ;
        public final double Distance ;
        public final Executor Function ;
        public boolean Executed ;

        public LocationBasedAction(Translation2d loc, double distance, Executor fun) {
            Location = loc ;
            Distance = distance ;
            Function = fun ;
        }
    }    

    private List<LocationBasedAction> actions_ ;

    public SwerveHolonomicDynamicPathAction(SwerveBaseSubsystem sub, String pathname, double maxv, double maxa, double to, Pose2dWithRotation pts[]) throws Exception {
        this(sub, pathname, maxv, maxa, to, 0.0, 0.0, pts) ;
    }    

    public SwerveHolonomicDynamicPathAction(SwerveBaseSubsystem sub, String pathname, double maxv, double maxa, double to, double rotpre, double rotpost, Pose2dWithRotation pts[]) throws Exception {
        super(sub);

        config_ = new TrajectoryConfig(maxv, maxa) ;

        double h = Math.atan2(pts[1].getY() - pts[0].getY(), pts[1].getX() - pts[0].getX()) ;
        Pose2d start = new Pose2d(pts[0].getTranslation(), Rotation2d.fromRadians(h)) ;
        pts[0] = new Pose2dWithRotation(start, pts[0].getRobotRotation()) ;

        MessageLogger logger = sub.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info) ;
        logger.add("SwerveHolonomicDynamicPathAction(start/end)") ;
        logger.add("start", pts[0]) ;
        logger.add("end", pts[pts.length - 1]) ;
        logger.endMessage();

        List<Pose2d> poses = Arrays.asList(pts) ;
        traj_ = TrajectoryGenerator.generateTrajectory(poses, config_) ;
        pathname_ = pathname ;

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot(pathname) ;

        actions_ = new ArrayList<LocationBasedAction>() ;    
        
        timer_ = new XeroTimer(sub.getRobot(), "path", to);

        rot_pre_ = rotpre ;
        rot_post_ = rotpost ;
        rot_start_ = pts[0].getRobotRotation().getDegrees() ;
        rot_travel_ = XeroMath.normalizeAngleDegrees(pts[pts.length - 1].getRobotRotation().getDegrees() - rot_start_) ;
    }

    private Rotation2d rotatationValue(double elapsed) {
        if (elapsed < rot_pre_)
            return Rotation2d.fromDegrees(rot_start_) ;
        
        if (elapsed > traj_.getTotalTimeSeconds() - rot_post_)
            return Rotation2d.fromDegrees(XeroMath.normalizeAngleDegrees(rot_start_ + rot_travel_)) ;

        //
        // The total time we are rotating
        //
        double span = traj_.getTotalTimeSeconds() - rot_pre_ - rot_post_ ;

        //
        // How far we are along the ramp
        //
        double pcnt = (elapsed - rot_pre_) / span ;

        //
        // The rotation where we should be
        // 
        double rv  = pcnt * rot_travel_ + rot_start_ ;

        // MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        // logger.startMessage(MessageType.Info) ;
        // logger.add("compute rotation") ;
        // logger.add("pre", rot_pre_) ;
        // logger.add("post", rot_post_) ;
        // logger.add("rot_travel", rot_travel_) ;
        // logger.add("span", span) ;
        // logger.add("pcnt", pcnt) ;
        // logger.add("rv", rv) ;
        // logger.add("rvnorm", XeroMath.normalizeAngleDegrees(rv)) ;
        // logger.endMessage();

        return Rotation2d.fromDegrees(XeroMath.normalizeAngleDegrees(rv)) ;
    }

    public void addLocationBasedAction(Translation2d loc, double dist, Executor action) {
        LocationBasedAction act = new LocationBasedAction(loc, dist, action) ;
        actions_.add(act) ;
    }    

    @Override
    public void start() throws Exception {
        super.start();

        waiting_ = false ;

        getSubsystem().startPlot(plot_id_, columns_);        
        start_ = getSubsystem().getRobot().getTime() ;

        //
        // Initialize the actions that are executed based on distance
        //
        for(LocationBasedAction item : actions_) {
            item.Executed = false ;
        }        
    }

    private void checkActions(double time) {
        Translation2d curloc = getSubsystem().getPose().getTranslation();

        for(LocationBasedAction item : actions_) {
            double act = curloc.getDistance(item.Location) ;
            if (act < item.Distance && !item.Executed) {
                item.Function.doit() ;
                item.Executed = true ;
            }
        }
    }

    @Override
    public void run() throws Exception {
        super.run();

        double now = getSubsystem().getRobot().getTime() ;
        double elapsed = now - start_ ;
        Trajectory.State target = traj_.sample(now - start_) ;

        checkActions(elapsed);

        Rotation2d rot = rotatationValue(elapsed) ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), target, rot) ;
        getSubsystem().drive(speed) ;

        Pose2d actual = getSubsystem().getPose() ;
        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info) ;
        logger.add("SwerveHolonomicPathFollower Target:") ;
        logger.add("time", elapsed) ;
        logger.add("target ", target.poseMeters) ;
        logger.add("actual", actual) ;
        logger.add("stangle", rot_start_) ;
        logger.add("enangle", rot_start_ + rot_travel_) ;
        logger.add("rotangle", rot) ;
        logger.endMessage();

        int i = 0 ;
        plot_data_[i++] = getSubsystem().getRobot().getTime() - start_ ;
        plot_data_[i++] = target.poseMeters.getX() ;
        plot_data_[i++] = target.poseMeters.getY() ;
        plot_data_[i++] = target.poseMeters.getRotation().getDegrees() ;
        plot_data_[i++] = actual.getX() ;
        plot_data_[i++] = actual.getY() ;
        plot_data_[i++] = actual.getRotation().getDegrees() ;
        plot_data_[i++] = rot.getDegrees() ;
        plot_data_[i++] = actual.getRotation().getDegrees();

        getSubsystem().addPlotData(plot_id_, plot_data_) ;   

        if (!waiting_ && elapsed >= traj_.getTotalTimeSeconds()) {
            waiting_ = true ;
            timer_.start() ;
        }

        if ((waiting_ && timer_.isExpired()) || (elapsed >= traj_.getTotalTimeSeconds() && controller().atReference())) {
            getSubsystem().endPlot(plot_id_);
            getSubsystem().drive(new ChassisSpeeds()) ;
            setDone() ;

            logger.startMessage(MessageType.Info) ;
            logger.add("finished path") ;
            logger.addQuoted(pathname_);
            logger.add("pose", actual);
            logger.add("duration", now - start_) ;
            logger.endMessage();
        }        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicDynamicPathAction";
    }
}
