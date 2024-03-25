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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class SwerveHolonomicDynamicPathAction extends SwerveHolonomicControllerAction {

    private Trajectory traj_ ;
    private TrajectoryConfig config_ ;
    private double start_ ;
    private Rotation2d end_rot_ ;
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
    } ;    

    public interface Executor {
        void doit() ;
    } ;   

    private class DelayBasedAction {
        public final double Delay ;
        public final Executor Function ;
        public boolean Executed ;

        public DelayBasedAction(double delay, Executor fun) {
            Delay = delay ;
            Function = fun ;
        }
    }    

    private List<DelayBasedAction> actions_ ;

    public SwerveHolonomicDynamicPathAction(SwerveBaseSubsystem sub, String pathname, double maxv, double maxa, double to, Pose2dWithRotation pts[]) throws Exception {
        this(sub, pathname, maxv, maxa, to, 0.0, 0.0, pts) ;
    }    

    public SwerveHolonomicDynamicPathAction(SwerveBaseSubsystem sub, String pathname, double maxv, double maxa, double to, double rotpre, double rotpost, Pose2dWithRotation pts[]) throws Exception {
        super(sub);

        config_ = new TrajectoryConfig(maxv, maxa) ;

        double h = Math.atan2(pts[1].getY() - pts[0].getY(), pts[1].getX() - pts[0].getX()) ;
        Pose2d start = new Pose2d(pts[0].getTranslation(), Rotation2d.fromRadians(h)) ;
        pts[0] = new Pose2dWithRotation(start, pts[0].getRotation()) ;

        traj_ = TrajectoryGenerator.generateTrajectory(Arrays.asList(pts), config_) ;
        end_rot_ = pts[pts.length - 1].getRotation() ;
        pathname_ = pathname ;

        MessageLogger logger = sub.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info) ;
        logger.add("creating dynamic path") ;
        logger.add("points", pts.length) ;
        for(int i = 0 ; i < pts.length ; i++) {
            logger.add(Integer.toString(i), pts[i].toString()) ;
        }
        logger.endMessage();

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot(pathname) ;

        actions_ = new ArrayList<DelayBasedAction>() ;    
        
        timer_ = new XeroTimer(sub.getRobot(), "path", to);

        rot_pre_ = rotpre ;
        rot_post_ = rotpost ;
        rot_start_ = pts[0].getRotation().getDegrees() ;
        rot_travel_ = XeroMath.normalizeAngleDegrees(pts[pts.length - 1].getRotation().getDegrees() - rot_start_) ;
    }

    private Rotation2d rotatationValue(double elapsed) {
        if (elapsed < rot_pre_)
            return Rotation2d.fromDegrees(rot_start_) ;
        
        if (elapsed > rot_pre_ + traj_.getTotalTimeSeconds())
            return Rotation2d.fromDegrees(XeroMath.normalizeAngleDegrees(rot_start_ + rot_travel_)) ;

        double span = traj_.getTotalTimeSeconds() - rot_pre_ - rot_post_ ;
        double pcnt = (elapsed - rot_pre_) / span ;
        return Rotation2d.fromDegrees(XeroMath.normalizeAngleDegrees(rot_start_ + pcnt * rot_travel_)) ;
    }

    public void addDelayBasedAction(double delay, Executor action) {
        DelayBasedAction act = new DelayBasedAction(delay, action) ;
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
        for(DelayBasedAction item : actions_) {
            item.Executed = false ;
        }        
    }

    private void checkActions(double time) {
        for(DelayBasedAction item : actions_) {
            if (time > item.Delay && !item.Executed) {
                MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
                logger.startMessage(MessageType.Info);
                logger.add("PathFollowing executing lambda") ;
                logger.add("target", item.Delay);
                logger.add("actual", time) ;
                logger.endMessage();       
                
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
        logger.add("SwerveHolonomicPathFollower Target:").add("time", now - start_) ;
        logger.add("target ", target.poseMeters) ;
        logger.add("actual", actual) ;
        logger.endMessage();

        int i = 0 ;
        plot_data_[i++] = getSubsystem().getRobot().getTime() - start_ ;
        plot_data_[i++] = target.poseMeters.getX() ;
        plot_data_[i++] = target.poseMeters.getY() ;
        plot_data_[i++] = target.poseMeters.getRotation().getDegrees() ;
        plot_data_[i++] = actual.getX() ;
        plot_data_[i++] = actual.getY() ;
        plot_data_[i++] = actual.getRotation().getDegrees() ;
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
