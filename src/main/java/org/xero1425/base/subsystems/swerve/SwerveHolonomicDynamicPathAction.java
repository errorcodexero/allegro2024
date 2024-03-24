package org.xero1425.base.subsystems.swerve;

import java.util.Arrays;

import org.xero1425.base.utils.Pose2dWithRotation;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

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

    private int plot_id_ ;
    private Double[] plot_data_ ;    
    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "th (deg)",
        "ax (m)", "ay (m)", "ah (deg)"
    } ;    

    public SwerveHolonomicDynamicPathAction(SwerveBaseSubsystem sub, String pathname, double maxv, double maxa, Pose2dWithRotation pts[]) throws Exception {
        super(sub);

        config_ = new TrajectoryConfig(maxv, maxa) ;
        traj_ = TrajectoryGenerator.generateTrajectory(Arrays.asList(pts),  config_) ;
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
    }

    @Override
    public void start() throws Exception {
        super.start();

        getSubsystem().startPlot(plot_id_, columns_);        
        start_ = getSubsystem().getRobot().getTime() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double now = getSubsystem().getRobot().getTime() ;
        Trajectory.State target = traj_.sample(now - start_) ;

        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), target, end_rot_) ;
        getSubsystem().drive(speed) ;

        Pose2d actual = getSubsystem().getPose() ;
        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("SwerveHolonomicPathFollower Target:").add("time", now - start_) ;
        logger.add(", target ") ;
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

        if (controller().atReference()) {
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
