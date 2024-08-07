package org.xero1425.base.subsystems.tankdrive;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.MissingPathException;
import org.xero1425.misc.PIDACtrl;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.misc.XeroMath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/// \file

/// \brief This class implements an action that follows a path.  The path is given by name and should have been
/// loaded by the XeroPathManager.  This class follows a traditional "Path Following" algorithm, where the position
/// velocity, and acceleration for the left and right side of the tank drive are generated by a path generation program
/// and this action just follows these paths as closely as possible.
public class TankDrivePathFollowerAction extends TankDrivePathAction {    
    // The data point we are currently processing
    private int index_ ;

    // The starting position of the left side of the robot
    private double left_start_ ;

    // The starting position of the right side of the robot
    private double right_start_ ;

    // The starting time for the path.
    private double start_time_ ;

    // The follower for the left side of the robot
    private PIDACtrl left_follower_ ;

    // The follower for the right side of the robot
    private PIDACtrl right_follower_ ;

    // If true, follow the path in reverse
    private boolean reverse_ ;

    // The starting angle for the robot
    private double start_angle_ ;

    // The starting angle for the path
    private double target_start_angle_ ;

    // The PID controller used to do angle correction for the robot heading
    private PIDCtrl angle_correction_pid_ ;

    // The ID for the plot generated by this action
    private int plot_id_ ;

    // The per robot loop plot data for this action
    private Double [] plot_data_ ;

    // The data columns to plot with this action
    private static final String[] plot_columns_ = {             
        "time (s)", 
        "ltpos (m)", "lapos (m)", "ltvel (m/s)", "lavel (m/s)", "ltaccel (m/s/s)", "laaccel (m/s/s)", "lout (volts)","lticks (ticks)","lvout (volts)","laout (volts)","lpout (volts)","ldout (volts)","lerr",
        "rtpos (m)", "rapos (m)", "rtvel (m/s)", "ravel (m/s)", "rtaccel (m/s/s)", "raaccel (m/s/s)", "rout (volts)","rticks (ticks)","rvout (volts)","raout (volts)","rpout (volts)","rdout (volts)","rerr",
        "thead (degs)", "ahead (degs)", "angerr (degs)", "angcorr", "pid-p", "pid-i", "pid-d", "pid-f"
    } ;

    // The data elements to extract from the path
    private final int LeftSide = 0 ;
    private final int RightSide = 1 ;

    /// \brief Create a new path follower action
    /// \param drive the tankdrive subsystem
    /// \param path the name of the path to follow, should be loaded by the XeroPathManager
    /// \param reverse if true, follow the path in reverse
    public TankDrivePathFollowerAction(TankDriveSubsystem drive, String path, boolean reverse)
            throws MissingPathException, BadParameterTypeException, MissingParameterException {
        super(drive, path) ;

        reverse_ = reverse ;

        left_follower_ = new PIDACtrl(drive.getRobot().getSettingsSupplier(), "subsystems:" + getSubsystem().getName() + ":follower:left", false) ;
        right_follower_ = new PIDACtrl(drive.getRobot().getSettingsSupplier(), "subsystems:" + getSubsystem().getName() + ":follower:right", false) ;
        angle_correction_pid_ = new PIDCtrl(drive.getRobot().getSettingsSupplier(), "subsystems:" + getSubsystem().getName() + ":angle_correction", false) ;

        plot_id_ = drive.initPlot(toString(0)) ;
        plot_data_ = new Double[plot_columns_.length] ;
    }

    /// \brief Start the path folowing action.  Record the initial state of the robot.
    @Override
    public void start() throws Exception {
        super.start() ;

        getSubsystem().setRecording(true);

        left_start_ = getSubsystem().getLeftDistance() ;
        right_start_ = getSubsystem().getRightDistance() ;

        index_ = 0 ;
        start_time_ = getSubsystem().getRobot().getTime() ;
        start_angle_ = getSubsystem().getAngle().getDegrees() ;
        target_start_angle_ = getPath().getSegment(LeftSide, 0).getHeading() ;

        XeroPathSegment lseg = getPath().getSegment(LeftSide, 0) ;
        XeroPathSegment rseg = getPath().getSegment(RightSide, 0) ;

        Pose2d startpose = new Pose2d((lseg.getX() + rseg.getX()) / 2.0, (lseg.getY() + rseg.getY()) / 2.0, Rotation2d.fromDegrees(lseg.getHeading())) ;
        getSubsystem().setPose(startpose);

        getSubsystem().startPlot(plot_id_, plot_columns_);
    }

    /// \brief Run the path following action.  Called each robot loop to adjust the left and right velocities of the
    /// tankdrive drivebase to keep the robot on the desired path.
    @Override
    public void run() {
        TankDriveSubsystem td = getSubsystem();
        XeroRobot robot = td.getRobot() ;

        MessageLogger logger = robot.getMessageLogger();
        logger.startMessage(MessageType.Debug, getActionLoggerID()) ;
        logger.add("index", index_) ;

        if (index_ < getPath().getTrajectoryEntryCount())
        {
            double dt = robot.getDeltaTime();
            XeroPathSegment lseg = getPath().getSegment(LeftSide, index_) ;
            XeroPathSegment rseg = getPath().getSegment(RightSide, index_) ;

            double laccel, lvel, lpos ;
            double raccel, rvel, rpos ;
            double thead, ahead ;

            // Compute the desired left and right side parameters
            if (reverse_)
            {
                laccel = -rseg.getAccel() ;
                lvel = -rseg.getVelocity() ;
                lpos = -rseg.getPosition() ;
                raccel = -lseg.getAccel() ;
                rvel = -lseg.getVelocity() ;
                rpos = -lseg.getPosition() ;
            }
            else
            {
                laccel = lseg.getAccel() ;
                lvel = lseg.getVelocity() ;
                lpos = lseg.getPosition() ;
                raccel = rseg.getAccel() ;
                rvel = rseg.getVelocity() ;
                rpos = rseg.getPosition() ;
            }

            // Compute the actual and target robot headings
            thead = XeroMath.normalizeAngleDegrees(lseg.getHeading() - target_start_angle_) ;
            ahead = XeroMath.normalizeAngleDegrees(getSubsystem().getAngle().getDegrees() - start_angle_) ;   

            // Compute teh distance travled by each side of the robot
            double ldist, rdist ;
            ldist = td.getLeftDistance() - left_start_ ;
            rdist = td.getRightDistance() - right_start_ ;
            
            // Use a PID controller with feed forward for velocity and acceleration to determine
            // the desired left and right motor power.
            double lout = left_follower_.getOutput(laccel, lvel, lpos, ldist, dt) ;
            double rout = right_follower_.getOutput(raccel, rvel, rpos, rdist, dt) ;

            // Compute an adjustment to the left and right power to keep the robot on the correct
            // heading.  Note: egative angle is clockwise
            double angerr = XeroMath.normalizeAngleDegrees(ahead - thead) ;
            double angcorr = angle_correction_pid_.getOutput(0, angerr, dt) ;

            lout -= angcorr ;
            rout += angcorr ;

            // Set the power
            td.setPower(lout, rout) ;

            // Add entries to the log file
            logger.add(", time", robot.getTime() - start_time_) ;
            logger.add(", left", lout) ;
            logger.add(", right", rout) ;
            logger.add(", angerr(degs)", angerr) ;
            logger.add(", angcorr(v)", angcorr) ;
            logger.add(", path-x", (lseg.getX() + rseg.getX()) / 2.0) ;
            logger.add(", path-y", (lseg.getY() + rseg.getY()) / 2.0) ;
            logger.add(", path-a", thead) ;
            logger.add(", robot-x", getSubsystem().getPose().getX()) ;
            logger.add(", robot-y", getSubsystem().getPose().getY()) ;
            logger.add(", robot-a", ahead) ;

            // Add the plot data

            plot_data_[0] = robot.getTime() - start_time_ ;

            // Left side
            plot_data_[1] = lpos ;
            plot_data_[2] = td.getLeftDistance() - left_start_ ;
            plot_data_[3] = lvel ;
            plot_data_[4] = td.getLeftVelocity() ;
            plot_data_[5] = laccel ;
            plot_data_[6] = td.getLeftAcceleration() ;
            plot_data_[7] = lout ;
            plot_data_[8] = (double)td.getLeftTick() ;
            plot_data_[9] = left_follower_.getVPart() ;
            plot_data_[10] = left_follower_.getAPart() ;
            plot_data_[11] = left_follower_.getPPart() ;
            plot_data_[12] = left_follower_.getDPart() ;
            plot_data_[13] = left_follower_.getLastError() ;                                                

            // Right side
            plot_data_[14] = rpos ;
            plot_data_[15] = td.getRightDistance() - right_start_ ;
            plot_data_[16] = rvel ;
            plot_data_[17] = td.getRightVelocity() ;
            plot_data_[18] = raccel ;
            plot_data_[19] = td.getRightAcceleration() ;
            plot_data_[20] = rout ;
            plot_data_[21] = (double)td.getRightTick() ;
            plot_data_[22] = right_follower_.getVPart() ;
            plot_data_[23] = right_follower_.getAPart() ;
            plot_data_[24] = right_follower_.getPPart() ;
            plot_data_[25] = right_follower_.getDPart() ;
            plot_data_[26] = right_follower_.getLastError() ;

            plot_data_[27] = thead ;
            plot_data_[28] = ahead ;
            plot_data_[29] = angerr ;
            plot_data_[30] = angcorr ;
            plot_data_[31] = angle_correction_pid_.getPComponent() ;
            plot_data_[32] = angle_correction_pid_.getPComponent() ;
            plot_data_[33] = angle_correction_pid_.getPComponent() ;
            plot_data_[34] = angle_correction_pid_.getPComponent() ;
            td.addPlotData(plot_id_, plot_data_);
        }
        logger.endMessage();
        index_++ ;

        if (index_ == getPath().getTrajectoryEntryCount())
        {
            getSubsystem().setRecording(false);
            td.endPlot(plot_id_);
            td.setPower(0.0, 0.0) ;
            setDone() ;
        }
    }

    /// \brief cancel the action, setting tank drive power to zero
    @Override
    public void cancel() {
        super.cancel() ;
        index_ = getPath().getTrajectoryEntryCount() ;

        getSubsystem().setPower(0.0, 0.0) ;
        getSubsystem().endPlot(plot_id_);
        getSubsystem().setRecording(false);
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action 
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDriveFollowPath-" + getPathName() ;
        return ret ;
    }
}