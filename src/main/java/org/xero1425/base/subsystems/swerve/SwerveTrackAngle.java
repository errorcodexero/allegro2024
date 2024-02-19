package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveTrackAngle extends Action {
    private SwerveBaseSubsystem swerve_ ;
    private double angle_ ;
    private double p_ ;
    private double postol_ ;
    private double veltol_ ;
    private boolean is_at_target_ ;
    private double err_ ;

    private XeroTimer timer_ ;

    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // Data for each loop of the plot
    private Double data_[] ;    

    private static String [] columns_ = { 
        "time", 
        "target (deg)", 
        "cur (deg)",
    } ;

    public SwerveTrackAngle(SwerveBaseSubsystem swerve, double angle, double postol, double veltol) throws BadParameterTypeException, MissingParameterException {
        super(swerve.getRobot().getMessageLogger());
        swerve_ = swerve ;
        angle_ = angle ;

        p_ = swerve.getSettingsValue("angle-tracker:p").getDouble() ;

        plot_id_ = swerve.initPlot("swerve-track-angle") ;
        data_ = new Double[columns_.length] ;
    }

    public void setAngle(double angle) {
        angle_ = angle ;

        double err = angle_ - swerve_.getPose().getRotation().getDegrees() ;        
        if (Math.abs(err) < postol_ && Math.abs(swerve_.getRotationalVelocity()) < veltol_) {
            is_at_target_ = true ;
        }
        else {
            is_at_target_ = false ;
        }
    }

    public boolean isAtTarget() {
        return is_at_target_ ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        is_at_target_ = false ;
        start_ = swerve_.getRobot().getTime() ;
        swerve_.startPlot(plot_id_, columns_) ;

        timer_ = new XeroTimer(swerve_.getRobot(), "track-angle-timer", 3.0) ;
    }

    public double getError() {
        return err_ ;
    }

    public double getRotVel() {
        return swerve_.getRotationalVelocity() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        err_ = angle_ - swerve_.getPose().getRotation().getDegrees() ;        
        if (Math.abs(err_) < postol_) {
            is_at_target_ = true ;
        }
        else {
            is_at_target_ = false ;
        }

        double angvel = Math.toRadians(err_ * p_) ;
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, angvel) ;
        swerve_.drive(speeds) ;

        if (timer_.isRunning()) {
            data_[0] = swerve_.getRobot().getTime() - start_ ;
            data_[1] = angle_ ;
            data_[2] = swerve_.getPose().getRotation().getDegrees() ;
            swerve_.addPlotData(plot_id_, data_) ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        try {
            swerve_.stop() ;
            swerve_.endPlot(plot_id_) ;
        }
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "SwerveTrackAngle";
    }
}
