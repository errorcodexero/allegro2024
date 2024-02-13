package org.xero1425.base.subsystems.swerve;

import org.xero1425.base.actions.Action;
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

    public SwerveTrackAngle(SwerveBaseSubsystem swerve, double angle, double postol, double veltol) throws BadParameterTypeException, MissingParameterException {
        super(swerve.getRobot().getMessageLogger());
        swerve_ = swerve ;
        angle_ = angle ;

        p_ = swerve.getSettingsValue("angle-tracker:p").getDouble() ;
    }

    public void setAngle(double angle) {
        angle_ = angle ;
    }

    public boolean isAtTarget() {
        return is_at_target_ ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        is_at_target_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double err = angle_ - swerve_.getPose().getRotation().getDegrees() ;        
        if (Math.abs(err) < postol_ && Math.abs(swerve_.getRotationalVelocity()) < veltol_) {
            is_at_target_ = true ;
        }

        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, err * p_) ;
        swerve_.drive(speeds) ;
    }

    @Override
    public void cancel() {
        super.cancel();
        try {
            swerve_.stop() ;
        }
        catch(Exception ex) {
        }
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "SwerveRotateToAngle";
    }
}
