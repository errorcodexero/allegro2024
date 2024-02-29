package org.xero1425.base.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveTrackAngle extends Action {
    private SwerveBaseSubsystem swerve_ ;
    private DoubleSupplier target_ ;
    private double p_ ;
    private double postol_ ;
    private boolean is_at_target_ ;
    private double err_ ;

    public SwerveTrackAngle(SwerveBaseSubsystem swerve, DoubleSupplier target, double postol) throws BadParameterTypeException, MissingParameterException {
        super(swerve.getRobot().getMessageLogger());
        swerve_ = swerve ;
        target_ = target ;
        postol_ = postol ;

        p_ = swerve.getSettingsValue("angle-tracker:p").getDouble() ;
    }

    public boolean isAtTarget() {
        return is_at_target_ ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        is_at_target_ = false ;
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

        err_ = target_.getAsDouble() ;
        if (Math.abs(err_) < postol_) {
            is_at_target_ = true ;
        }
        else {
            is_at_target_ = false ;
        }

        MessageLogger logger = swerve_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, swerve_.getLoggerID());
        logger.add("err", err_) ;
        logger.add("postel", postol_) ;
        logger.add("isAt", is_at_target_) ;
        logger.endMessage();

        double angvel = Math.toRadians(err_ * p_) ;
        ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, angvel) ;
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
        return prefix(indent) + "SwerveTrackAngle";
    }
}
