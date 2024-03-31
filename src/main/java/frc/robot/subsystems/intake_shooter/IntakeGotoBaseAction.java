package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.misc.ISettingsSupplier;

public abstract class IntakeGotoBaseAction extends Action {
    
    private double kTiltTolerance = 3.0 ;

    private enum State {
        Idle,
        AlightTilt,
        MoveBoth,
        FinishTilt,
        Done
    } ;

    private IntakeShooterSubsystem subsystem_;
    private MCMotionMagicAction updown_action_;
    private MCMotionMagicAction tilt_action_;

    private double tilt_min_ ;
    private double tilt_max_ ;
    private double tilt_target_ ;

    private double updown_min_ ;
    private double updown_max_ ;
    private double updown_target_ ;

    private State state_ ;

    protected IntakeGotoBaseAction(IntakeShooterSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;

        ISettingsSupplier settings = subsystem.getRobot().getSettingsSupplier() ;

        tilt_min_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:stow").getDouble() ;
        tilt_max_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:collect").getDouble() ;

        updown_min_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:stow").getDouble() ;
        updown_max_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:collect").getDouble() ;

        updown_action_ = new MCMotionMagicAction(subsystem_.getUpDown(), "pids:position", 0.0, 2.0, 2.0) ;
        tilt_action_ = new MCMotionMagicAction(subsystem_.getTilt(), "pids:position", 0.0, 2.0, 2.0) ;
        
        state_ = State.Idle ;
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return subsystem_ ;
    }

    private double calcTiltFromUpdown(double updown) {
        double ret = Double.MAX_VALUE ;

        if (updown < updown_min_)
            ret = tilt_min_ ;
        else if (updown > updown_max_)
            ret = tilt_max_ ;
        else
            ret = tilt_min_ + (tilt_max_ - tilt_min_) * (updown - updown_min_) / (updown_max_ - updown_min_) ;

        return ret ;
    }

    protected void gotoPosition(String updown, String tilt) throws Exception {
        gotoPosition(subsystem_.getUpDown().getSettingsValue(updown).getDouble(), subsystem_.getTilt().getSettingsValue(tilt).getDouble()) ;
    }

    protected void gotoPosition(double updown, double tilt) throws BadMotorRequestException, MotorRequestFailedException {
        updown_target_ = updown ;
        tilt_target_ = tilt ;

        double updownpos = subsystem_.getUpDown().getPosition() ;
        double tiltpos = subsystem_.getTilt().getPosition() ;
        double tiltstart = calcTiltFromUpdown(updownpos) ;

        if (Math.abs(tiltpos - tiltstart) > kTiltTolerance) {
            //
            // The tilt is off of the value path, so move the tilt to the value position
            // for the current updown location
            //
            tilt_action_.setTarget(tiltstart) ;
            subsystem_.getTilt().setAction(tilt_action_, true) ;
            state_ = State.AlightTilt ;
        }
        else {
            //
            // Go directly to the target position
            //

            //
            // Find the tilt position along the value transition path and set the
            // tile target to that path
            //
            tilt_action_.setTarget(calcTiltFromUpdown(updown)) ;

            //
            // Set the updown target
            //
            updown_action_.setTarget(updown);

            //
            // Now move the intake along the prescribed path
            //
            subsystem_.getTilt().setAction(tilt_action_, true) ;
            subsystem_.getUpDown().setAction(updown_action_, true) ;

            state_ = State.MoveBoth ;
        }
    }

    protected void runGoto() throws BadMotorRequestException, MotorRequestFailedException {
        switch(state_) {
            case Idle:
                break ;

            case AlightTilt:
                if (tilt_action_.isDone()) {
                    tilt_action_.setTarget(calcTiltFromUpdown(updown_target_)) ;                    
                    subsystem_.getTilt().setAction(tilt_action_, true) ;

                    updown_action_.setTarget(updown_target_) ;
                    subsystem_.getUpDown().setAction(updown_action_, true) ;
                    state_ = State.MoveBoth ;
                }
                break ;

            case MoveBoth:
                if (tilt_action_.isDone() && updown_action_.isDone()) {
                    //
                    // Move the tilt to its final state
                    //
                    tilt_action_.setTarget(tilt_target_) ;
                    subsystem_.getTilt().setAction(tilt_action_, true) ;
                    state_ = State.FinishTilt ;
                }
                break ;

            case FinishTilt:
                if (tilt_action_.isDone())
                    state_ = State.Done ;
                break ;

            case Done:
                break ;
        }
    }

    protected void cancelGoto() {
        updown_action_.cancel() ;
        tilt_action_.cancel() ;
        state_ = State.Done ;
    }

    protected boolean isAtTarget() {
        return state_ == State.Done ;
    }
}
