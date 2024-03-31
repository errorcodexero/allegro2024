package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public abstract class IntakeGotoBaseAction extends Action {
    
    private double kTiltTolerance = 3.0 ;

    private enum State {
        Idle,
        AlignTilt,
        MoveBoth,
        FinishTilt,
        Done
    } ;

    private IntakeShooterSubsystem subsystem_;
    private MCMotionMagicAction updown_action_;
    private MCMotionMagicAction tilt_action_;

    private MCTrackPosAction tilt_track_ ;

    private double tilt_stow_ ;
    private double tilt_collect_ ;
    private double tilt_target_ ;

    private double updown_collect_ ;
    private double updown_stow_ ;
    private double updown_target_ ;

    private State state_ ;

    protected IntakeGotoBaseAction(IntakeShooterSubsystem subsystem) throws Exception {
        super(subsystem.getRobot().getMessageLogger());
        subsystem_ = subsystem;

        ISettingsSupplier settings = subsystem.getRobot().getSettingsSupplier() ;

        tilt_stow_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:stow").getDouble() ;
        tilt_collect_ = settings.get("subsystems:" + subsystem_.getTilt().getName() + ":targets:collect").getDouble() ;

        updown_collect_ = settings.get("subsystems:" + subsystem_.getUpDown().getName() + ":targets:collect").getDouble() ;
        updown_stow_ = settings.get("subsystems:" + subsystem_.getUpDown().getName() + ":targets:stow").getDouble() ;

        updown_action_ = new MCMotionMagicAction(subsystem_.getUpDown(), "pids:position", 0.0, 4.0, 4.0) ;
        tilt_action_ = new MCMotionMagicAction(subsystem_.getTilt(), "pids:position", 0.0, 4.0, 4.0) ;
              
        state_ = State.Idle ;
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return subsystem_ ;
    }

    private double calcTiltFromUpdown(double updown) {
        double ret = Double.MAX_VALUE ;
     
        if (updown < updown_collect_)
            ret = tilt_collect_ ;
        else if (updown > updown_stow_)
            ret = tilt_stow_ ;
        else
            ret = tilt_stow_ + (tilt_collect_ - tilt_stow_) * (updown_stow_ - updown) / (updown_stow_ - updown_collect_) ;

        return ret ;
    }

    protected void gotoPosition(String updown, String tilt) throws Exception {
        gotoPosition(subsystem_.getUpDown().getSettingsValue(updown).getDouble(), subsystem_.getTilt().getSettingsValue(tilt).getDouble()) ;
    }

    protected void gotoPosition(double updown, double tilt) throws Exception {
        String type = "NONE" ;

        updown_target_ = updown ;
        tilt_target_ = tilt ;

        MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;

        double updownpos = subsystem_.getUpDown().getPosition() ;
        double tiltpos = subsystem_.getTilt().getPosition() ;
        double tiltstart = calcTiltFromUpdown(updownpos) ;

        if (tiltpos > tilt) {
            tilt_track_ = new MCTrackPosAction(subsystem_.getTilt(), "pids:position-track-down", tiltstart, 2.0, 2.0, true) ;
            type = "down" ;
        }
        else {
            tilt_track_ = new MCTrackPosAction(subsystem_.getTilt(), "pids:position-track-up", tiltstart, 2.0, 2.0, true) ;
            type = "up" ;
        }  

        if (Math.abs(tiltpos - tiltstart) > kTiltTolerance) {
            //
            // The tilt is off of the valid path, so move the tilt to the valid position
            // for the current updown location
            //
            tilt_action_.setTarget(tiltstart) ;
            subsystem_.getTilt().setAction(tilt_action_, true) ;
            
            state_ = State.AlignTilt ;

            logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
            logger.add("gotoPosition/start") ;
            logger.add("state", state_.toString()) ;
            logger.add("direction", type) ;
            logger.add("updown", updownpos) ;
            logger.add("tiltpos", tiltpos) ;
            logger.add("tiltstart", tiltstart) ;
            logger.add("tilt-target", tilt_target_) ;
            logger.add("updown-target", updown_target_) ;
            logger.endMessage();
        }
        else {
            //
            // Go directly to the target position as the tilt is on the valid path
            //

            //
            // Find the tilt position along the value transition path and set the
            // tile target to that path
            //
            tilt_track_.setTarget(tiltstart) ;

            //
            // Set the updown target
            //
            updown_action_.setTarget(updown_target_);

            //
            // Now move the intake along the prescribed path
            //
            subsystem_.getTilt().setAction(tilt_track_, true) ;
            subsystem_.getUpDown().setAction(updown_action_, true) ;

            state_ = State.MoveBoth ;

            logger.startMessage(MessageType.Debug, subsystem_.getLoggerID()) ;
            logger.add("gotoPosition/start") ;
            logger.add("state", state_.toString()) ;
            logger.add("direction", type) ;             
            logger.add("updown", updownpos) ;
            logger.add("tiltpos", tiltpos) ;
            logger.add("tiltstart", tiltstart) ;
            logger.add("tilt-target", tilt_target_) ;
            logger.add("updown-target", updown_target_) ;            
            logger.endMessage();            
        }
    }

    protected void runGoto() throws BadMotorRequestException, MotorRequestFailedException {
        State prev = state_ ;

        switch(state_) {
            case Idle:
                break ;

            case AlignTilt:
                if (tilt_action_.isDone()) {
                    double tilttar = calcTiltFromUpdown(subsystem_.getUpDown().getPosition()) ;

                    tilt_track_.setTarget(tilttar) ;   
                    subsystem_.getTilt().setAction(tilt_track_, true) ;

                    updown_action_.setTarget(updown_target_) ;
                    subsystem_.getUpDown().setAction(updown_action_, true) ;
                    state_ = State.MoveBoth ;
                }
                break ;

            case MoveBoth:
                if (updown_action_.isDone()) {
                    //
                    // Move the tilt to its final state
                    //
                    tilt_action_.setTarget(tilt_target_) ;                    
                    subsystem_.getTilt().setAction(tilt_action_, true) ;

                    if (Math.abs(subsystem_.getTilt().getPosition() - tilt_target_) < 5.0) {
                        state_ = State.Done ;
                    }
                    else {
                        state_ = State.FinishTilt ;
                    }
                }
                else {
                    double tilt = calcTiltFromUpdown(subsystem_.getUpDown().getPosition()) ;
                    tilt_track_.setTarget(tilt);
                }
                break ;

            case FinishTilt:
                if (tilt_action_.isDone())
                    state_ = State.Done ;
                break ;

            case Done:
                break ;
        }
        if (prev != state_) {
            MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("IntakeGotoBaseAction : " + prev.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }    
    }

    protected void cancelGoto() {
        updown_action_.cancel() ;
        tilt_action_.cancel() ;
        tilt_track_.cancel() ;
        state_ = State.Done ;
    }

    protected boolean isAtTarget() {
        return state_ == State.Done ;
    }
}
