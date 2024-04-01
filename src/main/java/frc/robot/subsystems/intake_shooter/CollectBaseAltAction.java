package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public abstract class CollectBaseAltAction extends Action {

    private IntakeShooterSubsystem sub_;

    private MCMotionMagicAction tilt_action_shoot_;
    private MCMotionMagicAction updown_action_shoot_ ;
    private MCMotionMagicAction tilt_action_stow_;
    private MCMotionMagicAction updown_action_stow_ ;    

    private MCMotionMagicAction tilt_act_ ;
    private MCMotionMagicAction updown_act_ ;

    public CollectBaseAltAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception {
        super(sub.getRobot());

        sub_ = sub;

        if (Double.isNaN(updown) || Double.isNaN(tilt)) {
            tilt = sub_.getTilt().getSettingsValue("targets:shoot").getDouble() ;
            updown = sub_.getUpDown().getSettingsValue("targets:stow").getDouble() ;
        }
        tilt_action_shoot_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", tilt, 1, 1) ;
        updown_action_shoot_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", updown, 1, 1) ;

        tilt = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;
        updown = sub_.getUpDown().getSettingsValue("targets:stow").getDouble() ;

        tilt_action_stow_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", tilt, 5, 5) ;
        updown_action_stow_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", updown, 5, 5) ;
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return sub_;
    }

    protected void startStow() {
        if (sub_.isHoldingNote()) {
            tilt_act_ = tilt_action_shoot_ ;
            updown_act_ = updown_action_shoot_ ;
        }
        else {
            tilt_act_ = tilt_action_stow_ ;
            updown_act_ = updown_action_stow_ ;
        }
        getSubsystem().getTilt().setAction(tilt_act_, true);
        getSubsystem().getUpDown().setAction(updown_act_, true) ;
    }

    protected void runStow() {
        if (tilt_act_.isDone() && updown_act_.isDone()) {
            setDone() ;
        }
    }
}
