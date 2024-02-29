package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public abstract class CollectBaseAltAction extends Action {

    private IntakeShooterSubsystem sub_;

    private MCMotionMagicAction tilt_stow_note_;
    private MCMotionMagicAction tilt_stow_no_note_ ;
    private MCMotionMagicAction tilt_stow_ ;
    private MCMotionMagicAction updown_stow_;

    public CollectBaseAltAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        tilt_stow_note_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position-collect", "targets:stow-note", 1, 1);
        tilt_stow_no_note_ = new MCMotionMagicAction(getSubsystem().getTilt(), "pids:position-collect", "targets:stow-no-note", 1, 1);
        updown_stow_ = new MCMotionMagicAction(getSubsystem().getUpDown(), "pids:position-collect", "targets:stow", 1, 1);
    }

    protected IntakeShooterSubsystem getSubsystem() {
        return sub_;
    }

    protected void startStow() {
        getSubsystem().getUpDown().setAction(updown_stow_, true);
        if (sub_.isHoldingNote())
            tilt_stow_ = tilt_stow_note_ ;
        else
            tilt_stow_ = tilt_stow_no_note_ ;

        getSubsystem().getTilt().setAction(tilt_stow_, true);
    }

    protected void runStow() {
        if (tilt_stow_.isDone() && updown_stow_.isDone()) {
            setDone() ;
        }
    }
}
