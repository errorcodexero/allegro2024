package frc.robot.subsystems.intakeshooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class PrepareTransferNoteAction extends Action {
    private IntakeShooterSubsystem sub_ ;
    private MCMotionMagicAction tilt_action_ ;
    private MCMotionMagicAction updown_action_ ;

    public PrepareTransferNoteAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        tilt_action_ = new MCMotionMagicAction(sub_.tilt(), "pids:position", "targets:transfer", 1, 1);
        updown_action_ = new MCMotionMagicAction(sub_.updown(), "pids:position", "targets:transfer", 1, 1) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.updown().setAction(updown_action_);
        sub_.tilt().setAction(tilt_action_) ;
    }

    @Override
    public void run() throws Exception {
        if (updown_action_.isDone() && tilt_action_.isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TransferNoteAction" ;
    }
}
