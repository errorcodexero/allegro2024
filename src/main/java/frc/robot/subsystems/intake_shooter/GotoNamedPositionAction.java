package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;


public class GotoNamedPositionAction extends Action{
    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction updown_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private String posname_ ;

    public GotoNamedPositionAction(IntakeShooterSubsystem sub, String posname) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub = sub_;
        posname_ = posname ;

        updown_action_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position" , "targets:" + posname , 0.5 , 1);
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position" , "targets:" + posname , 0.5 , 1);
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getUpDown().setAction(updown_action_, true);
        sub_.getTilt().setAction(tilt_action_, true);
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        if (updown_action_.isDone() && tilt_action_.isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "PrepNoteTransferAction " + posname_ ;
    }   
}