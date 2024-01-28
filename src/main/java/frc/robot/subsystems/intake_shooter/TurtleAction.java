package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class TurtleAction extends Action {
    private IntakeShooterSubsystem sub_ ;

    private MCMotionMagicAction tilt_action_ ;
    private MCMotionMagicAction updown_action_ ;

    public TurtleAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        updown_action_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position", "target:turtle", 1, 1) ;
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", "targets:turtle", 1, 1);
    }   

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getUpDown().setAction(updown_action_);
        sub_.getTilt().setAction(tilt_action_) ;
        sub_.getShooter1().setPower(0.0);
        sub_.getShooter2().setPower(0.0);
        sub_.getFeeder().setPower(0.0);
    }

    @Override
    public void run() throws Exception {
        if(updown_action_.isDone() && tilt_action_.isDone())
            setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TurtleAction" ;
    }    
}
