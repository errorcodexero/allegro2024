package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class PrepClimbAction extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb_;
    private AmpTrapStowAction stow_;
    public PrepClimbAction(AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb_ = new MCMotionMagicAction(sub.getClimber(), "pids:position", "targets:climbprep", 0.5, 0.5);
        stow_ = new AmpTrapStowAction(sub);
    }

    @Override
    public void start() throws Exception{
        super.start();
        stow_.start();
        sub_.getClimber().setAction(climb_, true);
    }

    @Override
    public void run() throws Exception{
        super.run();
        stow_.run();
        if(climb_.isDone() && stow_.isDone()){
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "PrepClimbAction";
    }
}
