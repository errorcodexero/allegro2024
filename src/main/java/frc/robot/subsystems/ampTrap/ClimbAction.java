package frc.robot.subsystems.ampTrap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
//import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class ClimbAction extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb_;
    public ClimbAction(AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb_ = new MCMotionMagicAction(sub.getClimber(), "pids:position", "targets:climbprep", 0.5, 0.5);
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getClimber().setAction(climb_);
    }

    @Override
    public void run() throws Exception{
        super.run();
        if(climb_.isDone()){
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ClimbAction";
    }
}
