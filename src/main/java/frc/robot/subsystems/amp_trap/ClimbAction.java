package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
//import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;


//The string type in the constructor specifies wether it is climbprep, with string "prep", climb, with string "", or climbdown with string "down"
public class ClimbAction extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb_;
    public ClimbAction(AmpTrapSubsystem sub, String type) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb_ = new MCMotionMagicAction(sub.getClimber(), "pids:position", "targets:climb" + type, 0.5, 0.5);
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getClimber().setAction(climb_, true);
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
