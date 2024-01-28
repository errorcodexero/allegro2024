package frc.robot.subsystems.ampTrap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
//import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class Climb extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb_;
    public Climb(AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb_ = new MCMotionMagicAction(sub.getClimber(), "climb", "climb", 0.5, 1);
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
