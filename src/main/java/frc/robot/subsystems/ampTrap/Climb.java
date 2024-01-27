package frc.robot.subsystems.ampTrap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
//import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class Climb extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb;
    public Climb(AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb = new MCMotionMagicAction(sub.climber(), "climber", 20, 0.5, 1);
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ClimbAction";
    }
}
