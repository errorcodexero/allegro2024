package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;

public class AmpTrapXferAction extends Action {
    public AmpTrapXferAction(AmpTrapSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
    }
    
    @Override
    public void start() throws Exception {
        super.start();
    }

    @Override
    public void run() throws Exception {
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapXferAction";
    }
}
