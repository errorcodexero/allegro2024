package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class AmpTrapXferAction extends Action {
    private AmpTrapSubsystem sub_ ;
    private MotorEncoderPowerAction power_ ;

    public AmpTrapXferAction(AmpTrapSubsystem sub) throws Exception {
        super(sub.getRobot());

        sub_ = sub ;
        double p = sub.getSettingsValue("actions:xfer:power").getDouble() ;
        power_ = new MotorEncoderPowerAction(sub.getManipulator(), p) ;
    }
    
    @Override
    public void start() throws Exception {
        super.start();

        sub_.setAction(power_, true) ;
    }

    @Override
    public void run() throws Exception {
    }

    @Override
    public void cancel() {
        power_.cancel() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapXferAction";
    }
}
