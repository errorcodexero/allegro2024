package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class AmpTrapEjectAction extends Action {
    private AmpTrapSubsystem sub_ ;
    private MotorEncoderPowerAction start_ ;
    private XeroTimer timer_ ;

    public AmpTrapEjectAction(AmpTrapSubsystem sub) {
        super(sub.getRobot().getMessageLogger()) ;

        sub_ = sub ;
        start_ = new MotorEncoderPowerAction(sub_.getManipulator(), -0.5) ;
        timer_ = new XeroTimer(sub_.getRobot(), "eject-timer", 1.0) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getManipulator().setAction(start_, true) ;
        timer_.start() ;
        sub_.setHoldingNote(false);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (timer_.isExpired()) {
            sub_.getManipulator().setPower(0.0) ;
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeEjectAction" ;
    }    
}
