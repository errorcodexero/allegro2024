package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class AmpTrapShootAction extends Action {
    private AmpTrapSubsystem sub_ ;
    private MotorEncoderPowerAction manip_action_ ;
    private double start_pos_ ;
    private double distance_ ;

    public AmpTrapShootAction(AmpTrapSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        distance_ = sub_.getSettingsValue("actions:shoot:distance").getDouble() ;

        double v = sub_.getSettingsValue("actions:shoot:power").getDouble() ;
        manip_action_ = new MotorEncoderPowerAction(sub.getManipulator(), v) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getManipulator().setAction(manip_action_, true) ;
        start_pos_ = sub_.getManipulator().getPosition() ;
    }

    @Override
    public void run() throws Exception {
        if (sub_.getManipulator().getPosition() - start_pos_ > distance_) {
            sub_.getManipulator().setPower(0.0);
            setDone() ;
        }
    }

    @Override
    public void cancel() {
        super.cancel();
        sub_.getManipulator().setPower(0.0);
    }
    
    @Override
    public String toString(int indent) {
        return prefix(indent) + "AmpTrapShootAction" ;
    }
}
