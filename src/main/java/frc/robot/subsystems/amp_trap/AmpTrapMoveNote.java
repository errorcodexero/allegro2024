package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class AmpTrapMoveNote extends Action {
    private AmpTrapSubsystem sub_ ;
    private double dist_ ;
    private double target_ ;
    private double power_ ;
    private MotorEncoderPowerAction action_ ;

    public AmpTrapMoveNote(AmpTrapSubsystem sub, String dist) throws BadParameterTypeException, MissingParameterException {
        this(sub, sub.getSettingsValue(dist).getDouble()) ;
    }

    public AmpTrapMoveNote(AmpTrapSubsystem sub, double dist) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger()) ;
        sub_ = sub ;
        dist_ = dist ;
        power_ = sub_.getManipulator().getSettingsValue("actions:move-note:power").getDouble();
        if (dist_ < 0) {
            power_ = -power_ ;
        }
        action_ = new MotorEncoderPowerAction(sub_.getManipulator(), power_) ;
    }

    @Override
    public void start() throws Exception {
        target_ = sub_.getManipulator().getPosition() + dist_ ;
        sub_.getManipulator().setAction(action_, true) ;

        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug) ;
        logger.add("start", sub_.getManipulator().getPosition()) ;
        logger.add("dist", dist_) ;
        logger.add("target", target_) ;
        logger.endMessage();
        
    }

    @Override
    public void run() throws Exception {
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug) ;
        logger.add("current", sub_.getManipulator().getPosition()) ;
        logger.add("target", target_) ;
        logger.endMessage();

        if (dist_ > 0.0 && sub_.getManipulator().getPosition() >= target_) {
            sub_.getManipulator().setPower(0.0);
            setDone() ;
        }
        else if (dist_ < 0.0 && sub_.getManipulator().getPosition() <= target_) {
            sub_.getManipulator().setPower(0.0);
            setDone() ;            
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapMoveNote " + dist_ ;
    }
}
