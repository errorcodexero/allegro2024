package frc.robot.subsystems.ampTrap;


import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;


public class AmpTrapStowAction extends Action{
    
    private AmpTrapSubsystem sub_ ;
    private MCMotionMagicAction stow_elevator_action_ ;
    private MotorEncoderPowerAction stow_roller_action_ ;
    private MCMotionMagicAction stow_pivot_action_ ;
    

    public AmpTrapStowAction(AmpTrapSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        // Initialization
        sub_ = sub;
        stow_elevator_action_ = new MCMotionMagicAction(sub_.getElevator(), "stow" , "stow" , 0 , 1);
        stow_roller_action_ = new MotorEncoderPowerAction(sub_.getManipulator(), 0);
        stow_pivot_action_ = new MCMotionMagicAction(sub_.getArm(), "stow" , "stow" , .5 , 1);
    }

    @Override
    public void start() throws Exception{
        super.start();

        sub_.getElevator().setAction(stow_elevator_action_, true);
        sub_.getManipulator().setAction(stow_roller_action_, true);
        sub_.getArm().setAction(stow_pivot_action_, true);
        setDone();
    }

    @Override
    public void run() throws Exception {
        super.run() ;

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapStowAction";
    }
}


    

