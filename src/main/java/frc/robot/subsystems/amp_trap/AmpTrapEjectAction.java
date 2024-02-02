package frc.robot.subsystems.amp_trap;


import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class AmpTrapEjectAction extends Action{
    
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction eject_elevator_;
    private MCMotionMagicAction eject_arm_;
    private MCMotionMagicAction eject_wrist_;
    private MCVelocityAction eject_manipulator_;
    
    
    public AmpTrapEjectAction(AmpTrapSubsystem sub) throws Exception {

        super(sub.getRobot().getMessageLogger());


        sub_ = sub;
        eject_elevator_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_arm_ = new MCMotionMagicAction(sub_.getArm(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_wrist_ = new MCMotionMagicAction(sub_.getWrist(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_manipulator_ = new MCVelocityAction(sub_.getManipulator(), "pids:position", "targets:eject");
    }

    @Override 
    public void start() throws Exception{
        super.start();

        sub_.getElevator().setAction(eject_elevator_, true);
        sub_.getArm().setAction(eject_elevator_, true);
        sub_.getWrist().setAction(eject_wrist_, true);
        sub_.getManipulator().setAction(eject_manipulator_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (eject_elevator_.isDone() && eject_arm_.isDone() && eject_wrist_.isDone() && eject_manipulator_.isDone()) {
            setDone();
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapEjectAction";
    }
}


