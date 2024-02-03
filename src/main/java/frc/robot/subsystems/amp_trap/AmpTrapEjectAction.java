package frc.robot.subsystems.amp_trap;


import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class AmpTrapEjectAction extends Action{
    
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction eject_elevator_;
    private MCMotionMagicAction eject_arm_;
    private MCMotionMagicAction eject_wrist_;
    private MCVelocityAction eject_manipulator_;
    private MCVelocityAction stop_manipulator_;
    private XeroTimer timer_;
    
    
    public AmpTrapEjectAction(AmpTrapSubsystem sub) throws Exception {

        super(sub.getRobot().getMessageLogger());


        sub_ = sub;
        eject_elevator_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_arm_ = new MCMotionMagicAction(sub_.getArm(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_wrist_ = new MCMotionMagicAction(sub_.getWrist(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_manipulator_ = new MCVelocityAction(sub_.getManipulator(), "pids:position", "targets:eject");
        stop_manipulator_ = new MCVelocityAction(sub_.getManipulator(), "pids:position", "targets:stop");
        timer_ = new XeroTimer(sub_.getRobot(), "Eject", 1.5);
    }

    @Override 
    public void start() throws Exception{
        super.start();
        sub_.getManipulator().setAction(eject_manipulator_, true);
        timer_.start();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        boolean step1= false;
        if(timer_.isExpired() && !step1){
            sub_.getElevator().setAction(eject_elevator_, true);
            sub_.getWrist().setAction(eject_wrist_, true);
            sub_.getArm().setAction(eject_arm_, true);
            sub_.getManipulator().setAction(stop_manipulator_);
            step1 = true;
        }
        if (eject_elevator_.isDone() && eject_arm_.isDone() && eject_wrist_.isDone()) {
            setDone();
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapEjectAction";
    }
}


