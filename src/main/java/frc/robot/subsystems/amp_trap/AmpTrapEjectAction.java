package frc.robot.subsystems.amp_trap;

// Runs the Eject Action for 1.5 seconds which runs the manipulator motor 
// after the elevator, arm, and wrist move to their respective stowed positions.
import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;

public class AmpTrapEjectAction extends Action{

    // Creating FMS states
    private enum State {
        Idle, 
        ReadyToEject,
        Eject,
        Stow
    }
    
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction eject_elevator_;
    private MCMotionMagicAction eject_arm_;
    //private MCMotionMagicAction eject_wrist_;
    private MCVelocityAction eject_manipulator_;
    private XeroTimer timer_;
    private State state_;
    private AmpTrapStowAction stow_amp_trap_;
    
    
    public AmpTrapEjectAction(AmpTrapSubsystem sub) throws Exception {

        super(sub.getRobot().getMessageLogger());

        state_ = State.Idle;

        sub_ = sub;
        eject_elevator_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_arm_ = new MCMotionMagicAction(sub_.getArm(), "pids:position" , "targets:stow" , 0.5 , 1);
        //eject_wrist_ = new MCMotionMagicAction(sub_.getWrist(), "pids:position" , "targets:stow" , 0.5 , 1);
        eject_manipulator_ = new MCVelocityAction(sub_.getManipulator(), "pids:position", "targets:eject", true);
        timer_ = new XeroTimer(sub_.getRobot(), "Eject", 1.5);
        stow_amp_trap_ = new AmpTrapStowAction(sub_);
    }

    @Override 
    public void start() throws Exception{
        super.start();

        state_ = State.ReadyToEject;
        sub_.getElevator().setAction(eject_elevator_, true);
        //sub_.getWrist().setAction(eject_wrist_, true);
        sub_.getArm().setAction(eject_arm_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        // Switching between different states
        switch(state_) {
            case Idle:
                break;
            case ReadyToEject:
                stateReadyToEject();
                break;
            case Eject:
                stateEject();
                break;
            case Stow:
                stateStow();
                break;
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapEjectAction";
    }


    // Creating and defining what happens during each state before it can switch
    private void stateReadyToEject(){
        if (eject_elevator_.isDone() && eject_arm_.isDone()){
            state_ = State.Eject;
            sub_.getManipulator().setAction(eject_manipulator_, true);
            timer_.start();
        }
    }

    private void stateEject(){
        if (timer_.isExpired()){
            sub_.getManipulator().cancelAction();
            sub_.setAction(stow_amp_trap_, true);
        }
    }

    private void stateStow(){
        if(stow_amp_trap_.isDone()){
            state_ = State.Idle;
            setDone();
        }
    }
}


