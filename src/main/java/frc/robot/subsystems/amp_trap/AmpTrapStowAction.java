package frc.robot.subsystems.amp_trap;

// Stows the elevator, climber, arm, wrist, and manipulator to their respective indicated positions.


import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;


public class AmpTrapStowAction extends Action{
    
    private AmpTrapSubsystem sub_ ;
    private MCMotionMagicAction stow_elevator_;
    private MotorEncoderPowerAction stow_roller_;
    private MCMotionMagicAction stow_pivot_;
    //private MCMotionMagicAction stow_wrist_;
    private MCMotionMagicAction stow_climber_;
    

    public AmpTrapStowAction(AmpTrapSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        // Initialization
        sub_ = sub;
        stow_elevator_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position" , "targets:stow" , 0.5 , 1);
        stow_roller_ = new MotorEncoderPowerAction(sub_.getManipulator(), 0);
        stow_pivot_ = new MCMotionMagicAction(sub_.getArm(), "pids:position" , "targets:stow" , 0.5 , 1);
        //stow_wrist_ = new MCMotionMagicAction(sub_.getWrist(), "pids:position" , "targets:stow" , 0.5 , 1);
        stow_climber_ = new MCMotionMagicAction(sub_.getClimber(), "pids:position", "targets:stow", 0.5, 1);
    }

    @Override
    public void start() throws Exception{
        super.start();

        sub_.getElevator().setAction(stow_elevator_, true);
        sub_.getManipulator().setAction(stow_roller_, true);
        sub_.getArm().setAction(stow_pivot_, true);
        //sub_.getWrist().setAction(stow_wrist_, true);
        sub_.getClimber().setAction(stow_climber_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (stow_elevator_.isDone() && stow_pivot_.isDone() && stow_climber_.isDone()) {
            setDone();
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AmpTrapStowAction";
    }
}