package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class PrepClimbAction extends Action{
    private AmpTrapSubsystem sub_;
    private MCMotionMagicAction climb_;
    private MCMotionMagicAction stow_elevator_;
    private MotorEncoderPowerAction stow_roller_;
    private MCMotionMagicAction stow_pivot_;
    //private MCMotionMagicAction stow_wrist_;
    public PrepClimbAction(AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        climb_ = new MCMotionMagicAction(sub.getClimber(), "pids:position", "targets:climbprep", 0.5, 0.5);
        stow_elevator_ = new MCMotionMagicAction(sub_.getElevator(), "pids:position" , "targets:stow" , 0.5 , 1);
        stow_roller_ = new MotorEncoderPowerAction(sub_.getManipulator(), 0);
        stow_pivot_ = new MCMotionMagicAction(sub_.getArm(), "pids:position" , "targets:stow" , 0.5 , 1);
        //stow_wrist_ = new MCMotionMagicAction(sub_.getWrist(), "pids:position" , "targets:stow" , 0.5 , 1);
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getElevator().setAction(stow_elevator_, true);
        sub_.getManipulator().setAction(stow_roller_, true);
        sub_.getArm().setAction(stow_pivot_, true);
        //sub_.getWrist().setAction(stow_wrist_, true);
        sub_.getClimber().setAction(climb_, true);
    }

    @Override
    public void run() throws Exception{
        super.run();
        if(climb_.isDone() && stow_elevator_.isDone() && stow_pivot_.isDone()){
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "PrepClimbAction";
    }
}
