package frc.robot.subsystems.amp_trap;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class PrepAmpAction extends Action{
    // Moves elevator
    private MCMotionMagicAction elevate_;

    // Moves arm
    private MCMotionMagicAction tiltArm_;

    // Moves wrist
    private MCMotionMagicAction tiltWrist_;

    private AmpTrapSubsystem sub_;

    public PrepAmpAction (AmpTrapSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        elevate_ = new MCMotionMagicAction(sub.getElevator(), "pids:position", "targets:ampprep", 0.5, 0.5);
        tiltArm_ = new MCMotionMagicAction(sub.getArm(), "pids:position", "targets:ampprep", 1, 1);
        tiltWrist_ = new MCMotionMagicAction(sub.getWrist(), "pids:position", "targets:ampprep", 2, 2);
    }

    @Override
    public void start() throws Exception{
        super.start();
        sub_.getElevator().setAction(elevate_, true);
        sub_.getArm().setAction(tiltArm_, true);
        sub_.getWrist().setAction(tiltWrist_, true);
    }

    @Override
    public void run() throws Exception{
        super.run();
        if(elevate_.isDone() && tiltArm_.isDone() && tiltWrist_.isDone()){
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "PrepAmpAction";
    }
}
