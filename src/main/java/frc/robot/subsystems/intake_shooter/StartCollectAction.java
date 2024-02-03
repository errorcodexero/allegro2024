package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.actions.Action;

public class StartCollectAction extends Action {
    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction collect_updown_;
    private MCMotionMagicAction collect_tilt_;


    public StartCollectAction(IntakeShooterSubsystem sub)throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        collect_updown_ = new MCMotionMagicAction(sub.getUpDown(), "pid:position", "targets:collect", 0.5, 0.5);
        collect_tilt_ = new MCMotionMagicAction(sub.getTilt(), "pid:position", "targets:collect", 0.5, 0.5);

    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getUpDown().setAction(collect_updown_, true);
        sub_.getTilt().setAction(collect_tilt_, true);
    }

    @Override
    public void run() throws Exception {
        super.run();
        boolean step1= false;
        boolean step2= false;
        boolean step3= false;
        
        if(collect_tilt_.isDone() && collect_updown_.isDone() && !step1){
            
        }
    }
   
    @Override
    public String toString(int indent) {
        return prefix(indent) + "StartCollectAction";
    }

}
