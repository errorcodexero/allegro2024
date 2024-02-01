package frc.robot.subsystems.intake_shooter;

import javax.swing.text.Position;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.RioHoldAction;

public class StartCollectAction extends Action{
    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction hold_;
    public StartCollectAction(IntakeShooterSubsystem sub)throws Exception{
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        hold_ = new MCMotionMagicAction(sub.getUpDown(), "pid:position", "targets:collect",0.5, 0.5);
    }
   @Override
   public void start() throws Exception {
       super.start();
       sub_.getUpDown().setAction(hold_, true);
   }
   @Override
   public void run() throws Exception{
       super.run();
   }
   
    @Override
    public String toString(int indent) {
        return prefix(indent) + "StartCollectAction";
    }

}
