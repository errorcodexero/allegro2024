package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.RioHoldAction;

public class StartCollectAction extends Action{
    private IntakeShooterSubsystem sub_;
    
    public StartCollectAction(IntakeShooterSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
   
    }
   @Override
   public void start() throws Exception {
       super.start();
    //sub_.getUpDown().setAction();
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
