package frc.robot.subsystems.intake_shooter;


import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;

public class IntakeShooterStowAction  extends Action{
    

    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction stow_updown_action_;



    public IntakeShooterStowAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        stow_updown_action_ = new MCMotionMagicAction(sub.getUpDown(), "stow", "stow", 0, 0); 
    }  

    @Override
    public void start() throws Exception{
        super.start();

        sub_.getUpDown().setAction(stow_updown_action_ , true);

       
    }

    @Override
    public void run() throws Exception {
        super.run() ;

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeShooterStowAction";
    }


}