package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;


public class IntakeGotoNamedPositionAction extends Action{
    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction updown_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private double updown_target_ ;
    private double tilt_target_ ;

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;

        updown_action_ = new MCMotionMagicAction(sub_.getUpDown(), "pids:position" , updown, 0.5 , 1);
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position" , tilt, 0.5 , 1);

        updown_target_ = updown;
        tilt_target_ = tilt;
    }

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, String updown, String tilt) throws Exception{
        this(sub, sub.getSettingsValue(updown).getDouble(), sub.getSettingsValue(tilt).getDouble());
    }    

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getUpDown().setAction(updown_action_, true);
        sub_.getTilt().setAction(tilt_action_, true);
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        if (updown_action_.isDone() && tilt_action_.isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "IntakeGotoNamedPositionAction updown " + updown_target_ + ", tilt " + tilt_target_ ;
    }   
}