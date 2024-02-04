package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;


public class PrepNoteTransferAction extends Action{
    private IntakeShooterSubsystem sub_;
    private MotorEncoderPowerAction feeder_;
    private MCMotionMagicAction updown_;

    public PrepNoteTransferAction(IntakeShooterSubsystem sub) throws Exception{
        super(sub.getRobot().getMessageLogger());

        sub = sub_;
        //detects if updown is in collecting state so it can prepare the feeder wheels
        //if(updown_ = new MCMotionMagicAction(sub.getUpDown(), "pids:position", "targets:collect", 0.5, 0.5)) {
        //    feeder_ = new MotorEncoderPowerAction(sub.getFeeder(), 1);
        //}
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getFeeder().setAction(feeder_, true);
        sub_.getUpDown().setAction(updown_, true);
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        //if (feeder_.isDone() && UpDown_.isDone()) {
        //    setDone();
        //}
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "PrepNoteTransferAction";
    }   
}