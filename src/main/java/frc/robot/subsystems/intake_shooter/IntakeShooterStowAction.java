package frc.robot.subsystems.intake_shooter;


import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class IntakeShooterStowAction  extends Action{
    

    private IntakeShooterSubsystem sub_;
    private MCMotionMagicAction stow_updown_;
    private MCMotionMagicAction stow_tilt_;
    private MotorEncoderPowerAction stow_feeder_;
    private MotorEncoderPowerAction stow_shooter1_;
    private MotorEncoderPowerAction stow_shooter2_;



    public IntakeShooterStowAction(IntakeShooterSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        stow_updown_ = new MCMotionMagicAction(sub.getUpDown(), "stow", "stow", 0, 1); 
        stow_tilt_ = new MCMotionMagicAction(sub.getTilt(), "stow" , "stow" , 0 , 1);
        stow_feeder_ = new MotorEncoderPowerAction(sub.getFeeder(), 0);
        stow_shooter1_ = new MotorEncoderPowerAction(sub.getShooter1(), 0);
        stow_shooter2_ = new MotorEncoderPowerAction(sub.getShooter2(), 0);
    }  

    @Override
    public void start() throws Exception{
        super.start();

        sub_.getUpDown().setAction(stow_updown_ , true);
        sub_.getTilt().setAction(stow_tilt_ , true);
        sub_.getFeeder().setAction(stow_feeder_ , true);
        sub_.getShooter1().setAction(stow_shooter1_ , true);
        sub_.getShooter2().setAction(stow_shooter2_ , true);

       
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if(stow_updown_.isDone() && stow_tilt_.isDone()) {
            setDone();
        }

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "IntakeShooterStowAction";
    }


}