package frc.robot.subsystems.intake_shooter;

public class IntakeGotoNamedPositionAction extends IntakeGotoBaseAction {

    private double updown_target_ ;
    private double tilt_target_ ;

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, double updown, double tilt) throws Exception{
        super(sub) ;

        updown_target_ = updown ;
        tilt_target_ = tilt ;
    }

    public IntakeGotoNamedPositionAction(IntakeShooterSubsystem sub, String updown, String tilt) throws Exception{
        this(sub, sub.getSettingsValue(updown).getDouble(), sub.getSettingsValue(tilt).getDouble());
    }    

    @Override
    public void start() throws Exception {
        super.start();

        gotoPosition(updown_target_, tilt_target_) ;
    }

    @Override
    public void run() throws Exception{
        super.run() ;

        runGoto();
        if (isAtTarget()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent){
        return spaces(indent) + "IntakeGotoNamedPositionAction updown " + updown_target_ + ", tilt " + tilt_target_ ;
    }   
}