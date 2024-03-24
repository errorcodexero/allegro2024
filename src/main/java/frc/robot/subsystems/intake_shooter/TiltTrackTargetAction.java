package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

public class TiltTrackTargetAction extends Action {
    private IntakeShooterSubsystem intake_;
    private double target_ ;
    private double start_time_ ;
    private double last_time_ ;
    private double last_pos_ ;
    private PIDCtrl ctrl_ ;
    private double ptol_ ;
    private double vtol_ ;
    private boolean is_at_target_ ;

    public TiltTrackTargetAction(IntakeShooterSubsystem intake, double target, double ptol, double vtol) throws MissingParameterException, BadParameterTypeException {
        super(intake.getRobot().getMessageLogger());
        intake_ = intake;
        target_ = target ;
        ptol_ = ptol ;
        vtol_ = vtol ;
        is_at_target_ = false ;

        String name = "subsystems:intake-tilt:pids:abs-position" ;
        ctrl_ = new PIDCtrl(intake_.getRobot().getSettingsSupplier(), name, false) ;
    }

    public void setTarget(double target) {
        target_ = target ;
    }

    public boolean isAtTarget() {
        return is_at_target_ ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        start_time_ = intake_.getRobot().getTime() ;
        last_time_ = start_time_ ;
        is_at_target_ = false ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double now = intake_.getRobot().getTime() ;
        double current = intake_.getAbsEncoderAngle() ;
        double output = ctrl_.getOutput(target_, current, now - last_time_) ;
        intake_.getTilt().setPower(output);

        double vel = (current - last_pos_) / (now - last_time_) ;
        if (Math.abs(current - target_) < ptol_ && Math.abs(vel) < vtol_)
            is_at_target_ = true ;

        last_time_ = now ;
        last_pos_ = current ;
    } 

    @Override
    public void cancel() {
        super.cancel();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TiltTrackTargetAction" ;
    }
}
