package frc.robot.subsystems.intake_shooter;

import java.lang.annotation.Target;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.utils.PieceWiseLinear;

import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;

public class IntakeShootAction extends Action {
    private final static double kVelocityStart = 45.0 ;
    private final static double kUpDownStart = 123.5 ;
    private final static double kTiltStart = -40.0 ;

    private IntakeShooterSubsystem sub_ ;
    private TargetTrackerSubsystem tracker_ ;
    private PieceWiseLinear updown_pwl_ ;
    private PieceWiseLinear tilt_pwl_ ;
    private PieceWiseLinear velocity_pwl_ ;

    private double current_updown_ ;
    private double current_tilt_ ;
    private double current_velocity_ ;

    private MCVelocityAction shooter1_ ;
    private MCVelocityAction shooter2_ ;
    private MCTrackPosAction updown_ ;
    private MCTrackPosAction tilt_ ;
    private MotorEncoderPowerAction feeder_on_ ;

    private boolean db_ready_ ;
    private boolean target_tracker_ready_ ;
    private boolean drive_team_ready_ ;
    private boolean shooting_ ;

    public IntakeShootAction(IntakeShooterSubsystem intake, TargetTrackerSubsystem tracker) throws Exception {
        super(intake.getRobot().getMessageLogger());

        sub_ = intake ;
        tracker_ = tracker ;

        double velthresh = sub_.getSettingsValue("actions:auto-shoot:shooter-velocity-threshold").getDouble() ;

        updown_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:updown") ;
        tilt_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:tilt") ;
        velocity_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:velocity") ;

        shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", kVelocityStart, velthresh, false) ;
        shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", kVelocityStart, velthresh, false) ;


        double updown_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-pos-threshold").getDouble() ;
        double updown_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-velocity-threshold").getDouble() ;
        updown_ = new MCTrackPosAction(sub_.getUpDown(), "pids:position", kUpDownStart, updown_pos_threshold, updown_vel_threshold) ;

        double tilt_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-pos-threshold").getDouble() ;
        double tilt_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-velocity-threshold").getDouble() ;        
        tilt_ = new MCTrackPosAction(sub_.getTilt(), "pids:position", kTiltStart, tilt_pos_threshold, tilt_vel_threshold) ;

        double feedpower = sub_.getSettingsValue("actions:auto-shoot:feeder-power").getDouble() ;
        double feedtime = sub_.getSettingsValue("actions:auto-shoot:feeder-time").getDouble() ;
        feeder_on_ = new MotorEncoderPowerAction(sub_.getFeeder(), feedpower, feedtime) ;
    }

    public void setDBReady(boolean ready) {
        db_ready_ = ready ;
    }

    public void setTargetTrackerReady(boolean ready) {
        target_tracker_ready_ = ready ;
    }

    public void setDriveTeamReady(boolean ready) {
        drive_team_ready_ = ready ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        db_ready_ = false ;
        shooting_ = false ;

        sub_.getUpDown().setAction(updown_, true);
        sub_.getTilt().setAction(tilt_, true);
        sub_.getShooter1().setAction(shooter1_, true);
        sub_.getShooter2().setAction(shooter2_, true);
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (shooting_) {
            if (isDone()) {
                sub_.getFeeder().setPower(0.0);
                sub_.getShooter1().setPower(0.0);
                sub_.getShooter2().setPower(0.0);
                setDone() ;
            }
        }
        else {
            double dist = tracker_.getDistance() ;
            current_updown_ = updown_pwl_.getValue(dist);
            current_tilt_ = tilt_pwl_.getValue(dist);
            current_velocity_ = velocity_pwl_.getValue(dist);

            updown_.setTarget(current_updown_);
            tilt_.setTarget(current_tilt_);
            shooter1_.setTarget(current_velocity_);
            shooter2_.setTarget(current_velocity_);

            if (updown_.isAtTarget() && tilt_.isAtTarget() && shooter1_.isAtVelocity() && shooter2_.isAtVelocity() && db_ready_ && target_tracker_ready_ && drive_team_ready_) {
                shooting_ = true ;
                sub_.getFeeder().setAction(feeder_on_, true);
            } 
        }
    }

    @Override
    public void cancel() {
        super.cancel();

        sub_.getFeeder().setPower(0.0);
        sub_.getShooter1().setPower(0.0);
        sub_.getShooter2().setPower(0.0);        
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeShootAction";
    }
}
