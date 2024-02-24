package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.Subsystem.DisplayType;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.oi.OILed.LEDState;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.oi.AllegroOIPanel;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class IntakeAutoShootAction extends Action {
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

    private boolean drive_team_ready_ ;
    private boolean initial_drive_team_ready_ ;
    private boolean require_april_tag_ ;
    private boolean shooting_ ;

    private boolean verbose_ ;

    private XeroTimer wait_timer_ ;
    private boolean waiting_ ;

    private boolean db_ready_ ;

    private double aim_threshold_ ;
    private double tilt_stow_value_ ;

    // The start time of the shoot action
    private double start_time_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // Data for each loop of the plot
    private Double data_[] ;

    // The columns to plot
    private static String [] columns_ = { 
        "time", 
        "shooter-target (deg/s)", 
        "shooter1 (deg/s)",
        "shooter2 (deg/s)",
        "updown-target (deg)",
        "updown (deg)",
        "tilt-target (deg)",
        "tilt (deg)",
        "dbready (bool)",
        "april-tag (bool)",
        "oiready (bool)",
        "distance (m)"
    } ;

    public IntakeAutoShootAction(IntakeShooterSubsystem intake, TargetTrackerSubsystem tracker, boolean initialDriveTeamReady, boolean requireAprilTag) throws Exception {
        super(intake.getRobot().getMessageLogger());

        sub_ = intake ;
        tracker_ = tracker ;
        initial_drive_team_ready_ = initialDriveTeamReady ;
        require_april_tag_ = requireAprilTag ;
        verbose_ = false ;
        waiting_ = false ;

        wait_timer_  = new XeroTimer(intake.getRobot(), "wait-timer", 2.0) ;

        ISettingsSupplier settings = sub_.getRobot().getSettingsSupplier() ;
        if (settings.isDefined("system:verbose:auto-shoot")) {
            verbose_ = settings.get("system:verbose:auto-shoot").getBoolean() ;
        } 

        tilt_stow_value_ = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;
        aim_threshold_ = sub_.getSettingsValue("actions:auto-shoot:aim-threshold").getDouble() ;

        double velthresh = sub_.getSettingsValue("actions:auto-shoot:shooter-velocity-threshold").getDouble() ;
        double updown_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-pos-threshold").getDouble() ;
        double updown_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-velocity-threshold").getDouble() ;
        double tilt_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-pos-threshold").getDouble() ;
        double tilt_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-velocity-threshold").getDouble() ;

        if (RobotBase.isSimulation()) {
            velthresh = 10.0 ;  
            updown_pos_threshold = 10.0 ;
            updown_vel_threshold = 10.0 ;
            tilt_pos_threshold = 10.0 ;
            tilt_vel_threshold = 10.0 ;
        }

        updown_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:updown") ;
        tilt_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:tilt") ;
        velocity_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls:shooter") ;

        shooter1_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", kVelocityStart, velthresh, false) ;
        shooter2_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", kVelocityStart, velthresh, false) ;
        updown_ = new MCTrackPosAction(sub_.getUpDown(), "pids:position", kUpDownStart, updown_pos_threshold, updown_vel_threshold, false) ;      
        tilt_ = new MCTrackPosAction(sub_.getTilt(), "pids:position", kTiltStart, tilt_pos_threshold, tilt_vel_threshold,true) ;

        double feedpower = sub_.getSettingsValue("actions:auto-shoot:feeder-power").getDouble() ;
        double feedtime = sub_.getSettingsValue("actions:auto-shoot:feeder-time").getDouble() ;
        feeder_on_ = new MotorEncoderPowerAction(sub_.getFeeder(), feedpower, feedtime) ;

        if (verbose_) {
            plot_id_ = sub_.initPlot("AutoShoot") ;
            data_ = new Double[columns_.length] ;
        } else {
            plot_id_ = -1 ;
        }
    }

    public boolean isShooting() {
        return shooting_ ;
    }

    public void setDriveTeamReady(boolean ready) {
        drive_team_ready_ = ready ;
    }

    public void setDBReady(boolean ready) {
        db_ready_ = ready ;
    }

    @Override
    public void start() throws Exception {
        super.start();

        shooting_ = false ;
        drive_team_ready_ = initial_drive_team_ready_ ;
        db_ready_ = false ;

        sub_.getUpDown().setAction(updown_, true);
        sub_.getTilt().setAction(tilt_, true);
        sub_.getShooter1().setAction(shooter1_, true);
        sub_.getShooter2().setAction(shooter2_, true);

        if (plot_id_ != -1) {
            start_time_ = sub_.getRobot().getTime() ;
            sub_.startPlot(plot_id_, columns_) ;
        }
    }

    private boolean aprilTagTest() {
        if (require_april_tag_) {
            return tracker_.seesTarget() ;
        }
        return true ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (waiting_ && !shooting_) {
            if (wait_timer_.isExpired()) {
                sub_.getFeeder().setAction(feeder_on_, true);
                shooting_ = true ;
            }
        }
        else if (shooting_) {
            if (feeder_on_.isDone()) {
                sub_.setHoldingNote(false);
                sub_.getFeeder().setPower(0.0);
                sub_.getShooter1().setPower(0.0);
                sub_.getShooter2().setPower(0.0);
                if (plot_id_ != -1) {
                    sub_.endPlot(plot_id_);
                }
                setDone() ;
            }
        }
        else {
            double dist = tracker_.getDistance() ;

            if (dist > aim_threshold_) {
                current_updown_ = updown_pwl_.getValue(dist) ;
                current_tilt_ = tilt_stow_value_ ;
                current_velocity_ = 50 ;
            }
            else {
                current_updown_ = updown_pwl_.getValue(dist);
                current_tilt_ = tilt_pwl_.getValue(dist);
                current_velocity_ = velocity_pwl_.getValue(dist);
            }

            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
            logger.add("shooting") ;
            logger.add("distance", dist) ;
            logger.add("updown", current_updown_).add("tilt", current_tilt_).add("velocity", current_velocity_) ;
            logger.endMessage();

            AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;
            AllegroOIPanel oi = robot.getOI().getPanel() ;

            updown_.setTarget(current_updown_);
            tilt_.setTarget(current_tilt_);
            shooter1_.setTarget(current_velocity_);
            shooter2_.setTarget(current_velocity_);

            oi.setTiltLED(tilt_.isAtTarget() ? LEDState.ON : LEDState.OFF);
            oi.setAprilTagLED(aprilTagTest() ? LEDState.ON : LEDState.OFF);
            oi.setDBLED(db_ready_ ? LEDState.ON : LEDState.OFF);
            oi.setVelocityLED((shooter1_.isAtVelocity() && shooter2_.isAtVelocity()) ? LEDState.ON : LEDState.OFF);

            if (verbose_) {
                SmartDashboard.putBoolean("Tilt", tilt_.isAtTarget()) ;
                SmartDashboard.putBoolean("UpDown", updown_.isAtTarget()) ;
                SmartDashboard.putBoolean("Shooter1", shooter1_.isAtVelocity()) ;
                SmartDashboard.putBoolean("Shooter2", shooter1_.isAtVelocity()) ;
                SmartDashboard.putBoolean("DB", db_ready_) ;
                SmartDashboard.putBoolean("AprilTag ", tracker_.seesTarget()) ;
                SmartDashboard.putBoolean("Button", drive_team_ready_) ;
            }

            if (plot_id_ != -1) {
                data_[0] = sub_.getRobot().getTime() - start_time_ ;
                data_[1] = current_velocity_ ;
                data_[2] = sub_.getShooter1().getVelocity() ;
                data_[3] = sub_.getShooter2().getVelocity() ;
                data_[4] = current_updown_ ;
                data_[5] = sub_.getUpDown().getPosition() ;
                data_[6] = current_tilt_ ;
                data_[7] = sub_.getTilt().getPosition() ;
                data_[8] = db_ready_ ? 1.0 : 0.0 ;
                data_[9] = aprilTagTest() ? 0.5 : 0.0 ;
                data_[10] = drive_team_ready_ ? 1.5 : 0.0 ;
                data_[11] = robot.getTargetTracker().getDistance() ;
                sub_.addPlotData(plot_id_, data_);
            }

            if (updown_.isAtTarget() && tilt_.isAtTarget() && shooter1_.isAtVelocity() && shooter2_.isAtVelocity() && db_ready_ && aprilTagTest() && drive_team_ready_) {
                shooting_ = true ;
                sub_.getFeeder().setAction(feeder_on_, true);
                // waiting_ = true ;
                // wait_timer_.start() ;
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
