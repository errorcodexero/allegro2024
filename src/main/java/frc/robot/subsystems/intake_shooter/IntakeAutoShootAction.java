package frc.robot.subsystems.intake_shooter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.xero1425.base.actions.Action;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.oi.OILed.LEDState;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem.LedMode;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.oi.AllegroOIPanel;
import frc.robot.subsystems.target_tracker.TargetTrackerSubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class IntakeAutoShootAction extends Action {
    private final static int kNumberTiltSamples = 5 ;
    private final static double kAbsTiltTolerance = 3.0 ;

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
    private MCTrackPosAction tilt_mc_ ;
    private MotorEncoderPowerAction feeder_on_ ;
    private BooleanSupplier at_target_supplier_ ;

    private boolean drive_team_ready_ ;
    private boolean initial_drive_team_ready_ ;
    private boolean shooting_ ;
    private boolean waiting_ ;

    private boolean offset_set_ ;

    private XeroTimer wait_timer_ ;

    private boolean verbose_ ;
    private boolean db_ready_ ;
    private boolean abs_enc_ready_ ;

    private boolean swerve_stopped_ ;
    private boolean gyro_stopped_ ;

    private double aim_threshold_ ;
    private double max_distance_ ;
    private double tilt_stow_value_ ;

    private double rotational_velocity_threshold_ ;

    private List<Double> abs_enc_samples_ ;

    private SwerveTrackAngle rotate_ ;

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
        "tilt-mc (deg)",
        "tilt-abs (deg)",
        "dbready (bool)",
        "april-tag (bool)",
        "oiready (bool)",
        "distance (m)",
        "offset (deg)",
        "swerve (bool)",
        "gyro (bool)",
        "absenc (bool)",
        "updown (bool)",
        "tilt (bool)",
        "shooter (bool)"
    } ;

    public IntakeAutoShootAction(IntakeShooterSubsystem intake, TargetTrackerSubsystem tracker, boolean initialDriveTeamReady, SwerveTrackAngle rotate) throws Exception {
        super(intake.getRobot());

        sub_ = intake ;
        tracker_ = tracker ;
        initial_drive_team_ready_ = initialDriveTeamReady ;
        rotate_ = rotate ;
        verbose_ = false ;

        ISettingsSupplier settings = sub_.getRobot().getSettingsSupplier() ;
        if (settings.isDefined("system:verbose:auto-shoot")) {
            verbose_ = settings.get("system:verbose:auto-shoot").getBoolean() ;
        } 

        tilt_stow_value_ = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;
        aim_threshold_ = sub_.getSettingsValue("actions:auto-shoot:aim-threshold").getDouble() ;
        max_distance_ = sub_.getSettingsValue("actions:auto-shoot:max-distance").getDouble() ;        
        rotational_velocity_threshold_ = sub_.getSettingsValue("actions:auto-shoot:rotational-velocity-threshold").getDouble() ;

        double velthresh = sub_.getSettingsValue("actions:auto-shoot:shooter-velocity-threshold").getDouble() ;
        double updown_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-pos-threshold").getDouble() ;
        double updown_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-velocity-threshold").getDouble() ;
        double tilt_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-pos-threshold").getDouble() ;
        double tilt_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-velocity-threshold").getDouble() ;

        wait_timer_ = new XeroTimer(intake.getRobot(), "wait-timer", 0.02);

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
        tilt_mc_ = new MCTrackPosAction(sub_.getTilt(), "pids:position", kTiltStart, tilt_pos_threshold, tilt_vel_threshold ,true) ;

        double feedpower = sub_.getSettingsValue("actions:auto-shoot:feeder-power").getDouble() ;
        double feedtime = sub_.getSettingsValue("actions:auto-shoot:feeder-time").getDouble() ;
        feeder_on_ = new MotorEncoderPowerAction(sub_.getFeeder(), feedpower, feedtime) ;

        if (verbose_) {
            plot_id_ = sub_.initPlot("AutoShoot") ;
            data_ = new Double[columns_.length] ;
        } else {
            plot_id_ = -1 ;
        }

        abs_enc_samples_ = new ArrayList<Double>() ;
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

    private boolean isAbsEncReady() {
        boolean ret = true ;
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug) ;
        logger.add("absenc:") ;
        logger.add("tilt", current_tilt_) ;
        logger.add("samples ") ;
        for(Double d : abs_enc_samples_) {
            double diff = Math.abs(d - current_tilt_) ;
            logger.add(" " + Double.toString(diff)) ;
            if (diff > kAbsTiltTolerance) {
                ret = false ;
            }
        }

        logger.endMessage();        
        return abs_enc_samples_.size() == 5 && ret ;
    }
    
    @Override
    public void start() throws Exception {
        super.start();

        if (!sub_.isHoldingNote()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("IntakeAutoShootAction started with no note in intake").endMessage();
            setDone() ;
        }
        else {
            AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;
            robot.getLimelight().setLedMode(LedMode.ForceOn);

            abs_enc_samples_.clear() ;

            offset_set_ = false ;
            shooting_ = false ;
            waiting_ = false ;
            drive_team_ready_ = initial_drive_team_ready_ ;
            db_ready_ = false ;

            MessageLogger logger = getMessageLogger() ;
            logger.startMessage(MessageType.Debug) ;
            logger.add("dtready", drive_team_ready_) ;
            logger.endMessage();

            sub_.getUpDown().setAction(updown_, true);
            sub_.getTilt().setAction(tilt_mc_, true);
            at_target_supplier_ = () -> tilt_mc_.isAtTarget() ;
            sub_.getShooter1().setAction(shooter1_, true);
            sub_.getShooter2().setAction(shooter2_, true);

            if (plot_id_ != -1) {
                start_time_ = sub_.getRobot().getTime() ;
                sub_.startPlot(plot_id_, columns_) ;
            }
        }
    }

    private boolean aprilTagTest() {
        boolean ret = true ;
        if (rotate_ != null && !tracker_.seesTarget()) {
            ret = false ;
        }
        return ret ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        abs_enc_samples_.add(sub_.getAbsEncoderAngle()) ;
        if (abs_enc_samples_.size() == kNumberTiltSamples + 1) {
            abs_enc_samples_.remove(0) ;
        }

        abs_enc_ready_ = isAbsEncReady() ;

        AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;        

        if (drive_team_ready_ && !offset_set_) {
            offset_set_ = robot.getTargetTracker().setOffset();
        }

        if (rotate_ != null) {
            db_ready_ = rotate_.isAtTarget() ;
        }
        else {
            db_ready_ = true ;
        }

        if (waiting_ && !shooting_) {
            if (wait_timer_.isExpired()) {
                shooting_ = true ;
                if (rotate_ != null) {
                    rotate_.cancel() ;
                }
                sub_.getFeeder().setAction(feeder_on_, true);                
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
                robot.getTargetTracker().clearOffset();
                robot.getLimelight().setLedMode(LedMode.ForceOff) ;
                setDone() ;
            }
        }
        else {
            double dist = tracker_.getDistance() ;

            if (dist > aim_threshold_) {
                current_updown_ = updown_pwl_.getValue(0.0) ;
                current_tilt_ = tilt_stow_value_ ;
                current_velocity_ = 10 ;
            }
            else {
                current_updown_ = updown_pwl_.getValue(dist);
                if (!dbReadyToShoot()) {
                    current_tilt_ = tilt_pwl_.getValue(dist);
                    current_velocity_ = velocity_pwl_.getValue(dist);
                }
            }

            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug) ;
            logger.add("shooting") ;
            logger.add("distance", dist) ;
            logger.add("updown", current_updown_).add("tilt", current_tilt_).add("velocity", current_velocity_) ;
            logger.add("pose", robot.getSwerve().getPose().toString()) ;
            logger.add("offset", robot.getTargetTracker().getOffset());
            logger.add("angle", tracker_.getRotation()) ;
            logger.endMessage();

            AllegroOIPanel oi = robot.getOI().getPanel() ;
            updown_.setTarget(current_updown_);
            tilt_mc_.setTarget(current_tilt_);
            shooter1_.setTarget(current_velocity_);
            shooter2_.setTarget(current_velocity_);

            oi.setTiltLED(at_target_supplier_.getAsBoolean() ? LEDState.ON : LEDState.OFF);
            oi.setAprilTagLED(aprilTagTest() ? LEDState.ON : LEDState.OFF);
            if (dist > max_distance_) {
                oi.setDBLED(LEDState.BLINK_FAST) ;
            }
            else {
                oi.setDBLED(dbReadyToShoot() ? LEDState.ON : LEDState.OFF);
            }
            oi.setVelocityLED((shooter1_.isAtVelocity() && shooter2_.isAtVelocity()) ? LEDState.ON : LEDState.OFF);

            if (plot_id_ != -1) {
                data_[0] = sub_.getRobot().getTime() - start_time_ ;
                data_[1] = current_velocity_ ;
                data_[2] = sub_.getShooter1().getVelocity() ;
                data_[3] = sub_.getShooter2().getVelocity() ;
                data_[4] = current_updown_ ;
                data_[5] = sub_.getUpDown().getPosition() ;
                data_[6] = current_tilt_ ;
                data_[7] = sub_.getTilt().getPosition() ;
                data_[8] = sub_.getAbsEncoderAngle() ;
                data_[9] = db_ready_ ? 1.0 : 0.0 ;
                data_[10] = aprilTagTest() ? 0.5 : 0.0 ;
                data_[11] = drive_team_ready_ ? 1.5 : 0.0 ;
                data_[12] = robot.getTargetTracker().getDistance() ;
                data_[13] = robot.getTargetTracker().getOffset() ;
                data_[14] = swerve_stopped_ ? 2.0 : 0.0 ;
                data_[15] = gyro_stopped_ ? 2.5 : 0.0 ;
                data_[16] = abs_enc_ready_ ? 3.0 : 0.0 ;
                data_[17] = updown_.isAtTarget() ? 3.5 : 0.0 ; 
                data_[18] = at_target_supplier_.getAsBoolean() ? 4.0 : 0.0 ;
                data_[19] = (shooter1_.isAtVelocity() && shooter2_.isAtVelocity()) ? 4.5 : 0.0 ; 
                sub_.addPlotData(plot_id_, data_);
            }

            if (readyToShoot() && dist < max_distance_) {
                waiting_ = true ;
                wait_timer_.start() ;
            }
        }
    }

    private boolean dbReadyToShoot() {
        //
        // We use the gyro rotation as it is more accurate
        //
        XeroGyro gyro = sub_.getRobot().getRobotSubsystem().getDB().getGyro() ;
        AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;

        swerve_stopped_ = robot.getSwerve().isStopped() ;
        gyro_stopped_ =  Math.abs(gyro.getRate()) < rotational_velocity_threshold_ ;

        return  db_ready_ && swerve_stopped_ && gyro_stopped_ ;
    }

    private boolean readyToShoot() {
        AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;        
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
        logger.add("updown", updown_.isAtTarget()) ;
        logger.add("tilt", at_target_supplier_.getAsBoolean()) ;
        logger.add("abstilt", abs_enc_ready_) ;
        logger.add("shooter1", shooter1_.isAtVelocity());
        logger.add("shooter2", shooter2_.isAtVelocity());
        logger.add("april", aprilTagTest());
        logger.add("driveteam", drive_team_ready_);
        logger.add("dbready", db_ready_);
        logger.add("swerve_stopped", swerve_stopped_);
        logger.add("gyro_stopped", gyro_stopped_) ;
        logger.add("offset", robot.getTargetTracker().getOffset()) ;
        logger.endMessage();

        return  updown_.isAtTarget() && 
                at_target_supplier_.getAsBoolean() && 
                shooter1_.isAtVelocity() && 
                shooter2_.isAtVelocity() && 
                aprilTagTest() && 
                drive_team_ready_ &&
                dbReadyToShoot() &&
                abs_enc_ready_ ;
    }

    @Override
    public void cancel() {
        super.cancel();

        AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;              

        sub_.getFeeder().setPower(0.0);
        sub_.getShooter1().setPower(0.0);
        sub_.getShooter2().setPower(0.0);
        robot.getLimelight().setLedMode(LedMode.ForceOff) ;        
    }

    @Override
    public String toString(int indent) {
        return prefix(indent) + "IntakeAutoShootAction";
    }
}
