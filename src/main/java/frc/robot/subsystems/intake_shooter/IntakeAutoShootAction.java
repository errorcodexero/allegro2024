package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.oi.OILed.LEDState;
import org.xero1425.base.subsystems.swerve.SwerveTrackAngle;
import org.xero1425.base.utils.PieceWiseLinear;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private boolean shooting_ ;
    private boolean waiting_ ;

    private XeroTimer wait_timer_ ;

    private boolean verbose_ ;
    private boolean db_ready_ ;

    private boolean swerve_stopped_ ;
    private boolean gyro_stopped_ ;

    private double aim_threshold_ ;
    private double tilt_stow_value_ ;

    private String strategy_ ;

    private double accel_threshold_ ;
    private double rotational_velocity_threshold_ ;

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
        "tilt (deg)",
        "dbready (bool)",
        "april-tag (bool)",
        "oiready (bool)",
        "distance (m)"
    } ;

    public IntakeAutoShootAction(IntakeShooterSubsystem intake, TargetTrackerSubsystem tracker, boolean initialDriveTeamReady, SwerveTrackAngle rotate) throws Exception {
        super(intake.getRobot().getMessageLogger());

        sub_ = intake ;
        tracker_ = tracker ;
        initial_drive_team_ready_ = initialDriveTeamReady ;
        rotate_ = rotate ;
        verbose_ = false ;

        ISettingsSupplier settings = sub_.getRobot().getSettingsSupplier() ;
        if (settings.isDefined("system:verbose:auto-shoot")) {
            verbose_ = settings.get("system:verbose:auto-shoot").getBoolean() ;
        } 

        String strategy = sub_.getRobot().getSettingsSupplier().get("subsystems:targettracker:strategy").getString() ;
        if (strategy.equals("pose")) {
            strategy_ = "pose" ;
        }
        else {
            strategy_ = "triangle" ;
        }
        
        tilt_stow_value_ = sub_.getTilt().getSettingsValue("targets:stow").getDouble() ;
        aim_threshold_ = sub_.getSettingsValue("actions:auto-shoot:aim-threshold").getDouble() ;
        accel_threshold_ = sub_.getSettingsValue("actions:auto-shoot:gyro-accel-threshold").getDouble() ;
        rotational_velocity_threshold_ = sub_.getSettingsValue("actions:auto-shoot:rotational-velocity-threshold").getDouble() ;

        double velthresh = sub_.getSettingsValue("actions:auto-shoot:shooter-velocity-threshold").getDouble() ;
        double updown_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-pos-threshold").getDouble() ;
        double updown_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:updown-velocity-threshold").getDouble() ;
        double tilt_pos_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-pos-threshold").getDouble() ;
        double tilt_vel_threshold = sub_.getSettingsValue("actions:auto-shoot:tilt-velocity-threshold").getDouble() ;

        wait_timer_ = new XeroTimer(intake.getRobot(), "wait-timer", 0.1);

        if (RobotBase.isSimulation()) {
            velthresh = 10.0 ;  
            updown_pos_threshold = 10.0 ;
            updown_vel_threshold = 10.0 ;
            tilt_pos_threshold = 10.0 ;
            tilt_vel_threshold = 10.0 ;
        }

        updown_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls-" + strategy_ + ":updown") ;
        tilt_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls-" + strategy_ + ":tilt") ;
        velocity_pwl_ = new PieceWiseLinear(sub_.getRobot().getSettingsSupplier(), "subsystems:intake-shooter:pwls-" + strategy_ + ":shooter") ;

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

        try {
            ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
            tab.addBoolean("tilt", ()->tilt_.isAtTarget()) ;
            tab.addBoolean("updown", ()->updown_.isAtTarget()) ;
            tab.addBoolean("shooter1", ()->shooter1_.isAtVelocity());
            tab.addBoolean("shooter2", ()->shooter2_.isAtVelocity()) ;
            tab.addBoolean("dbangle", ()-> { return db_ready_ ; }) ;
            tab.addBoolean("swerve", ()-> { return swerve_stopped_ ; }) ;
            tab.addBoolean("gyro", ()-> { return gyro_stopped_ ; }) ;                        
            tab.addBoolean("apriltag", ()->aprilTagTest()) ;
            tab.addBoolean("button", ()-> { return drive_team_ready_ ; });
        }
        catch(Exception ex) {
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
        waiting_ = false ;
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
        boolean ret = true ;
        if (rotate_ != null && !tracker_.seesTarget()) {
            ret = false ;
        }
        return ret ;
    }

    @Override
    public void run() throws Exception {
        super.run();

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
            oi.setDBLED(dbReadyToShoot() ? LEDState.ON : LEDState.OFF);
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
                data_[8] = db_ready_ ? 1.0 : 0.0 ;
                data_[9] = aprilTagTest() ? 0.5 : 0.0 ;
                data_[10] = drive_team_ready_ ? 1.5 : 0.0 ;
                data_[11] = robot.getTargetTracker().getDistance() ;
                sub_.addPlotData(plot_id_, data_);
            }

            if (readyToShoot()) {
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
        gyro_stopped_ =     Math.abs(gyro.getRate()) < rotational_velocity_threshold_ && 
                            Math.abs(gyro.getAccelX()) < accel_threshold_ && 
                            Math.abs(gyro.getAccelY()) < accel_threshold_ ; 

        MessageLogger logger = robot.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info).add("gyroinfo");
        logger.add("rate", gyro.getRate()) ;
        logger.add("ratelimit", rotational_velocity_threshold_) ;
        logger.add("ax", gyro.getAccelX()) ;
        logger.add("ay", gyro.getAccelY()) ;
        logger.add("alimit", accel_threshold_) ;
        logger.endMessage();

        return  db_ready_ && swerve_stopped_ && gyro_stopped_ ;
    }

    private boolean readyToShoot() {
        MessageLogger logger = sub_.getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, sub_.getLoggerID()) ;
        logger.add("updown", updown_.isAtTarget()) ;
        logger.add("tilt", tilt_.isAtTarget());
        logger.add("shooter1", shooter1_.isAtVelocity());
        logger.add("shooter2", shooter2_.isAtVelocity());
        logger.add("april", aprilTagTest());
        logger.add("driveteam", drive_team_ready_);
        logger.add("dbready", db_ready_);
        logger.add("swerve_stopped", swerve_stopped_);
        logger.add("gyro_stopped", gyro_stopped_) ;
        logger.endMessage();

        return  updown_.isAtTarget() && 
                tilt_.isAtTarget() && 
                shooter1_.isAtVelocity() && 
                shooter2_.isAtVelocity() && 
                aprilTagTest() && 
                drive_team_ready_ &&
                dbReadyToShoot() ;
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
