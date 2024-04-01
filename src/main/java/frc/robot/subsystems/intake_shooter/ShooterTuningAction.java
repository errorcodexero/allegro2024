package frc.robot.subsystems.intake_shooter;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MCMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MCTrackPosAction;
import org.xero1425.base.subsystems.motorsubsystem.MCVelocityAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.oi.OILed.LEDState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.subsystems.oi.Allegro2024OISubsystem;
import frc.robot.subsystems.toplevel.AllegroRobot2024;

public class ShooterTuningAction extends Action {

    private enum PlotStatus {
        Start,
        Stop,
        Nothing
    } ;

    private final double kFeederPowerLevel = 0.4 ;
    private final double kTiltLowerLimit = -70.0 ;
    private final double kTiltUpperLimit = 45.0 ;
    private final double kShooterVelocityLimit = 100 ;

    private final double kTiltPositionTolerance = 2.0 ;
    private final double kTiltVelocityTolerance = 1.0 ;

    private final double kVelocityFireTolerance = 10.0 ;
    private final double kTiltFireTolerance = 1.0 ;

    private static int plot_number_ = 1 ;

    private IntakeShooterSubsystem sub_ ;
    private SimpleWidget tilt_widget_ ;
    private SimpleWidget velocity_widget_ ;
    private SimpleWidget apply_widget_ ;
    private SimpleWidget plot_widget_ ;
    private boolean last_apply_value_ ;
    private boolean last_plot_value_ ;
    private boolean wait_for_ready_ ;
    private int plot_id_ ;
    private boolean plotting_ ;
    private double plot_start_time_ ;

    private MCVelocityAction velocity1_action_ ;
    private MCVelocityAction velocity2_action_ ;
    private MCMotionMagicAction tilt_action_ ;
    private MCTrackPosAction updown_action_ ;
    private MotorEncoderPowerAction feeder_start_action_ ;
    private MotorEncoderPowerAction feeder_stop_action_ ;
    private ShuffleboardTab tab_ ;

    private double current_tilt_ ;
    private double current_velocity_ ;
    private Double [] plot_data_ ;

    private double updown_ ;
    private boolean wait_tilt_ ;

    private static final String[] plot_columns_ = { 
        "time (s)",
        "s1-vel-t (%%velunits%%)",
        "s1-vel-a (%%velunits%%)",        
        "s2-vel-t (%%velunits%%)",
        "s2-vel-a (%%velunits%%)",        
        "tilt-t (v)",
        "tilt-a (v)",
        "fire (bool)"
    } ;    

    public ShooterTuningAction(IntakeShooterSubsystem sub, double updown, double init) throws Exception {
        super(sub.getRobot()) ;
        sub_ = sub ;
        updown_ = updown ;

        velocity1_action_ = new MCVelocityAction(sub_.getShooter1(), "pids:velocity", 0.0, kVelocityFireTolerance, false);
        velocity2_action_ = new MCVelocityAction(sub_.getShooter2(), "pids:velocity", 0.0, kVelocityFireTolerance, false);
        tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", init, kTiltPositionTolerance, kTiltVelocityTolerance) ;
        feeder_start_action_ = new MotorEncoderPowerAction(sub_.getFeeder(), kFeederPowerLevel);
        feeder_stop_action_ = new MotorEncoderPowerAction(sub_.getFeeder(), 0.0);
        updown_action_ = new MCTrackPosAction(sub_.getUpDown(), "pids:position", updown_, 2, 1, false) ;

        plot_data_ = new Double[plot_columns_.length] ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        wait_for_ready_ = true ;
        last_apply_value_ = false ;
        last_plot_value_ = false ;
        plotting_ = false ;
        wait_tilt_ = true ;
        tab_ = Shuffleboard.getTab("Tuning");
        tilt_widget_ = tab_.add("Tilt Input", 0.0).withWidget(BuiltInWidgets.kTextView) ;
        velocity_widget_ = tab_.add("Velocity Input", 0.0).withWidget(BuiltInWidgets.kTextView) ;
        apply_widget_ = tab_.add("Apply", false).withWidget(BuiltInWidgets.kToggleSwitch) ;
        plot_widget_ = tab_.add("Plot", false).withWidget(BuiltInWidgets.kToggleSwitch) ;

        tab_.addDouble("VSET", () -> { return current_velocity_ ;});
        tab_.addDouble("TSET", () -> { return current_tilt_ ;});

        sub_.getShooter1().setAction(velocity1_action_, true) ;
        sub_.getShooter2().setAction(velocity2_action_, true) ;
        sub_.getTilt().setAction(tilt_action_, true) ;
    }

    private PlotStatus checkPlot() {
        PlotStatus ret = PlotStatus.Nothing ;
        boolean curval = plot_widget_.getEntry().getBoolean(false) ;

        if (curval && !last_plot_value_) {
            ret = PlotStatus.Start ;
        }
        else if (!curval && last_plot_value_) {
            ret = PlotStatus.Stop ;
        }
        last_plot_value_ = curval ;
        return ret;
    }

    private boolean applySettings() {
        boolean curval = apply_widget_.getEntry().getBoolean(false) ;
        boolean ret = curval && !last_apply_value_ ;

        last_apply_value_ = curval ;
        return ret ;
    }    

    @Override
    public void run() throws Exception {
        super.run() ;
        PlotStatus st = checkPlot() ;
        boolean fire = false ;


        if (st == PlotStatus.Start) {
            plot_id_ = sub_.initPlot("ShooterTuning-" + plot_number_++) ;
            sub_.startPlot(plot_id_, plot_columns_);
            plotting_ = true ;
            plot_start_time_ = sub_.getRobot().getTime();
        }
        else if (st == PlotStatus.Stop) {
            sub_.endPlot(plot_id_);
            plotting_ = false ;
        }

        if (wait_tilt_) {
            sub_.getUpDown().setAction(updown_action_, true) ;
            wait_tilt_ = false ;
        }
        else if (applySettings()) {
            double v ;

            //
            // We are applying the settings, reset the switch to its off position
            //
            apply_widget_.getEntry().setBoolean(false) ;

            //
            // Apply the velocity if its in the limits
            //
            v = velocity_widget_.getEntry().getDouble(0.0) ;
            if (v <= kShooterVelocityLimit) {
                current_velocity_ = v ;
                velocity1_action_.setTarget(current_velocity_);
                velocity2_action_.setTarget(current_velocity_);
                sub_.getFeeder().setAction(feeder_stop_action_, true) ;
                wait_for_ready_ = true ;
            }

            //
            // Apply the tilt if its in the limits
            //
            v = tilt_widget_.getEntry().getDouble(0.0) ;
            if (v >= kTiltLowerLimit && v <= kTiltUpperLimit && Math.abs(v - current_tilt_) > 0.1) {
                current_tilt_ = v ;
                tilt_action_ = new MCMotionMagicAction(sub_.getTilt(), "pids:position", current_tilt_, kTiltPositionTolerance, kTiltVelocityTolerance) ;
                sub_.getTilt().setAction(tilt_action_, true) ;
                sub_.getFeeder().setAction(feeder_stop_action_, true) ;
                wait_for_ready_ = true ;
            }
        }

        fire = isReady() ;

        if (wait_for_ready_ && fire) {
            wait_for_ready_ = false ;
            sub_.getFeeder().setAction(feeder_start_action_, true) ;
        }

        if (plotting_) {
            plot_data_[0] = sub_.getRobot().getTime() - plot_start_time_ ;
            plot_data_[1] = current_velocity_ ;
            plot_data_[2] = sub_.getShooter1().getVelocity() ;
            plot_data_[3] = current_velocity_ ;
            plot_data_[4] = sub_.getShooter2().getVelocity() ;
            plot_data_[5] = current_tilt_ ;
            plot_data_[6] = sub_.getTilt().getPosition() ;
            plot_data_[7] = fire ? 1.0 : 0.0 ;
            sub_.addPlotData(plot_id_, plot_data_);
        }
    }

    private boolean isReady() {
        AllegroRobot2024 robot = (AllegroRobot2024)sub_.getRobot().getRobotSubsystem() ;
        Allegro2024OISubsystem oi = robot.getOI() ;

        boolean tiltready = Math.abs(sub_.getTilt().getPosition() - current_tilt_) < kTiltFireTolerance ;
        oi.getPanel().setDBLED(velocity1_action_.isAtVelocity() ? LEDState.ON : LEDState.OFF) ;
        oi.getPanel().setVelocityLED(velocity2_action_.isAtVelocity() ? LEDState.ON : LEDState.OFF) ;
        oi.getPanel().setTiltLED(tiltready ? LEDState.ON : LEDState.OFF) ;
        oi.getPanel().setAprilTagLED(updown_action_.isAtTarget() ? LEDState.ON : LEDState.OFF) ;

        return velocity1_action_.isAtVelocity() && velocity2_action_.isAtVelocity() &&  tiltready && updown_action_.isAtTarget() ;
    }
    
    public String toString(int indent) {
        return prefix(indent) + "TuningAction" ;
    }
}
