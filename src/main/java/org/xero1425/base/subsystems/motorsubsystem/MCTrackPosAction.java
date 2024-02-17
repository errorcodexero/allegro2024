package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroPidType;

public class MCTrackPosAction extends MotorAction {

    // The plot ID for the action
    private int plot_id_ ;

    // Data for each loop of the plot
    private Double data_[] ;

    // The columns to plot
    private static String [] columns_ = { 
        "time", 
        "target (%%posunits%%)", 
        "actual (%%posunits%%)",
        "mctarget (%%posunits%%)",
        "velocity (%%velunits%%)",
        "voltage (v)",
        "error (percent)"
    } ;    

    private double start_ ;
    private double target_ ;
    private double pos_threshold_ ;
    private double vel_threshold_ ;
    private String name_ ;
    private boolean is_at_target_ ;

    private static int which_ = 0 ;

    public MCTrackPosAction(MotorEncoderSubsystem sub, String name, String target, double posthresh, double velthresh, boolean plot) throws Exception {
        this(sub, name, sub.getSettingsValue(target).getDouble(), posthresh, velthresh, plot) ;
    }    

    public MCTrackPosAction(MotorEncoderSubsystem sub, String name, double target, double posthresh, double velthresh, boolean plot) throws Exception {
        super(sub);

        target_ = target ;
        name_ = name ;

        pos_threshold_ = posthresh ;
        vel_threshold_ = velthresh ;

        if (plot) {
            plot_id_ = sub.initPlot(toString(0) + "-" + String.valueOf(which_++)) ;     
        }
        else {
            plot_id_ = -1 ;
        }
        data_ = new Double[columns_.length] ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        if (plot_id_ != -1) {
            MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
            getSubsystem().startPlot(plot_id_, convertUnits(columns_, me.getUnits())) ;
        }
        start_ = getSubsystem().getRobot().getTime() ;

        //
        // We are using a control loop in the motor controller, get the parameters from the
        // settings file
        //
        XeroEncoder enc = ((MotorEncoderSubsystem)getSubsystem()).getEncoder() ;
        double p = getSubsystem().getSettingsValue(name_ + ":kp").getDouble() / enc.mapPhysicalToMotor(1) ;
        double i = getSubsystem().getSettingsValue(name_ + ":ki").getDouble() / enc.mapPhysicalToMotor(1) ;
        double d = getSubsystem().getSettingsValue(name_ + ":kd").getDouble() / enc.mapPhysicalToMotor(1) ;
        double v = getSubsystem().getSettingsValue(name_ + ":kv").getDouble() / enc.mapPhysicalToMotor(1) ;
        double a = getSubsystem().getSettingsValue(name_ + ":ka").getDouble() / enc.mapPhysicalToMotor(1) ;
        double s = getSubsystem().getSettingsValue(name_ + ":ks").getDouble() ;
        double g = getSubsystem().getSettingsValue(name_ + ":kg").getDouble() ;
        double outmax = getSubsystem().getSettingsValue(name_ + ":outmax").getDouble() ;
        getSubsystem().getMotorController().setPID(XeroPidType.MotionMagic, p, i, d, v, a, g, s, outmax);

        double maxv = getSubsystem().getSettingsValue(name_ + ":maxv").getDouble() * enc.mapPhysicalToMotor(1) ;
        double maxa = getSubsystem().getSettingsValue(name_ + ":maxa").getDouble() * enc.mapPhysicalToMotor(1) ;
        double jerk = getSubsystem().getSettingsValue(name_ + ":jerk").getDouble() * enc.mapPhysicalToMotor(1) ;
        getSubsystem().getMotorController().setMotionMagicParams(maxv, maxa, jerk) ;

        setTarget(target_) ;
    }

    /// \brief Update the target velocity to a new velocity
    /// \param target the target velocity desired
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        //
        // If we are running the loop in the motor controller, commuincate the new target to the
        // motor controller.  Since the motor controller does not run its PID loop in robot units,
        // we must as the subsystem to translate the units from robot units to motor controller units.
        //
        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
        double pos = me.getPosition() ;
        double vel = me.getVelocity() ;
        double error = Math.abs(pos - target_) ;

        double ticks = me.getEncoder().mapPhysicalToMotor(target) ;
        getSubsystem().getMotorController().set(XeroPidType.MotionMagic, ticks) ;

        if (error < pos_threshold_ && Math.abs(vel) < vel_threshold_) {
            is_at_target_ = true ;
        }
        else {
            is_at_target_ = false ;
        }
    }

    public boolean isAtTarget() {
        return is_at_target_ ;
    }

    @Override
    public void run() throws Exception  {
        super.run() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();

        double pos = me.getPosition() ;
        double vel = me.getVelocity() ;
        double error = Math.abs(pos - target_) ;
        double mctarget = me.getMotorController().getClosedLoopTarget() ;
        mctarget = me.getEncoder().mapMotorToPhysical(mctarget) ;

        if (plot_id_ != -1) {
            data_[0] = getSubsystem().getRobot().getTime() - start_ ;
            data_[1] = target_ ;
            data_[2] = pos ;
            data_[3] = mctarget ;
            data_[4] = vel ;
            data_[5] = me.getMotorController().getVoltage() ;
            data_[6] = error ;
            getSubsystem().addPlotData(plot_id_, data_);
        }

        if (error < pos_threshold_ && Math.abs(vel) < vel_threshold_) {
            is_at_target_ = true ;
        }
    }
    
    /// \brief Cancel the velocity action, settings the power of the motor to zero
    @Override
    public void cancel() {
        super.cancel() ;

        try {
            getSubsystem().getMotorController().set(XeroPidType.Power, 0.0);
        }
        catch(Exception ex) {
        }

        getSubsystem().setPower(0.0);
        if (plot_id_ != -1)
            getSubsystem().endPlot(plot_id_) ;
    }    

    @Override
    public String toString(int indent) {
        String ret ;

        ret = spaces(indent) + "MCTrackPosAction (" + getSubsystem().getName() + ")";
        ret += " target=" + target_ ;        

        return ret ;
    }
}
