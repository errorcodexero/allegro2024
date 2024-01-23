package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This action causes a MotorEncoderSubsystem to maintain a constant velocity.
public class MCVelocityAction extends MotorAction {   

    private final static double kDefaultPlotDuration = 5.0 ;

    // An index used to ensure individual instances of this action produce separate plots
    private static int which_ = 1 ;

    // The target velocity
    private double target_ ;

    // The error in the last robot loop
    private double error_ ;

    // The start time for the action
    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // The name of the action
    private String name_ ;

    // The duration of the plot request
    private double plot_duration_ ;

    // The timer for the plot
    private XeroTimer plot_timer_ ;

    // Data for each loop of the plot
    private Double data_[] ;

    // The columns to plot
    private static String [] columns_ = { 
        "time", 
        "target (%%velunits%%)", 
        "actual (%%velunits%%)", 
        "error (%%velunits%%)" 
    } ;

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param name the name of the action, for entries from the settings file
    /// \param target the traget velocity
    public MCVelocityAction(MotorEncoderSubsystem sub, String name, double target)
            throws MissingParameterException, BadParameterTypeException, BadMotorRequestException, MotorRequestFailedException {

        super(sub);

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;

        name_ = name ;
        target_ = target;

        String pidname = "subsystems:" + sub.getName() + ":" + name_ ;

        plot_id_ = sub.initPlot(toString(0) + "-" + String.valueOf(which_++)) ;     
        plot_duration_ = kDefaultPlotDuration ;

        if (settings.isDefined(pidname + ":plot-duration")) {
            plot_duration_ = settings.get(pidname + ":plot-duration").getDouble() ;
        }
        plot_timer_ = new XeroTimer(sub.getRobot(), "velocity-action-plot", plot_duration_) ;
        data_ = new Double[columns_.length] ;
    }

    /// \brief Create a new MotorEncoderVelocityAction
    /// \param sub the target MotorEncoderSubsystem
    /// \param target a string with the name of the target velocity in settings file
    public MCVelocityAction(MotorEncoderSubsystem sub, String name, String target) throws BadParameterTypeException, MissingParameterException, BadMotorRequestException, MotorRequestFailedException {
        this(sub, name, sub.getSettingsValue(target).getDouble()) ;
    }

    public double getError() {
        return error_ ;
    }

    /// \brief Return the name of the action
    /// \returns the name of the action
    public String getName() {
        return name_ ;
    }

    /// \brief Update the target velocity to a new velocity
    /// \param target the target velocity desired
    public void setTarget(double target) throws BadMotorRequestException, MotorRequestFailedException {
        target_ = target ;

        //
        // If we are running the loop in the motor controller, commuincate the new target to the
        // motor controller.  Since the motor controller does not run its PID loop in robot units,
        // we must as the subsystem to translate the units from robot units to motor controller units.
        //
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        double ticks = sub.getEncoder().mapPhysicalToMotor(target) ;
        getSubsystem().getMotorController().set(XeroPidType.Velocity, ticks) ;
    }

    /// \brief Returns the current target
    /// \returns the current target
    public double getTarget() {
        return target_ ;
    }

    /// \brief Start the velocity action
    @Override
    public void start() throws Exception {
        super.start() ;

        if (plot_id_ != -1) {
            MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
            getSubsystem().startPlot(plot_id_, convertUnits(columns_, me.getUnits())) ;
            plot_timer_.start() ;
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
        double s = getSubsystem().getSettingsValue(name_ + ":ks").getDouble() / enc.mapPhysicalToMotor(1) ;
        double g = getSubsystem().getSettingsValue(name_ + ":kg").getDouble() / enc.mapPhysicalToMotor(1) ;
        double outmax = getSubsystem().getSettingsValue(name_ + ":outmax").getDouble() ;

        getSubsystem().getMotorController().setPID(XeroPidType.Velocity, p, i, d, v, a, g, s, outmax) ;
        setTarget(target_) ;
    }

    /// \brief Process the velocity action once per robot loop, adjusting the power as needed
    @Override
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;
        error_ = target_ - me.getVelocity() ;

        if (plot_id_ != -1) {
            data_[0] = getSubsystem().getRobot().getTime() - start_ ;
            data_[1] = target_ ;
            data_[2] = me.getVelocity() ;
            data_[3] = error_ ;
            getSubsystem().addPlotData(plot_id_, data_);

            if (plot_timer_.isExpired()) {
                getSubsystem().endPlot(plot_id_) ;
                plot_id_ = -1 ;
            }
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

    /// \brief return a human readable string for the action
    /// \param indent the amount of white space prior to the description
    /// \returns a human readable string for the action
    @Override
    public String toString(int indent) {
        String ret = null ;

        ret = prefix(indent) + "MCVelocityAction, " + getSubsystem().getName() + ", " +  Double.toString(target_) ;
        return ret ;
    }
}
