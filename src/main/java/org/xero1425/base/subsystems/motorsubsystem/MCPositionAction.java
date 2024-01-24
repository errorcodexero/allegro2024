package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.IMotorController.XeroPidType;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;

public class MCPositionAction extends MotorAction {

    // An index used to ensure individual instances of this action produce separate plots
    private static int which_ = 1 ;

    private double target_ ;
    private int plot_id_ ;
    private double start_ ;
    private XeroTimer plot_timer_ ;
    private String name_ ;
    private double plot_duration_ ;

    private String [] columns_ = new String[] {
        "time (s)",
        "position (%%posunits%%)",
        "velocity (%%velunits%%)",
    } ;

    public MCPositionAction(MotorEncoderSubsystem sub, String name, double target) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        String pidname = "subsystems:" + sub.getName() + ":" + name_ ;        

        name_ = name ;
        target_ = target;

        plot_id_ = sub.initPlot(toString(0) + "-" + String.valueOf(which_++)) ;   
        plot_duration_ = 10.0 ;
        if (settings.isDefined(pidname + ":plot-duration")) {
            plot_duration_ = settings.get(pidname + ":plot-duration").getDouble() ;
        }
        plot_timer_ = new XeroTimer(sub.getRobot(), "velocity-action-plot", plot_duration_) ;         
    }

    public MCPositionAction(MotorEncoderSubsystem sub, String name, String target) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        name_ = name ;        

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        String pidname = "subsystems:" + sub.getName() + ":" + name_ ;

        target_ = getSubsystem().getSettingsValue(target).getDouble() ;

        plot_id_ = sub.initPlot(toString(0) + "-" + String.valueOf(which_++)) ;   
        plot_duration_ = 10.0 ;
        if (settings.isDefined(pidname + ":plot-duration")) {
            plot_duration_ = settings.get(pidname + ":plot-duration").getDouble() ;
        }
        plot_timer_ = new XeroTimer(sub.getRobot(), "velocity-action-plot", plot_duration_) ;         
    }    
    
    @Override
    public void start() throws Exception {
       super.start() ;

        if (plot_id_ != -1) {
            getSubsystem().startPlot(plot_id_, columns_) ;
            plot_timer_.start() ;
        }

        start_ = getSubsystem().getRobot().getTime() ;

        //
        // We are using a control loop in the motor controller, get the parameters from the
        // settings file
        //
        XeroEncoder enc = ((MotorEncoderSubsystem)getSubsystem()).getEncoder() ;
        double p = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":kp").getDouble()) ;
        double i = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":ki").getDouble()) ;
        double d = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":kd").getDouble()) ;
        double v = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":kv").getDouble()) ;
        double a = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":ka").getDouble()) ;
        double s = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":ks").getDouble()) ;
        double g = enc.mapPhysicalToMotor(getSubsystem().getSettingsValue(name_ + ":kg").getDouble()) ;
        double outmax = getSubsystem().getSettingsValue(name_ + ":outmax").getDouble() ;

        getSubsystem().getMotorController().setPID(XeroPidType.Velocity, p, i, d, v, a, g, s, outmax) ;
        setTarget(target_);        
    }

    /// \brief Process the velocity action once per robot loop, adjusting the power as needed
    @Override
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem() ;

        if (plot_id_ != -1) {
            Double[] data = new Double[columns_.length] ;
            data[0] = getSubsystem().getRobot().getTime() - start_ ;
            data[1] = target_ ;
            data[2] = me.getVelocity() ;
            getSubsystem().addPlotData(plot_id_, data);

            if (plot_timer_.isExpired()) {
                getSubsystem().endPlot(plot_id_) ;
                plot_id_ = -1 ;
            }
        }
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
        getSubsystem().getMotorController().set(XeroPidType.Position, ticks) ;
    }    

    /// \brief return a human readable string for the action
    /// \param indent the amount of white space prior to the description
    /// \returns a human readable string for the action
    @Override
    public String toString(int indent) {
        String ret = null ;

        ret = prefix(indent) + "MCPositionAction, " + getSubsystem().getName() + ", " +  Double.toString(target_) ;
        return ret ;
    }    
}
