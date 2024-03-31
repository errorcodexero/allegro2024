package org.xero1425.base.subsystems.motorsubsystem;

/// \file

/// \brief This action applies a sequence of motor powers to a motor
public class MotorPowerSequenceAction extends MotorAction {

    // The set of times for each motor power, should be identical in size to powers_
    private double[] times_;

    // The set of powers, should be identical in size of times_
    private double[] powers_;

    // The current index in the times_ and powers_ arrays
    private int index_;

    // The current time within the current index_
    private double index_time_;

    // The start time for a timed action
    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    private Double data_[] ;

    // The columns to plot
    private static final String[] plot_columns_ = { 
        "time (s)",
        "pos (%%posunits%%)",
        "vel (%%velunits%%)",
        "out (v)",
        "encoder (ticks)" 
    } ;    

    /// \brief Create a new MotorPowerSequenceAction
    /// \param sub the MotorPower subsystem this action is for
    /// \param times the set of times, must match the powers size below
    /// \param powers the set of powers, must match the times size below
    /// \throws Exception if the times and powers arrays are not the same size
    public MotorPowerSequenceAction(MotorSubsystem sub, double[] times, double[] powers) throws Exception {
        super(sub);

        times_ = times;
        powers_ = powers;

        if (times_.length == 0 || powers_.length == 0 || times_.length != powers_.length)
            throw new Exception("invalid arguments to MotorPowerSequenceAction action");

        plot_id_ = sub.initPlot(toString(0)) ;
        data_ = new Double[plot_columns_.length] ;            
    }

    /// \brief Start the action
    @Override
    public void start() throws Exception {
        super.start();

        index_ = 0;
        setupCurrentIndex();

        start_ = getSubsystem().getRobot().getTime() ;
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem();
        getSubsystem().startPlot(plot_id_, convertUnits(plot_columns_, sub.getUnits()));        
    }

    /// \brief run the action.  If the current power has expired, it moves to the next power in the
    /// arrays of times and powers.  When the last entry is finished, it marks the action as done.
    @Override
    public void run() throws Exception {
        super.run();

        data_[0] = getSubsystem().getRobot().getTime() - start_ ;
        data_[1] = ((MotorEncoderSubsystem)(getSubsystem())).getPosition() ;
        data_[2] = ((MotorEncoderSubsystem)(getSubsystem())).getVelocity() ;
        data_[3] = getSubsystem().getPower() ;
        data_[4] = ((MotorEncoderSubsystem)(getSubsystem())).getEncoderRawCount() ;
        getSubsystem().addPlotData(plot_id_, data_);        

        if (getSubsystem().getRobot().getTime() - index_time_ > times_[index_]) {
            index_++;
            if (index_ == times_.length) {
                getSubsystem().endPlot(plot_id_) ;                
                setDone();
            } else {
                setupCurrentIndex();
            }
        }
    }

    /// \brief Cancel the action, setting the motor power to 0.0
    @Override
    public void cancel() {
        super.cancel();

        index_ = times_.length;
        getSubsystem().setPower(0.0);

        getSubsystem().endPlot(plot_id_) ;        
    }

    /// \brief Returns a human readable string representing the action
    /// \returns a human readable string representing the action
    @Override
    public String toString(int indent) {
        StringBuilder bld = new StringBuilder() ;

        for(int i = 0 ; i < times_.length ; i++)
        {
            bld.append('(') ;
            bld.append(Double.toString(times_[i])) ;
            bld.append(',') ;
            bld.append(Double.toString(powers_[i])) ;
            bld.append(')') ;
        }
        return spaces(indent) + "MotorPowerSequenceAction " + bld.toString() ;
    }

    private void setupCurrentIndex() {
        index_time_ = getSubsystem().getRobot().getTime();
        getSubsystem().setPower(powers_[index_]);
    }    
}
