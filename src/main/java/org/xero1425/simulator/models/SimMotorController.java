package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationEngine;

public abstract class SimMotorController implements ISimMotorController {
    final static boolean kPlotMotorSims = true ;

    private SimulationEngine engine_ ;
    private int plot_id_ ;
    private Double [] data_ ;

    private static String [] plot_cols_ = { "time (s)", "battery (v)", "motor (v)", "pos (rot)", "vel (rps)" } ;

    protected SimMotorController(SimulationEngine engine, String bus, int canid) {
        engine_ = engine ;

        if (kPlotMotorSims) {
            String busname = (bus.length() > 0) ? bus : "<EMPTY>" ;
            data_= new Double[plot_cols_.length] ;
            plot_id_ = engine.getRobot().getPlotManager().initPlot("talon-" + busname + "-" + canid) ;
            engine.getRobot().getPlotManager().startPlot(plot_id_, plot_cols_);
        }
    }

    public void addPlotData(double bvolts, double mvolts, double pos, double vel) {
        if (kPlotMotorSims) {
            data_[0] = engine_.getRobot().getTime() ;
            data_[1] = bvolts ;
            data_[2] = mvolts ;
            data_[3] = pos ;
            data_[4] = vel ;
            engine_.getRobot().getPlotManager().addPlotData(plot_id_, data_);
        }
    }

    protected SimulationEngine getEngine() {
        return engine_ ;
    }
}
