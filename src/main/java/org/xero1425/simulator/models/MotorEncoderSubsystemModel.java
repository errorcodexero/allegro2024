package org.xero1425.simulator.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

public class MotorEncoderSubsystemModel extends SimulationModel {
    SparkMaxSimMotorController motor_ ;

    public MotorEncoderSubsystemModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }   

    @Override
    public boolean create(SimulationEngine engine) throws Exception {
        String bus = getStringProperty("bus") ;
        int canid = getIntProperty("canid") ;
        double ticks = getDoubleProperty("ticks-per-rev-per-second") ;

        motor_ = new SparkMaxSimMotorController(engine, bus, canid, ticks);

        setCreated();
        return true ;
    }

    @Override
    public void run(double dt) {
        motor_.run(dt) ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue v) {
        return true ;
    }
}
