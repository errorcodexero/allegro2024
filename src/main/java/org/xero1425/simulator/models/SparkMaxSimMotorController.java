package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationEngine;
import com.revrobotics.CANSparkMax;

import org.xero1425.base.motors.IMotorController;
import org.xero1425.base.motors.SparkMaxMotorController;

public class SparkMaxSimMotorController implements ISimMotorController {
    private double ticks_ ;
    private double last_ticks_ ;
    private double velocity_ ;

    private static final double kTicksPerRev = 42 ;
    private SparkMaxMotorController motor_ ;
    private double ticks_per_volt_per_second_ ;
    
    
    public SparkMaxSimMotorController(SimulationEngine engine, String bus, int canid, double ticksPerVoltPerSecond) throws Exception {
        if (!bus.isEmpty()) {
            throw new Exception("SparkMax controllers must have an empty bus");
        }

        IMotorController ctrl = engine.getRobot().getMotorFactory().getMotorController(bus, canid) ;
        if (!(ctrl.getNativeController() instanceof CANSparkMax)) {
            throw new Exception("motor on bus '" + bus + "', can id " + canid + " is not a CANSparkMax motor");
        }

        motor_ = (SparkMaxMotorController)ctrl ;
        ticks_ = 0.0 ;
        last_ticks_ = 0.0 ;
        ticks_per_volt_per_second_ = ticksPerVoltPerSecond ;
    }

    @Override
    public void run(double dt) {
        SparkMaxMotorController.SimState state = motor_.getSimState() ;

        ticks_ += ticks_per_volt_per_second_ * state.getMotorVoltage() * dt ;
        velocity_ = (ticks_ - last_ticks_) / dt ;

        last_ticks_ = ticks_ ;

        state.setPosition(ticks_);
        state.setVelocity(velocity_);
    }

    @Override
    public double ticksPerRev() {
        return kTicksPerRev ;
    }
} ;