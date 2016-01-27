package com.thereferees.ftc_app.OpModes.lib;
/**
 * Created by Andrew on 11/18/2015.
 */
public interface PIDInterface {
    public void setTarget(double target);
    public void setMaxPower(double maxPower);
    public void setMinPower(double minPower);
}
