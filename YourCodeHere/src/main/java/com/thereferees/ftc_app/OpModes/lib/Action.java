package com.thereferees.ftc_app.OpModes.lib;

/**
 * Created by Saba on 1/26/16.
 */
public abstract class Action {
    public abstract void init();
    public abstract void onComplete();

    public static long startTime = 0;

    public void action() {

    }
}
