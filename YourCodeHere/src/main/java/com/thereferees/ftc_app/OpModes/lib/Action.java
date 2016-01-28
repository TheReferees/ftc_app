package com.thereferees.ftc_app.OpModes.lib;

/**
 * Created by Saba on 1/26/16.
 */
public abstract class Action {
    public abstract void action();
    public abstract void onComplete();
    public abstract void init();

    public boolean hasInitialized = false;
}
