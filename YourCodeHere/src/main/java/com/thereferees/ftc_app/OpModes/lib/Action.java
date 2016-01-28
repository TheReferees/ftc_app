package com.thereferees.ftc_app.OpModes.lib;

/**
 * Created by Saba on 1/26/16.
 */
public interface Action {
    void action();
    void onComplete();
    void init();

    boolean hasInitialized = false;
}
