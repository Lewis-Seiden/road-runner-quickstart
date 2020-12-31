package org.firstinspires.ftc.teamcode.hardware;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ElevatorMoveEventManager extends Thread {
    Launcher launcher;
    LinearOpMode parentOP;
    public boolean elevatorMoveRequsted;
    int offset = 0;

    public ElevatorMoveEventManager(Launcher launcher, LinearOpMode parentOP)
    {
        this.launcher = launcher;
        this.parentOP = parentOP;
        elevatorMoveRequsted = false;
    }
    public void run()
    {
        while (!parentOP.isStopRequested())
        {
            if(elevatorMoveRequsted)
            {
                SystemClock.sleep(300);
                launcher.elevatorMove(0.8, offset);
                SystemClock.sleep(1200);
                elevatorMoveRequsted = false;

            }

        }
    }
    public void elevatorMove()
    {
        elevatorMoveRequsted = true;
    }
}
