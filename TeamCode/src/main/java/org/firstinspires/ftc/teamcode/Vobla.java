package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Vobla {
    private Servo one = null;
    private Servo two = null;
    private boolean one_bool = false;
    private boolean two_bool = false;
    private static final double up = 0;
    private static final double down = 0;
    private static final double take = 0;
    private static final double push = 0;

    void change_high(){
        if (one_bool == true){
            one.setPosition(up);
        }
        else{
            one.setPosition(down);
        }
    }
    void change_push(){
        if (two_bool == true){
            two.setPosition(push);
        }
        else{
            two.setPosition(take);
        }
    }
    void up(){
        one.setPosition(up);
    }
    void down(){
        one.setPosition(down);
    }
    void take(){
        two.setPosition(take);
    }
    void push(){
        two.setPosition(push);
    }
}
