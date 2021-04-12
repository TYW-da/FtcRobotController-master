package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name="TeleOP", group="Iterative Opmode")

public class Teleop_Id extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx fl1 = null;
    private DcMotorEx fr1 = null;
    private DcMotorEx fl2 = null;
    private DcMotorEx fr2 = null;
    private DcMotorEx fcov = null;
    private DcMotorEx fstv = null;
    private Servo finger = null;
    private Servo fsrv = null;
    private Servo fcol = null;

    private DcMotorEx EncX = null;
    private DcMotorEx EncY = null;

    private final ElapsedTime timer1 = new ElapsedTime();
    private final ElapsedTime timerpad1 = new ElapsedTime();
    private final ElapsedTime timerpad2 = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        fr1 = hardwareMap.get(DcMotorEx.class, "R1");
        fr2 = hardwareMap.get(DcMotorEx.class, "R2");
        fl1 = hardwareMap.get(DcMotorEx.class, "L1");
        fl2 = hardwareMap.get(DcMotorEx.class, "L2");
        fstv = hardwareMap.get(DcMotorEx.class, "Shooter");
        fcov = hardwareMap.get(DcMotorEx.class, "Conv");
        finger = hardwareMap.get(Servo.class, "Finger");
        fsrv = hardwareMap.get(Servo.class, "Fix");
        fcol = hardwareMap.get(Servo.class, "Col");
        EncX = hardwareMap.get(DcMotorEx.class, "EncX");
        EncY = hardwareMap.get(DcMotorEx.class, "EncY");
        fr1.setDirection(DcMotorEx.Direction.FORWARD);
        fl1.setDirection(DcMotorEx.Direction.REVERSE);
        fr2.setDirection(DcMotorEx.Direction.REVERSE);
        fl2.setDirection(DcMotorEx.Direction.FORWARD);
        fstv.setDirection(DcMotorEx.Direction.REVERSE);
        fcov.setDirection(DcMotorEx.Direction.FORWARD);
        fstv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        initGyro();
    }
    @Override
    public void init_loop() {
        int encx = 0;
        int ency = 0;
    }
    @Override
    public void start() {
        runtime.reset();
        fcol.setPosition(Range.clip(0.0,0.0, 1.0));

    }
    boolean button_b_last = false;
    boolean button_x_last = false;
    boolean button_y_last = false;
    boolean button_a_last = false;
    int b = 0;
    int x_1 = 0;
    int covup = 0;
    int covdown = 0;
    double convus = 0.0;
    int c = 0;
    double encx = 0;
    double ency = 0;

    BNO055IMU gyro = null;

    void initGyro(){
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(new BNO055IMU.Parameters());
    }

    double getGyroHeading(){
        return -gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    double ak(){
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.get("Control Hub");
        double akk=12/voltageSensor.getVoltage();
        return akk;
    }
    void sleep_m(double time){
        timer1.reset();
        while(timer1.milliseconds() < time){}
    }
    void shoot() {
        double power= 1440;
        fstv.setVelocityPIDFCoefficients(55,0.2, 0, 16.11*ak());
        fstv.setVelocity(power);

    }
    void motors(double x1, double y1, double x2){
        double l1 = Range.clip(-y1+x1-x2, -1.0, 1.0);
        double r1 = Range.clip(-y1-x1+x2, -1.0, 1.0) ;
        double l2 = Range.clip(y1+x1+x2, -1.0, 1.0) ;
        double r2 = Range.clip(y1-x1-x2, -1.0, 1.0) ;
        fl1.setPower(l1);
        fl2.setPower(l2);
        fr1.setPower(r1);
        fr2.setPower(r2);
    }

    @Override
    public void loop() {
       if(c == 0){
            encx = EncX.getCurrentPosition();
            ency = EncY.getCurrentPosition();
       }
        c++;

        double l1 = 0.0;
        double r1 = 0.0;
        double l2 = 0.0;
        double r2 = 0.0;

        double y1 = gamepad1.left_stick_y;
        double x1 = -gamepad1.left_stick_x;
        double x2 = -gamepad1.right_stick_x;

        if(gamepad1.dpad_up){
            fcol.setPosition(Range.clip(0.3,0.0, 1.0));
        }
        else{
            if(gamepad1.dpad_down){
                fcol.setPosition(Range.clip(0.8,0.0, 1.0));
            }
        }

        if(gamepad1.x && (gamepad1.x != button_x_last)){
            x_1++;
            if(x_1%2 != 0) {
                fsrv.setPosition(Range.clip(0.5, 0.0, 1.0));
            }
            else{
                fsrv.setPosition(Range.clip(0.0, 0.0, 1.0));
            }
        }
        button_x_last = gamepad1.x;

        if(gamepad1.b && gamepad1.b != button_b_last){
            if(b%2 != 0) {
                shoot();
            }
            if(b%2 == 0){
                fstv.setPower(0);
            }
            b++;
        }
        button_b_last = gamepad1.b;

        if(gamepad1.right_bumper){
            finger.setPosition(-0.3);
        }
        else finger.setPosition(0.3);

        if(gamepad1.right_trigger > 0.1){
                finger.setPosition(-0.3);
                sleep_m(200);
                finger.setPosition(0.3);
        }
        else finger.setPosition(0.3);
        if(gamepad1.y && gamepad1.y != button_y_last){
            covup++;
            if(covup%2 != 0) {
                convus = 1;
            }
            else{
                convus = 0;
            }
        }
        else {
            if(gamepad1.a && gamepad1.a != button_a_last) {
                covdown++;
                if (covdown % 2 == 0) {
                    convus = -1;
                } else {
                    convus = 0;
                }
            }
        }
        button_a_last = gamepad1.a;
        button_y_last = gamepad1.y;
        fcov.setPower(Range.clip(convus, -1.0, 1.0));

        motors(x1, y1, x2);

    }

    @Override
    public void stop() {
    }

}