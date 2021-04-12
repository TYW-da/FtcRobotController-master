package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Blue_right", group="Iterative Opmode")

public class Blue_right extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx fl1 = null;
    private DcMotorEx fr1 = null;
    private DcMotorEx fl2 = null;
    private DcMotorEx fr2 = null;
    private DcMotorEx fstv = null;
    private DcMotorEx EncX = null;
    private DcMotorEx EncY = null;
    private Servo finger = null;
    private DcMotorEx fcov = null;
    private final ElapsedTime timer1 = new ElapsedTime();
    private Servo fsrv = null;
    private Servo fcol = null;
    private OpenCVNodeWebcam openCVNodeWebcam = new OpenCVNodeWebcam();

    void motors(double y1, double x1, double x2) {
        double l1 = Range.clip(-y1 + x1 - x2, -1.0, 1.0);
        double r1 = Range.clip(-y1 - x1 + x2, -1.0, 1.0);
        double l2 = Range.clip(y1 + x1 + x2, -1.0, 1.0);
        double r2 = Range.clip(y1 - x1 - x2, -1.0, 1.0);
        fl1.setPower(l1);
        fl2.setPower(l2);
        fr1.setPower(r1);

        fr2.setPower(r2);
    }
    void vobla(){
        fcol.setPosition(0.8);
        sleep_m(1000);
        fsrv.setPosition(0.8);
        sleep_m(500);
    }

    BNO055IMU gyro = null;

    void initGyro() {
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(new BNO055IMU.Parameters());
    }

    void f_angle(double angle, double x,double y) {
        double ang = 0;
        double speed_ang = 0;
        ang = getGyroHeading();
        speed_ang = Math.abs(ang - angle) * 0.01;
        if (speed_ang < 0.15) speed_ang = 0.15;
        speed_ang = Math.abs(ang - angle) * 0.06;
        if (ang < angle+2) motors(y, x, speed_ang);
        else if (ang > angle-2) motors(y, x, -speed_ang);
        else motors(y,x,0);

    }

    double ak(){
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.get("Control Hub");
        double akk=12/voltageSensor.getVoltage();       //12/13
        return akk;
    }
    double six() {
        double sx = 0;
        double vel=EncX.getVelocity();
        if (vel==0) vel=1;
        else vel=EncY.getVelocity();
        sx = 0.048/8192 * 3.14  * EncX.getCurrentPosition();
        return sx;
    }
    double siy() {
        double sy = 0;
        double vel=EncY.getVelocity();
        if (vel==0) vel=1;
        else vel=EncY.getVelocity();
        sy = 0.048/8192 * 3.14  * EncY.getCurrentPosition();
        return sy;
    }
    void f_x(double x, double angle,boolean brash){
        double speed_x = 0;
        double plavnostx = 4;
        double error=0;
        while(Math.abs(x - six()) > 0.1 && opModeIsActive()) {
            error = x - six();
            speed_x = -(error * plavnostx);
            if (speed_x>0 && speed_x<0.3) speed_x = 0.3;
            if (speed_x<0 && speed_x>-0.3) speed_x = -0.3;
            motors(0,speed_x,0);
            f_angle(angle,speed_x,0);
            if (brash == true) fcov.setPower(1);
            else fcov.setPower(0);
        }
        timer1.reset();
        while(timer1.milliseconds() < 400 && opModeIsActive()){
            error = x-six();
            speed_x = -(error * plavnostx*2);
            if (speed_x>0 && speed_x<0.3) speed_x = 0.3;
            if (speed_x<0 && speed_x>-0.3) speed_x = -0.3;
            motors(0,speed_x,0);
            f_angle(angle,speed_x,0);
            if (brash == true) fcov.setPower(1);
            else fcov.setPower(0);
        }
        motors(0,0,0);

    }
    void f_y(double y, double angle,boolean brash){
        double speed_y = 0;
        double plavnosty = 2.5;
        double error=0;
        while(Math.abs(y - siy()) > 0.1 && opModeIsActive()) {
            error = y-siy();
            speed_y = -(error * plavnosty);
            if (speed_y>0 && speed_y<0.3) speed_y = 0.3;
            if (speed_y<0 && speed_y>-0.3) speed_y = -0.3;
            motors(speed_y,0,0);
            f_angle(angle,0,speed_y);
            if (brash) fcov.setPower(1);
            else fcov.setPower(0);
        }
        timer1.reset();
        while(timer1.milliseconds() < 400 && opModeIsActive()){
            error = y-siy();
            speed_y = -(error * plavnosty*2);
            if (speed_y>0 && speed_y<0.15) speed_y = 0.2;
            if (speed_y<0 && speed_y>-0.15) speed_y = -0.2;
            motors(speed_y,0,0);
            f_angle(angle,0,speed_y);
            if (brash == true) fcov.setPower(1);
            else fcov.setPower(0);
        }
        motors(0,0,0);
    }
    double getGyroHeading() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    void teddy(double x,double y,double angle,boolean brash){

        f_x(x, angle,brash);
        f_y(y, angle,brash);

    }
    void cam_shooter() {
        double velocity= -1450;
        fstv.setVelocityPIDFCoefficients(55,0.2, 0, 16.11*ak());
        timer1.reset();
        while (timer1.milliseconds() < 1000 && opModeIsActive()) {
            fstv.setVelocity(velocity);
        }
        fstv.setVelocity(velocity);
        finger.setPosition(-0.3);
        sleep_m(100);
        finger.setPosition(0.3);
        for (int i = 0; i < 4; i++) {
            timer1.reset();
            while (timer1.milliseconds() < 200 && opModeIsActive()) {
                fstv.setVelocity(velocity);
            }
            fstv.setVelocity(velocity);
            finger.setPosition(-0.3);
            sleep_m(100);
            finger.setPosition(0.3);
        }
        telemetry.update();
        finger.setPosition(0.3);
        fstv.setVelocity(0);
    }
    void power_shots(){
        double velocity= -1330;
        fstv.setVelocityPIDFCoefficients(55,0.2, 0, 16.11*ak());
        timer1.reset();
        while (timer1.milliseconds() < 1000 && opModeIsActive()) {
            fstv.setVelocity(velocity);
        }
        fstv.setVelocity(velocity);
        finger.setPosition(-0.3);
        sleep_m(100);
        finger.setPosition(0.3);
    }
    void sleep_m(double time){
        timer1.reset();
        while(timer1.milliseconds() < time && opModeIsActive()){}
    }

    @Override
    public void runOpMode() {
        initGyro();
        openCVNodeWebcam.initialize(this);
        //    col = Camera.GetCircles();
        waitForStart();
        telemetry.addData("Status", "Initialized");
        //      telemetry.addData("Circles: ", col);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //Motors
        fr1 = hardwareMap.get(DcMotorEx.class, "R1");
        fr2 = hardwareMap.get(DcMotorEx.class, "R2");
        fl1 = hardwareMap.get(DcMotorEx.class, "L1");
        fl2 = hardwareMap.get(DcMotorEx.class, "L2");
        fstv = hardwareMap.get(DcMotorEx.class, "Shooter");
        fcov = hardwareMap.get(DcMotorEx.class, "Conv");

        //Encoders
        EncX = hardwareMap.get(DcMotorEx.class, "EncY");
        EncY = hardwareMap.get(DcMotorEx.class, "EncX");
        //Servo
        fsrv = hardwareMap.get(Servo.class, "Fix");
        fcol = hardwareMap.get(Servo.class, "Col");
        finger = hardwareMap.get(Servo.class, "Finger");
        double x_1 = 0;
        double y_1 = 0;
        fr1.setDirection(DcMotorEx.Direction.FORWARD);
        fl1.setDirection(DcMotorEx.Direction.REVERSE);
        fr2.setDirection(DcMotorEx.Direction.REVERSE);
        fl2.setDirection(DcMotorEx.Direction.FORWARD);
        fstv.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fstv.setDirection(DcMotorSimple.Direction.FORWARD);
        EncX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EncX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EncY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fcov.setDirection(DcMotorEx.Direction.FORWARD);
        fcov.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        runtime.reset();
        fcol.setPosition(0);
        fsrv.setPosition(0);

        switch (openCVNodeWebcam.retrieveResult()){
            case FOUR:
                teddy(0,1.15,0,false);
                teddy(0,1.15,13,false);
                cam_shooter();
                teddy(0,1.5,0,false);
                teddy(-0.4,1.5,0,true);
                teddy(-0.4,0.5,0,true);
                teddy(-0.4,1.15,0,true);
                cam_shooter();
                teddy(0.3,2.2,0,false);
                vobla();
                teddy(-0.8,1.7,0,false);

                break;
            case ONE:
                teddy(0,1.15,0,false);
                teddy(0,1.15,13,false);
                cam_shooter();
                teddy(0,1.5,0,false);
                teddy(-0.43,1.5,0,true);
                teddy(-0.43,0.2,0,true);
                teddy(-0.5,1.15,0,true);
                cam_shooter();
                teddy(-0.5,1.8,0,false);
                vobla();
                teddy(-0.8,1.7,0,false);
                break;
            default:
            case ZERO:
                teddy(0,1.15,0,false);
                teddy(0,1.15,13,false);
                cam_shooter();
                teddy(0.4,1.15,0,false);
                vobla();
                teddy(-0.8,1.7,0,false);
                break;
        }
    }
}