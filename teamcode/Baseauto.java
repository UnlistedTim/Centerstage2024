package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.IMU;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


public class Baseauto extends BaseClass {

    public Baseauto(LinearOpMode op, Pose2d p1) {
        super(op, p1);
    }

    public ElapsedTime runtimea = new ElapsedTime();







    void aouttake2r() {

        intake_roller.setPower(-0.9);

        outtake_move.setPosition(finalx);
        double dis = 300;
        double target = 240;// M 180
        int step = 0;
        if (baseblue) target = 240;

//        move(-0.26);
//        pause(40);
//        move(-0.3);
//        pause(30);
//        while (Op.opModeIsActive() && (dis > target)) {//to do&&!timer3(700)
//            dis = out_right_dis.getDistance(DistanceUnit.MM);
//            if(dis>1000) dis=target;
//            if (dis < 260&&step==0) {
//                move(-0.15);
//                outtake_cam.setPosition(out_cam[1]);
//                step=1;
//            }
//        }

        move(-0.5);
        pause(55);
        while ((dis > target)) {//to do&&!timer3(700)
            dis = out_right_dis.getDistance(DistanceUnit.MM);
            if (dis > 1000) dis = target;
            if (dis < 320 & step == 0) {
                // move(-0.15);
                outtake_cam.setPosition(out_cam[1]);
                step = 1;
            }

        }

        stop_drive();
        pause(150);
        drop_qty = 1;
        move(0.4);
        pause(150);


        stop_drive();
        if (cycle == 0) {
            // pause(50);
            finalxy();
            outtake_slide(finaly, out_speed[2]);
            outtake_move.setPosition(finalx);
            pause(160);
        }
        // if(cycle==0) pause(160);
        move(-0.4);
        pause(60);

        outtake_cam.setPosition(out_cam[0]);
        pause(90);

        stop_drive();
        pause(150);

        forward(0.8, 100);//0.3 200

        outtake_move.setPosition(out_move[3]);

        outtake_slide(out_height[10], out_speed[3]);
        stop_drive();
//        outtake_handle.setPosition(out_handle[5]);
        intake = true;
        cycle++;


//        pause(200);
//        drop_qty=1;
////        move(0.3);
////        pause(250);
//        move(0.5);
//        pause(150);
//
//            stop_drive();
//
//        if(cycle==0)
//        {
//            finalxy();
//            outtake_slide(finaly, out_speed[2]);
//            outtake_move.setPosition(finalx);
//            pause(200);
//        }
//            move(-0.3);
//            pause(100);
//        outtake_cam.setPosition(out_cam[0]);
//            pause(150);
//            stop_drive();
//            pause(200);
//
//        forward(0.5, 150);//0.3 200
//        outtake_move.setPosition(out_move[3]);
//        outtake_handle.setPosition(out_handle[5]);
//        outtake_slide(out_height[0], out_speed[2]);
//        stop_drive();
//        cycle++;

        intake_roller.setPower(0);
    }

    void aouttake2l() {

        intake_roller.setPower(-0.9);

        outtake_move.setPosition(finalx);
        int step = 0;
        double dis = 300;
        double target = 280;//240
        if (baseblue) target = 290;
        if (cycle == 0) target = 265;
//        move(-0.6);// -0.26
//        pause(30);
//        while ((dis > target)) {//to do&&!timer3(700)
//            dis = out_left_dis.getDistance(DistanceUnit.MM);
//            if(dis>1000) dis=target;
//            if (dis < 280&&step==0) {
//                move(-0.15);
//                outtake_cam.setPosition(out_cam[1]);
//                step=1;
//            }
//        }

        move(-0.5);
        pause(55);
        while ((dis > target)) {//to do&&!timer3(700)
            dis = out_left_dis.getDistance(DistanceUnit.MM);
            if (dis > 1000) dis = target;
            if (dis < 300 && step == 0) {
                // move(-0.15);
                outtake_cam.setPosition(out_cam[1]);
                step = 1;
            }

        }

        stop_drive();
        pause(150);
        drop_qty = 1;
        move(0.4);
        pause(150);


        // pause(50);

        stop_drive();
        if (cycle == 0) {
            pause(50);
            finalxy();
            outtake_slide(finaly, out_speed[2]);
            outtake_move.setPosition(finalx);
            // pause(100);
        }

        if (cycle == 0) pause(160);
        move(-0.4);
        pause(60);
        if (cycle == 0) pause(30);

        outtake_cam.setPosition(out_cam[0]);
        pause(90);

        stop_drive();
        pause(150);

        forward(0.8, 100);//0.3 200


        //  outtake_handle.setPosition(out_handle[5]);
        outtake_move.setPosition(out_move[3]);
        outtake_slide(out_height[10], out_speed[2]);
        stop_drive();
        intake = true;
        cycle++;

        intake_roller.setPower(0);
    }


    public void afinalxy() {
        drop_qty = 2;


        if (index == 1) {
            input1x = 5;
            input1y = cycle + 1;
            //if(cycle==2) input1x=6;
            finalxy();

        }  //target y=2,x=5;

        else {


            input1x = 3;
            input1y = cycle + 1;
            // if(!baseblue) {input1x=2;}
            finalxy();


        }


        if (cycle == 2) finaly = finaly + layer_rate + 50;


    }


    public void ain() {
        double angle_gap, yaw, target_dis = -23;//24.5
        double x1, xtarget = 0, xgap, yrange, dis, distar = 38;
        double max_strafe = 0.80;
        int dir = 1;
        double strafe_gain = 0.1;
        int step = 0;
        if (volthi) max_strafe = 0.78;
        boolean stop = false;
        if (baseblue) {
            dir = -1;
            target_dis = -23;
        }//-24.5
        //  if (out_handling) {step = 1;out_handling = false;}
        timer3(0);
        updatePoseEstimate();


        while (!stop && Op.opModeIsActive()) {// todo &&!timer3(1200)

            updatePoseEstimate();
            x1 = 0;
            xgap = xtarget - x1;
            if (Math.abs(xgap) < 10) {
                if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
                else dis = side_right_dis.getDistance(DistanceUnit.INCH);

                if (Math.abs(dis - distar) < 2) break;
            }
            // if(Math.abs(xgap)<1) break;

            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = -Range.clip(angle_gap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange = dir * Range.clip(xgap * strafe_gain, -max_strafe, max_strafe);
            moveRobot(0, yrange, yaw, -1);

            if (step == 0) {
                if (outtake_slider.getCurrentPosition() > out_height[2]) {
                    outtake_handle.setPosition(out_handle[0]);// todo need adjust;
                    timer2(0);
                    step = 1;
                }
            }
            if (step == 1) {
                if (timer2(400)) {
                    outtake_move.setPosition(finalx);
                    step = 2;
                }
            }


        }
        stop_drive();
    }

    public void aout() {
        double angle_gap, yaw;//24.5
        double x1, xgap, yrange, xtargetabs = 27;//27
        double max_strafe = 0.8, astrafe_gain = 0.25, dis, distar = 24;
        ax = 0;
        ay = 0;
        int dir = 1;
        if (volthi) max_strafe = 0.78;
        if (cycle == finalcycle) {
            outtake_handle.setPosition(out_handle[5]);
            intake_handle(0);

            outtake_slide(out_height[0], out_speed[2]);

        }
        boolean stop = false;
        if (baseblue) {
            dir = -1;
            xtargetabs = 30.5;
        }
        // if(baseblue&&cycle==2)xtargetabs= 29;
        if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
        else dis = side_right_dis.getDistance(DistanceUnit.INCH);
        if (dis < 60) ax = dis - distar;
        else ax = -3;//0
        startPoseA = new Pose2d(ax, ay, Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES))));

        intake_roller.setPower(0.95);

        if (cycle == 3) {
            intake_handle(0);
            xtargetabs = 22;
        }
        while (!stop && Op.opModeIsActive()) {// todo &&!timer3(1200)
            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            updatePoseEstimate();
            x1 = 0;
            xgap = xtargetabs - x1;
            if (Math.abs(xgap) < 1) {
                break;
            }
            yaw = -Range.clip(angle_gap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange = dir * Range.clip(xgap * astrafe_gain, -max_strafe, max_strafe);
            moveRobot(0, yrange, yaw, -1);
        }
        stop_drive();
    }


    public void bstrafefromwall(boolean intake) {
        double angle_gap, yaw, target_dis = 22.5, target_disa = 22.5, target_disb = 21, target_dis1 = 22.5, target_dis2 = 17.5, target_dis3 = 17.8, target_dis4 = 12.2;//target for intake  // dist2 16.5
        double x1, xtarget = 0, xgap, ygap = 0, yrange, dis, strafe_gain = 0.1, max_strafe = 0.85, xrange, max_move = 0.3, move_gain = 0.05, ytarget = 35;
// target-is=23.5
        int dir = 1, step = 5;
        if (volthi) max_strafe = 0.82;
        boolean stop = false;
        if (!intake) {
            step = 0;
            if (out_handling) {
                step = 1;
                out_handling = false;
            }
            if (!baseblue) {
                if ((cycle == 0) && (index == 1)) target_dis = target_dis1;
                else target_dis = target_dis2;
            }
            if (baseblue) {
                if ((cycle == 0) && (index == 3)) target_dis = target_dis3;
                else target_dis = target_dis4;
            }
//            if(cycle==0 &&outtake_1_delay>0) target_dis=target_dis-1.5;
//            if(cycle==1 &&outtake_2_delay>0) target_dis=target_dis-1.5;
        } else {
            if (baseblue) target_dis = target_disb;
        }

        if (baseblue) {
            dir = -1;
        }//-24.5

        timer3(0);
        updatePoseEstimate();
        if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
        else dis = side_right_dis.getDistance(DistanceUnit.INCH);
        xtarget = 0;//inch
        while (!stop && Op.opModeIsActive()) {// todo &&!timer3(1200)

            updatePoseEstimate();
            //  x1=getPoseEstimate().getX();
            x1 = 0;
            xgap = xtarget - x1;
            //if(intake)  ygap=grab_dis.getDistance(DistanceUnit.CM)-ytarget;
//            if(cycle==0) {
//                if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
//                else dis = side_right_dis.getDistance(DistanceUnit.INCH);
//                if (dis>distar) break;
//            }

            if (Math.abs(xgap) < 1) break;

            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = -Range.clip(angle_gap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange = dir * Range.clip(xgap * strafe_gain, -max_strafe, max_strafe);

            // if(intake && Math.abs(xgap)>10)  xrange=dir*Range.clip(ygap * move_gain, -max_move, max_move); else xrange=0;
            moveRobot(0, yrange, yaw, -1);

            if (step == 0) {
                if (outtake_slider.getCurrentPosition() > out_height[2]) {
                    outtake_handle.setPosition(out_handle[0]);// todo need adjust;
                    timer2(0);
                    step = 1;
                }
            }
            if (step == 1) {
                if (timer2(400)) {
                    outtake_move.setPosition(finalx);
                    step = 2;
                }
            }
            if (step == 5) {
                if (outtake_slider.getCurrentPosition() > (out_height[2] - 14)) {
                    outtake_handle.setPosition(out_handle[4]);
                    timer2(0);
                    step = 2;
                }
            }

        }
        stop_drive();
    }


    public void bstrafetowall(boolean intake) {
        double angle_gap, yaw, target_dis = -22, strafe_gain = 0.15;//0.12
        double x1, xtarget, xgap, yrange, dis;
        double max_strafe = 0.95;
        int dir = 1;
//        int  step = 0;
        intake_roller.setPower(0.95);
        //  if(volthi) max_strafe=0.78;
//        if(intake) step=5;
        boolean stop = false;
        if (baseblue) {
            dir = -1;
        }//-24.5
        if (baseblue && intake) target_dis = target_dis - 3;
        if (intake && (endgame - runtime.seconds()) < 7) {
            timeend = true;
            finalcycle = cycle;
        }
        if (cycle == finalcycle) {
            outtake_slide(out_height[0], out_speed[2]);
            walltarget = walltarget + 5;
            intake_handle(0);
            outtake_handle.setPosition(out_handle[5]);
        }
        timer3(0);
        updatePoseEstimate();

        while (!stop && Op.opModeIsActive()) {// todo &&!timer3(1200)
            updatePoseEstimate();

            xgap = 0;
            if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
            else dis = side_right_dis.getDistance(DistanceUnit.INCH);
            if (dis < (walltarget + 5) || (Math.abs(xgap) < 2.5)) break;
            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = -Range.clip(angle_gap * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            yrange = dir * Range.clip(xgap * strafe_gain, -max_strafe, max_strafe);
            moveRobot(0, yrange, yaw, -1);
//            if(step==0&&(Math.abs(xgap)<14)) {
//              outtake_slide(out_height[0],out_speed[3]);
//               // intake_roller.setPower(-0.9);
//                step = 1;
//            }

        }
        stop_drive();
        if (!intake) outtake_slide(out_height[0], out_speed[3]);


    }


    void astraight(boolean intake) {
        double angle_gap, yaw, yaw_gain = 0.04, max_turn = 0.35;//0.035,0.3
        double ygap, target_dis1 = 83, target_dis2 = -79, ytarget, yrange = 0, move_gain = 0.04, max_move = 0.85;//-78
        double y1 = 0, target_dis = 0;
        boolean stop = false;
        int dir = 1, step = 0;
        if (baseblue) dir = -1;
        if (volthi) max_move = 0.78;

        if (intake) {

            if (baseblue) target_dis = -target_dis1;
            if (!baseblue) {
                target_dis = target_dis1;
            }
            outtake_handle.setPosition(out_handle[5]);
            outtake_slide(out_height[0], out_speed[3]);
            step = 5;
        }
        if (!intake) {
            // intake_roller.setPower(0.95);

            bfinalxy();
            if (!baseblue) {
                target_dis = target_dis2;
            }
            if (baseblue) {
                target_dis = -target_dis2;
            }
            //  step = 0;

        }
        timer3(0);
        updatePoseEstimate();

        while (!stop && Op.opModeIsActive()) {//!timer3(time)&&
            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = Range.clip(angle_gap * yaw_gain, -max_turn, max_turn);
            updatePoseEstimate();

            // ygap=ytargetabs-y1;
            ygap = 0;
            if (Math.abs(ygap) < 1) {

                break;
            }


            yrange = dir * Range.clip(ygap * move_gain, -max_move, max_move);
            moveRobot(yrange, 0, yaw, 1);
//            if (step == 0) {
//
//                if(timer3(100)) {
//                    outtake_slide(out_height[0], out_speed[1]);
//                     step=1;
//                }
//            }

            if (step == 0 && timer3(250)) {
                outtake_slide(out_height[0], out_speed[3]);
                step = 1;
            }

            if (step == 1) {
                if (outtake_slider.getCurrentPosition() < (out_height[0] + 10) || timer3(1000)) {//timer3 1000

                    outtake_cam.setPosition(out_cam[2]);
                    timer5(0);
                    intake_roller.setPower(0);
                    //intake_grab(0);
                    step = 2;

                }
            }
            if (step == 2) {
                if (Math.abs(ygap) < 26 && timer5(450)) {

                    outtake_slide(finaly, out_speed[3]);

                    step = 3;

                }
            }
//            if (step == 3) {
//
//                if (outtake_slider.getCurrentPosition() > out_height[2]) {
//                    outtake_handle.setPosition(out_handle[0]);
//                    timer2(0);
//                    out_handling = true;
//                    intake_grab(0);
//                    step = 4;
//                }
//
//            }


            if (step == 5) {//&& timer3(1400) -32+62=30

                if (Math.abs(ygap) < 24) {
                    outtake_slide(out_height[2], out_speed[2]);
                    intake_handle(2);
                    intake_grab(1);
                    step = 6;
                }
            }
        }
        stop_drive();
        if (intake) {
            pause(200);
        }
        intake_roller.setPower(0);

    }

    void bfinalxy() {
        if ((!baseblue && wall_route) || (baseblue && !wall_route)) {
            if (cycle == 0) {
                input1x = 1;
                input1y = 1;
                input2x = input1x + 3;
                input2y = input1y;
                if (index == 3) {
                    input1x = 3;
                    input2x = input1x - 2;
                }
            } else {
                if (index < 3) input1x = 4;
                else input1x = 1;
                input1y = cycle + 1;
                input2y = input1y;
            }
        } else {
            if (cycle == 0) {
                input1x = 6;
                input1y = 1;
                input2x = input1x - 3;
                input2y = input1y;
                if (index == 1) {
                    input1x = 4;
                    input2x = input1x + 2;
                }
            } else {
                if (index > 1) input1x = 4;
                else input1x = 6;
                if (cycle == 1) input1y = 3;
                if (cycle == 2 && index == 1) {
                    input1y = 4;
                    input1x = input1x + 1;
                }
            }

        }

        finalxy();
    }


    void bstraight(boolean intake) {
        double angle_gap, yaw, yaw_gain = 0.04, max_turn = 0.2, dis;//0.035,0.3
        double ygap, xgap, target_dis1 = 75, target_dis3 = 75, target_dis2 = 79, ytarget, yrange = 0, move_gain = 0.04, max_move = 0.95;//-78
        double target_dis = 0, xrange = 0, strafe_gain = 0.1, max_strafe = 0.2, grab_dis_tar = 350;
        boolean stop = false;
        int dir = 1, step = 0;
        if (baseblue) dir = -1;
        if (volthi) max_move = 0.93;
        if (intake) {
            if (baseblue) target_dis = -(target_dis3);
            if (!baseblue) {
                target_dis = target_dis1;
            }
            step = 5;
        }
        if (!intake) {
            if (cycle == 0 && (outtake_1_delay > 0)) {
                target_dis2 += 3.0;
            }
            if (cycle == 1 && (outtake_2_delay > 0)) {
                target_dis2 += 3.0;
            }
            if (!baseblue) {
                target_dis = -target_dis2;
            }
            if (baseblue) {
                target_dis = target_dis2;
            }
            intake_roller.setPower(0);
            bfinalxy();
        }

        timer3(0);
        updatePoseEstimate();

        while (!stop && Op.opModeIsActive()) {//!timer3(time)&&
            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = Range.clip(angle_gap * yaw_gain, -max_turn, max_turn);
            updatePoseEstimate();


            // if(intake&&Math.abs(ygap)<10 &&grab_dis.getDistance(DistanceUnit.MM)<grab_dis_tar) break;
            ygap = 0;
            if (Math.abs(ygap) < 2) {
                break;
            }

            if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
            else dis = side_right_dis.getDistance(DistanceUnit.INCH);
            if (dis > 60) dis = walltarget;
            xgap = -(walltarget - dis);
            yrange = dir * Range.clip(ygap * move_gain, -max_move, max_move);
            xrange = dir * Range.clip(xgap * strafe_gain, -max_strafe, max_strafe);
            moveRobot(yrange, xrange, yaw, 1);

            if (step == 0) {

                if (outtake_slider.getCurrentPosition() < 8 || timer3(1000)) {
                    outtake_cam.setPosition(out_cam[2]);
                    timer4(0);

                    //   intake_roller.setPower(0);
                    step = 1;
                }
            }


            if (step == 1) {
                if (Math.abs(ygap) < 28 && timer4(500)) {
                    outtake_slide(finaly, out_speed[3]);
                    step = 2;
                }
            }
            if (step == 2) {
                if (outtake_slider.getCurrentPosition() > out_height[2]) {
                    outtake_handle.setPosition(out_handle[0]);
                    timer2(0);
                    out_handling = true;
                    intake_roller.setPower(0);
                    step = 3;
                }
            }

            if (step == 5) {//&& timer3(1400) -32+62=30
                if (Math.abs(ygap) < 24) {
                    outtake_slide(out_height[2], out_speed[2]);
                    intake_handle(2);
                    intake_grab(1);
                    step = 6;
                    intake_roller.setPower(0);
                }
            }
        }
        stop_drive();
        intake_roller.setPower(0);

    }

    void agrab() {
        outtake_handle.setPosition(out_handle[4]);
        double dis = 500;
        bot_fla = false;
        top_fla = false;
        boolean stop = false;
        int step = 0;
        if (cycle > 1) step = 5;
        if (volthi) move(0.24);
        else move(0.26);
        while (dis > 80 && Op.opModeIsActive()) {//80
            dis = grab_dis.getDistance(DistanceUnit.MM);
            if (dis < 160) {
                move(0.12);//0.14
            }
        }
        move(0.10);//0.1
        intake_handle(1);
        intake_grab(0);
        intake_roller.setPower(0.8);
        //   outtake_handle.setPosition(out_handle[4]);
        pause(350);//390
        intake_grab(1);
        outtake_slide(out_height[1], out_speed[1]);
        pause(320);//390
        forward(-0.18, 160);
        move(0.15);

        pause(90);
        move(0.10);

        intake_grab(0);
        if (bot_color.red() + bot_color.blue() + bot_color.green() > 2500) bot_fla = true;
        pause(400);//400
        timer2(0);
        intake_grab(4);

        while (!(timer2(850)) && !stop && Op.opModeIsActive()) {// todo 80000 only for debugging, need adjust the time.
            if (bot_fla) {
                if (top_color.red() + top_color.blue() + top_color.green() > 2500) top_fla = true;
            } else if (bot_color.red() + bot_color.blue() + bot_color.green() > 2500) {
                pause(150);
                bot_fla = true;
            }
            if (step == 5) {
                move(-0.15);
                step = 6;
            }
            if (step == 6) {
                if (top_fla) break;
            }
            if (step == 0 && timer2(100)) {
                move(-0.15);
                step = 1;
            }
            if (step == 1 && timer2(250)) {
                stop_drive();
                step = 2;
            }
            if (step == 2 && timer2(450)) {
                move(-0.11);
                step = 3;
            }//500
            if (step == 3 && top_fla) stop = true;

        }

        if (top_fla) outtake_slide(out_height[0], out_speed[1]);
        dis = grab_dis.getDistance(DistanceUnit.CM);
        if (dis > 100) dis = 20;
        delta_dis = (dis - 10) * 0.39;
        if (cycle < 2) {
            move((-0.1));
            pause(200);
        }//-0.1
        move(-0.5);
        pause(100);
        afinalxy();
        intake_handle(4);
        intake_grab(1);

    }


    public void aouttake_lift() {
        outtake_handle.setPosition(out_handle[4] + 0.015);
        startSlide(out_height[3] - 120, out_speed[2], true);

    }

    public void bouttake_lift() {
        outtake_handle.setPosition(out_handle[4] + 0.015);
        startSlide(out_height[1], out_speed[2], true);

    }

    public void aouttake_drop(int astep) {
        //pause(400);
        if (astep == 0) outtake_handle.setPosition(out_handle[4]);
        if (astep == 1) outtake_slide(out_height[0], out_speed[1]);
    }

    void bgrab() {
        //intake_grab(1);
        double target = 130;//75
        double dis = 500;
        if (baseblue && wall_route) target = 120;//115
        if (!wall_route) target = 140;
        if (baseblue && wall_route && cycle > 0) target = target - 15;//need check

        if (grab_counter > 0) target = target + 25;

        move(0.5);
        if (bot_color.red() + bot_color.blue() + bot_color.green() > 2500) {
            half_full = true;
        }

        pause(30);
        move(0.3);
        if (!wall_route) pause(20);
        while (dis > target && Op.opModeIsActive()) {//75
            dis = grab_dis.getDistance(DistanceUnit.MM);
//            if (dis < 150) {
//                move(0.14);//0.14
//            }
        }
        stop_drive();
        intake_handle(1);
        intake_grab(0);
        intake_roller.setPower(0.88);
        pause(350);//390
        outtake_handle.setPosition(out_handle[4]);


        if (cycle > 0 && !half_full) {
            intake_grab(3);
            pause(300);//400
            //  if(cycle>1) pause()
            intake_grab(0);
            pause(250);//390
            if (cycle > 1) pause(50);

        }
        intake_grab(1);
        pause(300);//400}
//        move(-0.18);
//        pause(250); // 200
        outtake_slide(out_height[1], out_speed[3]);
        if (delay[grab_counter] > 0) pause(delay[grab_counter]);
        if (wall_route) {
            move(-0.9);
            pause(100);
        } else {
            intake_roller.setPower(0.95);
            move(-0.3);
            pause(400);
        }
        intake = false;
        intake_handle(3);//fold
        intake_grab(2);//fold
        stop_drive();
        drop_qty = 2;
        grab_counter++;
        intake_roller.setPower(0.95);
        half_full = false;
    }


    public void buildTrajectories() {
// middleroute

        Pose2d r_r_1_1 = new Pose2d(23.5, 6, Math.toRadians(50));//44
        Pose2d r_r_2_1 = new Pose2d(28, -3, Math.toRadians(0));
        Pose2d r_r_3_1 = new Pose2d(19.5, -9.5, Math.toRadians(-1));
        Pose2d r_r_0_2 = new Pose2d(25, -32, Math.toRadians(90));//26


        Pose2d b_l_1_1 = new Pose2d(19, 8, Math.toRadians(0));
        Pose2d b_l_2_1 = new Pose2d(26, 1.5, Math.toRadians(0));
        Pose2d b_l_3_1 = new Pose2d(23, -7.5, Math.toRadians(-46));
        Pose2d b_l_0_2 = new Pose2d(27.5, 29, Math.toRadians(-89));


        Pose2d r_l_1_1 = new Pose2d(28, 8, Math.toRadians(0));
        Pose2d r_l_2_1 = new Pose2d(29, 2.5, Math.toRadians(0));
        Pose2d r_l_3_1 = new Pose2d(25, -9, Math.toRadians(-46));
        Pose2d r_l_0_2 = new Pose2d(24.5, 13, Math.toRadians(92));//25


        Pose2d b_r_1_1 = new Pose2d(22.5, 6.5, Math.toRadians(50));//44
        Pose2d b_r_2_1 = new Pose2d(29, -3.0, Math.toRadians(0));
        Pose2d b_r_3_1 = new Pose2d(28.5, -11, Math.toRadians(0));
        Pose2d b_r_0_2 = new Pose2d(25.5, -13, Math.toRadians(-94));//26
        Pose2d b_r_1_2 = new Pose2d(25, -13, Math.toRadians(-96));//26


    }
}
