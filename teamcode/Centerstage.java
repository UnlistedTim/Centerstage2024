package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;


@TeleOp(name = "CenterStageA", group = "A")
public class Centerstage extends LinearOpMode {
    Baseauto rbg;


    boolean robo_drive = true;
    boolean end_flag = false;
    public enum State {
        INTAKE,
        INTAKEDONE,
        LIFT,
        OUTTAKE,
        OUTTAKEDONE,
    }

    State state = State.INTAKE;



    @Override
    public void runOpMode() {


        telemetry.addLine("Wait ! Initializing............. ");
        telemetry.update();
        Pose2d p1 =new Pose2d(0,0,0);

        rbg = new Baseauto(this, p1);

        while (!rbg.init(false) && !isStopRequested())

        {


        }
      //  if(rbg.april_ready) telemetry.addLine("April tag setup ready!");
        telemetry.addLine("For practice only:");
        telemetry.addLine(" BLUE:Driver --Cross ");
        telemetry.addLine(" RED: Driver---Circle");
        telemetry.addLine("Press  driver: ∆  to reset the IMU angle to Zero " );

        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.triangle) {
                rbg.imu.initialize(rbg.imuparameters);
                rbg.imu.resetYaw();
                sleep(1000);
                break;

            }
            if (gamepad1.cross) {

                rbg.baseblue=true;
                rbg.base_align_angle=-90;
                rbg.base_apr_id=1;

            }
            if ( gamepad1.circle) {

                rbg.baseblue=false;
                rbg.base_align_angle=90;
                rbg.base_apr_id=4;

            }

        }

        telemetry.addLine("Press Start Now!:");
        telemetry.update();
         if (isStopRequested()) return;

     rbg.resetindex();
        waitForStart();

     rbg.endgame = rbg.runtime.seconds();
     rbg.tel_start();

     //only for the cam outside duirng the autonomus



        while (opModeIsActive()) {


            switch (state) {

                case INTAKE:
                    if (gamepad1.right_bumper || rbg.lifting) rbg.intake_rolla(false);
                    if (gamepad1.left_bumper) rbg.grab();
//                    if (gamepad1.left_trigger > 0.5|| rbg.folding) rbg.grab_fold();
                    if (gamepad1.circle) {
                        rbg.forcing = true;
                        rbg.pix_full = true;
                        rbg.intake_roller.setPower(-0.9);
                        rbg.intake_rolling=false;
                    }

                    if (rbg.pix_det() && rbg.intake_ready) state = State.INTAKEDONE;

                    break;

                case INTAKEDONE:
                    if (rbg.intake_done()) {
                        rbg.resetindex();
                        if (rbg.forcing) rbg.forcing_single();
                        state = State.LIFT;
                    }
                    break;


                case LIFT:

                    if ((!rbg.outtaking) && (gamepad1.right_bumper || gamepad1.left_bumper)) {

                        if (gamepad1.right_bumper){
                            rbg.side_align = false;
                        }
                        else{
                            rbg.side_align = true;
                        }
                        rbg.outtaking = true;
                        rbg.finalxy();

                    }

                    if (gamepad1.right_bumper || gamepad1.left_bumper) {
                        if (gamepad1.right_bumper){
                            rbg.side_align = false;
                        }
                        else{
                            rbg.side_align = true;
                        }
                        rbg.driving = false;
                        rbg.april_align(rbg.base_apr_id);
                    } else rbg.driving = true;

                    if (rbg.outtaking && rbg.outtake_up_post()) {
                        if (rbg.outtake_servo_out()) {
                            rbg.resetindex();
                            rbg.outtaking = false;
                            state = State.OUTTAKE;
                        }
                    }
                    break;

                case OUTTAKE:

                    if (gamepad1.right_bumper || gamepad1.left_bumper) {
                        if (gamepad1.right_bumper){
                            rbg.side_align = false;
                        }
                        else{
                            rbg.side_align = true;
                        }
                            rbg.driving = false;
                            rbg.april_align(rbg.base_apr_id);
                            rbg.aligning = true;
                    }
                    else
                    {
                            rbg.aligning = false;
                            rbg.driving = true;
                    }
                rbg.coord_adj(gamepad2.right_stick_x, gamepad2.right_stick_y);
                rbg.fine_adj(gamepad2.left_stick_x, gamepad2.left_stick_y);

                if (gamepad2.right_bumper && !rbg.aligning) rbg.outtake_drop();


                if (rbg.drop_qty == 0) state = State.OUTTAKEDONE;
                    break;


                case OUTTAKEDONE:
                    if (rbg.outtake_donea()) {

                        state = State.INTAKE;
                    }

                    break;

            }


// run every cycle below

            if (rbg.driving) {
                if (robo_drive)
                    rbg.robot_centric(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                   // rbg.demo_robot_centric(gamepad1.right_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x );
                else
                    rbg.field_centric(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }
            rbg.in_roll_back(gamepad1.right_trigger);

            if(rbg.tempinput!=0) {
                if (gamepad2.touchpad || rbg.touching) {
                    if (!rbg.touching) {
                        rbg.touching = true;
                        rbg.timerconfirm(0);
                    }

                    if (!gamepad2.touchpad && rbg.timerconfirm(80)) {
                        rbg.input_confirm(gamepad2.touchpad_finger_1_x, gamepad2.touchpad_finger_1_y);
                    }
                }


            }


            if(rbg.timerinput(150))  rbg.coordinate_input2(gamepad2.triangle, gamepad2.circle, gamepad2.cross, gamepad2.square,gamepad2.dpad_up,gamepad2.dpad_right,gamepad2.dpad_down);
            if (rbg.runtime.seconds() - rbg.endgame > 105 && !end_flag){
                gamepad1.rumble(1.0,1.0,750);
                end_flag = true;

            }





//        if (gamepad1.dpad_left) {
//                rbg.stop_drive();
//                if (rbg.runtime.seconds() - rbg.endgame > 88) {
//                    rbg.drone_hang();
//                } else {
//                    sleep(300);
//                    if (gamepad1.dpad_left) {
//                        rbg.drone_hang();
//                        //
//
//
//                    }
//                }
//
//            }

            if (gamepad1.dpad_up) {
                rbg.stop_drive();
                if (rbg.runtime.seconds() - rbg.endgame > 88) {
                    rbg.drone();
                }
                else {
                    sleep(300);
                    if (gamepad1.dpad_up) {
                        rbg.drone();
                    }
                }

            }

            if (gamepad1.dpad_right) {
                rbg.stop_drive();
                if (rbg.runtime.seconds() - rbg.endgame > 89) {
                    rbg.hang();
                } else {
                    sleep(300);
                    if (gamepad1.dpad_right) {
                        rbg.hang();
                        //


                    }
                }

            }




            if (gamepad1.triangle) {
                rbg.stop_drive();
                sleep(250);
                robo_drive = !robo_drive;
            }

            if (gamepad1.dpad_down) {
                rbg.stop_drive();
                sleep(350);
                if (gamepad1.dpad_down) {
                    rbg.out_slide_reset();
                    state = State.INTAKE;
                }
            }

            if (gamepad1.ps){
                rbg.stop_drive();
                sleep(200);
                if (gamepad1.ps){
                    rbg.baseblue = true;
                    rbg.base_align_angle = -90;
                    rbg.base_apr_id = 1;
                }
            }

            if(gamepad1.share) {
                rbg.stop_drive();
                sleep(200);
             if(gamepad1.share) {
                 if (rbg.baseblue) {
                     rbg.imu.resetYaw();
                     rbg.base_align_angle = -90;
                     rbg.base_apr_id = 1;


                     sleep(500);
                 } else {
                     rbg.imu.resetYaw();
                     rbg.base_align_angle = 90;
                     rbg.base_apr_id = 4;
                     sleep(500);
                 }
                 gamepad1.rumble(0.3, 0.3, 300);
             }



            }



           if(rbg.printing) {


               telemetry.addData("Y1", rbg.input1y);

               telemetry.addData("X1", rbg.input1x);

               telemetry.addLine();
               telemetry.addLine();



               telemetry.addData("Y2", rbg.input2y);
               telemetry.addData("X2", rbg.input2x);


               telemetry.update();
               rbg.printing=false;
           }



        }
    }
}









