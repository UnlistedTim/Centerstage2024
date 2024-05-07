package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import   java.lang.Math;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class BaseClass extends MecanumDrive {
    protected DcMotorEx front_left;
    protected DcMotorEx front_right;
    protected DcMotorEx rear_left;
    protected DcMotorEx rear_right;
    protected DcMotorEx arm_rotate;
    protected DcMotorEx arm_slide;
    protected Servo arm_grab;
    protected Servo arm_handle;
    
    
    
    
    
    
    
    
    protected DcMotorEx intake_roller;
    protected DcMotorEx outtake_slider;
    protected DcMotorEx hang_slider;
    protected DistanceSensor out_left_dis;
    protected DistanceSensor out_right_dis;
    protected DistanceSensor grab_dis;
    protected  DistanceSensor side_left_dis;
    protected  DistanceSensor side_right_dis;
    private VoltageSensor myControlHubVoltageSensor;
    private VoltageSensor myExpansionHub2VoltageSensor;
    protected ColorSensor bot_color;
    protected ColorSensor top_color;
    protected Servo intake_grab;
    protected Servo intake_handle;
    protected Servo hang_lock;
    protected WebcamName webcam1;
    protected Servo outtake_cam;
    protected Servo outtake_handle;
    HuskyLens huskyLens;
    protected Servo outtake_move;


    protected DigitalChannel right_touch;
    protected DigitalChannel left_touch;

    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode Op;
    double st,st1,st2,st3,st4,st5,st6,strb;
    double robotVoltage;
    public static boolean right=false;

   public static boolean baseblue=false,baseright=true;
   public static double  base_align_angle;
   public static int    base_apr_id;

   //Servo preset value
   double arm_handle_ip1, arm_handle_ip2,arm_handle_ip3,arm_handle_ip4,arm_handle_ip5,arm_handle_idle;
   double arm_handle_op1,arm_handle_op2,arm_handle_op3,arm_handle_op4,arm_handle_op6,arm_handle_op7,arm_handle_op8,arm_handle_op9;
  double arm_grab_hold,arm_grab_idle,arm_grab_open1,arm_grab_open2;

  //Motor preset value

    int  arm_rotate_ip1,arm_rotate_ip2,arm_rotate_ip3,arm_rotate_ip4,arm_rotate_ip5;
    int  arm_rotate_op1,arm_rotate_op2,arm_rotate_op3,arm_rotate_op4,arm_rotate_op5,arm_rotate_op6,arm_rotate_op7,arm_rotate_op8,arm_rotate_op9;
    int arm_slide_extend,arm_sldie_idle;
    int  arm_slide_op1,arm_slide_op2,arm_slide_op3,arm_slide_op4,arm_slide_op5,arm_slide_op6,arm_slide_op7,arm_slide_op8,arm_slide_op9;




    final double SPEED_GAIN = 0.025; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.033; //0.03  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.03;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.36;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    double DESIRED_DISTANCE_T = 27.5;//CM
    //double DESIRED_DISTANCE_A =10;//INCH?

    double rangeError = 20;
    int finalcycle=3;
    double headingError = 10;
    double yawError = 10;
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)


    int outtake_1_delay = 0;
    int outtake_2_delay = 0;
   // Pose2d P;

    final double HSPEED_GAIN = 0.025; // 0.02  //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
     double HSTRAFE_GAIN = 0.018; //0.01,0.02  //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double HTURN_GAIN = 0.032;  //0.015  //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double HMAX_AUTO_SPEED = 0.3;   //  Clip the approach speed to this max value (adjust for your robot)
    final double HMAX_AUTO_STRAFE = 0.28; //0.5  //  Clip the approach speed to this max value (adjust for your robot)
    final double HMAX_AUTO_TURN = 0.25;   //  Clip the turn speed to this max value (adjust for your robot)

    double april_left_strafe = -6.6, april_right_strafe = 8.5;  //8.0
    double HrangeError = 20 ,walltarget=3,temp1,temp2,temp3;
    double HheadingError = 10;
    double HyawError = 10;
    double Hdrive = 0;        // Desired forward power/speed (-1 to +1)
    double Hstrafe = 0;        // Desired strafe power/speed (-1 to +1)
    double Hturn = 0;        // Desired turning power/speed (-1 to +1)
    Pose2d startPoseA = new Pose2d(0, 0, 0);


    long[] delay={0,0,0,0};
    final  double[] row1x = {0.42,0.175,0.295, 0.365, 0.47,0.555,0.675};
   final double[] row2x = {0.42, 0.105, 0.24,0.33,0.42,0.51,0.61,0.766};

   final int[] row1y = {310, 440,350,340,340,350,440};
   final int[] row2y = {540, 750,610,580,560,570,620,750};

//    final  double[] row1xc = {0.42, 0.42,0.42, 0.42, 0.185, 0.285, 0.375};
final  double[] row1xc = {0.42, 0.42,0.42, 0.42, 0.205, 0.305, 0.395};
    final double[] row2xc = {0.42, 0.42, 0.42, 0.42, 0.1,   0.245, 0.335,0.42};

    final int[] row1yc = {350, 350, 350, 350, 420, 360,350};
    final int[] row2yc = {550, 550,550, 550,740,610, 560, 550};

   boolean roll_flag = false,timeend=false;


    double finalx=0.42;
   int  finaly=750;


    int input1x = 0,currentx=4;
    int input1y = 0,currenty=2;
    int tempinput = 0,intake_done_step=0;
    int index=2;
    double husk_tar=110;//110
    static int encoder;


    int input2x = 0;
    int input2y = 0;

    class Cservo {
        Servo name;

     double[] postion;
     Cservo (Servo s,double v1, double v2)
     {
         this.name=s;
         name.scaleRange(v1,v2); // limit operation interval of the servo
         name.setDirection(Servo.Direction.REVERSE); // Reverse the Servo rotating direction
         double name_open = 0.3, name_close = 0.5, name_hold = 0.3; //define preset positions
         name.setPosition(name_open);
         name.setPosition(name_close);
         name.setPosition(name_hold);

     }


    }


    int tmp1=1000,tmp2=1000;


    int[] out_height = {0, 350, 425, 655, 885, 1115, 1345, 1450, 1750, 1800,140};// 0, withdraw, l,m,h // 700 old


   final int[] out_speed = {600, 1200, 2800, 1800};
  //  int height_level = 2;//Default postion level
    //int move_level = 3; // Default positon level
    double[] out_move = {0.12, 0.27, 0.35, 0.42, 0.49, 0.57, 0.72};//
 //   double[] fine_move = {-0.04, -0.02, 0, 0.02, 0.04};

   final double[] fine_move_1x = {0.05,0.08, 0.06, 0.05,0.05, 0.06, 0.08};
   final  double[] fine_move_2x = {0.04,0.10, 0.07, 0.05, 0.04,0.05,0.07, 0.10};
    double[] in_handle = {0.165,0.975,0.94,0.43,0.6,0.860,.98}; // start_fold,grab, move, move_fold,,hinge,auto,auto_left;0.962
   // double[] in_handle = {0.14,0.975,0.94,0.3,0.6,0.86}; //close/fold, intake, move,hinge,auto,auto_left;0.962

    double[] out_handle = {0.16, 0.34, 0.48, 0.62, 0.78,0.12,1.0};

    double[] out_cam = {0, 0.17, 0.34};
    //handle 0.375, arm 0.001 , grab 0.30


    //double[] in_grab = {0.35, 0.8 ,0.335,0.45,0.5};// close//fold, open,unhold,hold,preopen 0.35


    double[] in_grab = {0.36, 0.75 ,0.400,0.63,0.54};// close, open,lock/hold, autopen/halfopen, 0.35,0.78


    int  drop_qty = 2;
//    int bred = 0, bgreen = 0, bblue = 0, tred = 0, tgreen = 0, tblue = 0;

    int fine_move_level = 0, fine_height_level=0, fine_height_offset=0;
    int turn_dir, g_level,down_post_step=0;
    boolean intake=true;

    boolean lifting = false, intake_rolling = false, outtaking = false, driving = true,bot_fla=false,top_fla=false;
    boolean forcing = false, leveling = false, launching = false,  turning = false, out_handling=false;
    boolean vri1 = false, vir2 = false,roll_backing=false,forwarding=false,outtake_dropping = false;
    boolean aligning = false,adjusting=false,printing=true,init_setup=true, april_ready=false,setup_mode=false;
    boolean intake_grabing = false, pix_full = false,dropping=false,cor_updating1=false,cor_updating2=false, adj_key_reset=true,fadj_key_reset=true,grab_folding=true;
    int grab_step = 0, align_step = 0, servo_step = 0, post_step = 0, drop_step = 0, done_step = 0, color_step = 0, turn_step = 0,input_count=0,roll_back_step=0,intake_step=0;
    boolean touching=false,out_handle_in=false; boolean half_full = false,wall_route=false, side_align = false;
    double headingang, targetang, turnang,stinput=0,endgame,fine_move_offset=0,time_period=0,stconfirm=0;
    int folding_step=0,grab_counter=0;
    boolean folding=false, intake_ready=false, prev_side_align = false;
    double ax11,ax12,ax21,ax22;
    double delta_dis=0;
    int    ay11,ay12,ay21,ay22;
    int cycle=0;// pile intake

      int layer_rate = 230;
      int drop_offset = 310;//300
      int x=0;
      double ax,ay;
   boolean volthi=false,acal=false;
   double y;


    //private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    //private static final int DESIRED_TAG_ID = 1; //-1    // Choose the tag you want to approach or set to -1 for ANY tag.
    public VisionPortal visionPortal;               // Used to manage the video source.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    IMU imu;
    IMU.Parameters imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));


    public BaseClass(LinearOpMode linearOpMode, Pose2d pose) {
        super(linearOpMode.hardwareMap,pose);
        Op = linearOpMode;
        initHardware(Op.hardwareMap);
    }

    public BaseClass(HardwareMap hardwareMap ,Pose2d pose1) {
        super(hardwareMap,pose1);
        initHardware(hardwareMap);

    }
    void intake_rolla(boolean grab) {


        if (intake_step == 0){

            if (!grab && (!grab_folding)) grab_fold();
            if(!grab) {
                if (myControlHubVoltageSensor.getVoltage()>13.0)intake_roller.setPower(0.7);
                else intake_roller.setPower(0.72);
                intake_rolling = true;
               }
            else{
                intake_roller.setPower(0); // 0.7
                 intake_rolling = false;
                }
            if(intake_ready) return;
            lifting=true;

            if(outtake_slider.getCurrentPosition() < out_height[2]-8) {
                outtake_slide(out_height[2], out_speed[3]);
                intake_step = 1;
            }

        }
        if (intake_step == 1){
            if (outtake_slider.getCurrentPosition() > out_height[2]-8){
                outtake_handle.setPosition(out_handle[4]);
                timer3(0);
                intake_step = 2;
            }
        }
        if (intake_step == 2){
            if (timer3(700)){
                outtake_slide(out_height[1], out_speed[1]);
                lifting = false;
                intake_ready=true;
                intake_step=0;


            }

        }


    }
    void intake_handlea(int p)

    {

        intake_handle.setPosition(in_handle[index]);

    }




    public void intake_handle(int index)

    {

        intake_handle.setPosition(in_handle[index]);

    }
    public void husk_alignb() {
// 240 120
//220 110
//200 108
//190 105
//180  99
        int READ_PERIOD = 1;int j=0;
        double y0=240;
        int ytarget=200;
        x = 0;y=0;
        // intake_handle(2);
        if(volthi) HSTRAFE_GAIN=0.016;
        //timer3(0);
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        timer3(0);
        y=grab_dis.getDistance(DistanceUnit.MM);
        if(y>2000) y=y0;


        while (!timer3(1000)) {


            HuskyLens.Block[] blocks = huskyLens.blocks();

            for (int i = 0; i < blocks.length; i++) {

                if (blocks[i].x > 0) x = blocks[i].x;

            }
         //   husk[j]=x;
//            if(j<24)j++;
            y=grab_dis.getDistance(DistanceUnit.MM);
            if(y>1000) y=y0;
            y0=y;
            HrangeError=(y-ytarget)/10;
            husk_tar=(y-180)*22/60+100;
         //   husk_tar=(y-180)/3+98;

            if (x > 0) HyawError = (x - husk_tar);// 110 iss the target x valule for alignment
            else HyawError = 0;
            HheadingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            Hdrive = Range.clip(HrangeError * HSPEED_GAIN, -HMAX_AUTO_SPEED, HMAX_AUTO_SPEED);
            Hstrafe = Range.clip(-HyawError * HSTRAFE_GAIN, -HMAX_AUTO_STRAFE, HMAX_AUTO_STRAFE);
            Hturn = Range.clip(HheadingError * HTURN_GAIN, -HMAX_AUTO_TURN, HMAX_AUTO_TURN);
            moveRobot(Hdrive, Hstrafe, Hturn, 1);
            if (x> 0 && Math.abs(HyawError) < 12) break;
        }
        stop_drive();
    }




    public void out_slide_reset() {
        intake_roller.setPower(0);



        stop_drive();
        intake_handle(2);
        outtake_slide(outtake_slider.getCurrentPosition() + 450, out_speed[0]);
        pause(1000);
        intake_grab(1);
        outtake_cam.setPosition(out_cam[0]);
        outtake_move.setPosition(out_move[3]);
        pause(600);
        outtake_handle.setPosition(out_handle[4]);
        intake_grab(0);
        pause(600);

        intake_handle(3);
        outtake_slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake_slider.setPower(-0.15);
        pause(1500);
        intake_handle(0);
        outtake_slider.setPower(0);
        pause(200);
        outtake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slider.setTargetPosition(0);
        outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      


    }


    public void finalxy(){

        if(drop_qty==2) {
           if(!adjusting) {
               if (input1x > 0){
                   currentx=input1x;
               }
               if (input1y > 0){
                   currenty=input1y;
               }


           }
           cor_updating1=false;
        }
        else {
                if (!adjusting) {
                    if (input2x > 0){
                        currentx=input2x;
                    }
                    if (input2y > 0){
                        currenty=input2y;
                    }


                }
                cor_updating2=false;
            }


       // if(currentx==0||currenty==0) vir2=true;
//        if(currenty==0 || currentx==0) return;
        if (side_align){
            if (currentx < 4){
                vir2 = true;
                return;
            }
            if (currenty%2==0){

                finalx = row2xc[currentx];
                finaly = row2yc[currentx] + (currenty-2)*layer_rate;
                finaly+=drop_offset;
            }
            else{
                if (currentx == 7) {// may change to 6;
                    currentx=6;
                    vir2 = true;

                }
                finalx = row1xc[currentx];
                finaly = row1yc[currentx] + (currenty-1)*layer_rate;
                finaly+=drop_offset;
                //if(currenty==1) finaly-=30;
            }
        }
        else{
            if (currenty%2==0){

                finalx = row2x[currentx];
                finaly = row2y[currentx] + (currenty-2)*layer_rate;
                finaly+=drop_offset;
            }
            else{
                if (currentx == 7) {// may change to 6;
                    currentx=6;
                    vir2 = true;

                }
                finalx = row1x[currentx];
                finaly = row1y[currentx] + (currenty-1)*layer_rate;
                finaly+=drop_offset;

                //if(currenty==1) finaly-=30;
            }


        }
        if(currenty==1) finaly=finaly-35;
        prev_side_align = side_align;

    }


      void  outtake_forward() {
            outtake_update();
          double dis=out_left_dis.getDistance(DistanceUnit.MM);
          if (dis<300) dis=200;
          else dis=250;
        timer(0);
          move(-0.22);
          while (out_left_dis.getDistance(DistanceUnit.MM) > dis&&!timer(400)) {

              }
          stop_drive();
          forwarding=true;


      }

    boolean outtake_donea()
    {
           outtake_move.setPosition(out_move[3]);
           outtake_handle.setPosition(out_handle[5]);
           outtake_slide(out_height[0],out_speed[2]);
            resetindex();
            forcing = false;
            input1x=0;
            input2x=0;
            input1y=0;
            input2y=0;
//            finalx = 0.42;
//            finaly = 750;
            return  true;
    }




    boolean outtake_servo_out() {
        if (servo_step == 0) {
            outtake_handle.setPosition(out_handle[0]);
            servo_step = 1;
            timer(0);
            return false;

        }
        if (servo_step == 1 && timer(400)) {
            servo_step = 0;
            outtake_move.setPosition(finalx);
            return true;
        }


        return false;

    }

    public void startSlide(int target, int vel, boolean async) {



        if (async) {

            Thread thread = new Thread() {
                public void run() {



                    outtake_slider.setTargetPosition(target);


                    while (outtake_slider.getCurrentPosition() < 150) {
                        outtake_slider.setVelocity(out_speed[0]);
                    }
                    outtake_slider.setVelocity(vel);

                }
            };
            thread.start();
        } else outtake_slide(target, vel);

    }

    void intake_grab(int index)

    {
        intake_grab.setPosition(in_grab[index]);

    }



    boolean init(boolean auto) {
       if(init_setup) {
           if(baseblue) {
               base_align_angle=-90;
               base_apr_id=1;
           }
           else {
               base_align_angle=90;
               base_apr_id=4;
           }

           if (auto) {
           if(!wall_route) outtake_cam.setPosition(out_cam[2]);
            imu.initialize(imuparameters);
            imu.resetYaw();
            outtake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtake_slider.setTargetPosition(0);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pause(500);
            intake_handle(0);//fold
           outtake_handle.setPosition(out_handle[4]);
           outtake_move.setPosition(out_move[3]);
              // intake_grab(0);//fold




           }
           else {
               intake_handle(0);//fold
            imu.initialize(imuparameters);
               pause(500);
               outtake_cam.setPosition(out_cam[0]);
        }
           tmp1=outtake_slider.getCurrentPosition();// for firt time reading issue
        hang_lock.setPosition(0);
        //outtake_handle.setPosition(out_handle[4]);
        outtake_move.setPosition(out_move[3]);
        if(myControlHubVoltageSensor.getVoltage()>13.7) volthi=true;
        else volthi=false;

        //init_setup=false; move in iniPariltag
       }
       return iniAprilTag(webcam1);


    }





    void tel_start()

    {
        intake_grab(0);//fold
        intake_handle(0);
        outtake_handle.setPosition(out_handle[5]);

    }

    boolean forcing_single() {

        if ((top_color.red() + top_color.blue() + top_color.green()) > 2500) {
            forcing = false;
            return false;
        }


        return true;

    }


    void che_voltage() {
        robotVoltage = myControlHubVoltageSensor.getVoltage();
     //   robotVoltage = myExpansionHub2VoltageSensor.getVoltage();


    }


    boolean pix_det() {
        if (pix_full) return true;
        if (color_step == 0) {

            if (bot_color.red()+bot_color.blue()+bot_color.green() < 2500)   return false;
            else {
                timer(0);
                color_step = 1;
                half_full = true;
                time_period=300;
                return false;
            }
        }

        if (color_step == 1 && timer(time_period)) {
            timer(0);
            time_period=200;
            if ((top_color.red() + top_color.blue() + top_color.green() > 2500)) {

                pix_full = true;
                half_full = false;
                color_step = 0;

                intake_roller.setPower(-0.9);
                intake_rolling=false;
                timerroollback(0);
                roll_flag=true;
                return true;
            }

        }
        return false;
    }

    public void initHardware(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuparameters);
        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
        rear_left = hardwareMap.get(DcMotorEx.class, "rear_left");
        rear_right = hardwareMap.get(DcMotorEx.class, "rear_right");

        arm_rotate = hardwareMap.get(DcMotorEx.class, "arm_rotate");
        arm_slide = hardwareMap.get(DcMotorEx.class, "arm_slide");

        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
        arm_handle = hardwareMap.get(Servo.class,"arm_handle");
  

        
        


        intake_roller = hardwareMap.get(DcMotorEx.class, "intake_roller");
        outtake_slider = hardwareMap.get(DcMotorEx.class, "outtake_slider");

        hang_slider = hardwareMap.get(DcMotorEx.class, "hang_slider");
        intake_grab = hardwareMap.get(Servo.class, "intake_grab");
        intake_handle = hardwareMap.get(Servo.class,"left_intake_handle");
        outtake_cam = hardwareMap.get(Servo.class, "outtake_cam");
        outtake_handle = hardwareMap.get(Servo.class, "outtake_handle");
        outtake_move = hardwareMap.get(Servo.class, "outtake_move");
        //   outtake_arm = hardwareMap.get(Servo.class,"outtake_arm");
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
      //  webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        hang_lock = hardwareMap.get(Servo.class, "hang_lock");


       side_left_dis = hardwareMap.get(DistanceSensor.class, "side_left_dis");
        side_right_dis = hardwareMap.get(DistanceSensor.class, "side_right_dis");
        bot_color = hardwareMap.get(ColorSensor.class, "bot_color");
        top_color = hardwareMap.get(ColorSensor.class, "top_color");
        out_right_dis = hardwareMap.get(DistanceSensor.class, "left_dis");
        out_left_dis = hardwareMap.get(DistanceSensor.class, "right_dis");
        grab_dis = hardwareMap.get(DistanceSensor.class, "grab_dis");
        //  side_disl=hardwareMap.get(DistanceSensor.class,"side_dis");


        right_touch = hardwareMap.get(DigitalChannel.class, "right_touch");
        left_touch = hardwareMap.get(DigitalChannel.class, "left_touch");
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        myExpansionHub2VoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");


//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }


        outtake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slider.setTargetPosition(0);
        outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang_slider.setTargetPosition(0);
        hang_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    boolean outtake_up_post() {

        if (post_step == 0) {
            outtake_handle.setPosition(out_handle[4] + 0.015);
            outtake_slide(finaly,out_speed[1]);
            post_step = 1;
            return false;
        }


        if (post_step == 1 && outtake_slider.getCurrentPosition() > 150) {
            outtake_slider.setVelocity(out_speed[2]);
            post_step = 2;
            return false;
        }

        if (outtake_slider.getCurrentPosition() > (out_height[2]-6)) {

            // post_step=0;
            return true;

        }
        return false;
    }

    public void grab_fold(){

           intake_handle(3);
           intake_grab(0);

           grab_folding=true;

    }





    boolean intake_done() {


        if(!grab_folding) grab_fold();
        if (intake_done_step==0&& outtake_down_post()) {


           // outtake_cam.setPosition(out_cam[2]);
          //  timer5(0);
            outtake_cam.setPosition(out_cam[2]);
            timer(0);
            lifting = false;
           // intake_roller.setPower(0);
           intake_handle(0);
            intake_ready=false;
            done_step=0;
            intake_done_step=1;

        }
      if(intake_done_step==1&&timer(400)) {intake_roller.setPower(0); return true;}


        return false;

    }


    boolean outtake_down_post() {

        if (down_post_step == 0) {
//            intake_roller.setPower(-0.9);
           // intake_rolling=false;
           outtake_cam.setPosition(out_cam[0]);
           // tmp1=outtake_slider.getCurrentPosition();
            outtake_slide(out_height[0],out_speed[1]);
           down_post_step = 1;
            return false;
        }

     if(down_post_step==1) {

         if (outtake_slider.getCurrentPosition()< (out_height[0] + 9)) {

             down_post_step = 2;

             return true;
         }
     }

    return false;
    }

    protected void field_centric(double iy, double ix, double irx) {
        double y = iy; // Remember, this is reversed!
        double x = ix * 1.1; // Counteract imperfect strafing
        double rx = irx * 0.75;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
//        rear_left.setDirection(DcMotorSimple.Direction.REVERSE);


        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        front_left.setPower(frontLeftPower);
        rear_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        rear_right.setPower(backRightPower);

    }







    public void in_roll_back(double trigger){
        if(trigger>0.9){ intake_roller.setPower(-0.9);roll_flag=true;timerroollback(0);}
        else if(roll_flag&&timerroollback(3000)){ intake_roller.setPower(0); roll_flag=false;}
    }

    public void clockwise(double power) {
        front_left.setPower(power);
        rear_left.setPower(power);
        front_right.setPower(-power);
        rear_right.setPower(-power);
    }





    public void straft(boolean right, double power) {

        double dir = 1;
        if (right) dir = -1;
        // double  power=1;
        front_left.setPower(dir * (power + 0.035));
        rear_left.setPower(-dir * power);
        front_right.setPower(-dir * (power + 0.03));
        rear_right.setPower(dir * power);
    }





    protected void robot_centric(double iy, double ix, double irx) {
        double y = iy;
        double x = -ix * 1.1; // Counteract imperfect strafing
        double rx = -irx * 0.75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        front_left.setPower(-frontLeftPower);
        front_right.setPower(-frontRightPower);
        rear_left.setPower(-backLeftPower);
        rear_right.setPower(-backRightPower);


    }

    protected void demo_robot_centric(double iy, double ix, double irx) {
        double y = iy;
        double x = -ix * 1.1; // Counteract imperfect strafing
        double rx = -irx * 0.75;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        front_left.setPower(-frontLeftPower*0.3);
        front_right.setPower(-frontRightPower*0.3);
        rear_left.setPower(-backLeftPower*0.3);
        rear_right.setPower(-backRightPower*0.3);


    }




//    public void drone_launch() {
//        stop_drive();
//
//        launching=true;
//        pause(500);
//
//
//    }

    public void drone_hang() {

        int out_high=0; int step=0;
        boolean left_flag=false, right_flag=false;
        stop_drive();
        hang_slider.setTargetPosition(-4520); //-4550
        hang_slider.setVelocity(2800); // 2800
        outtake_slider.setTargetPosition(out_height[8]);//1750
        outtake_slider.setVelocity(out_speed[2]);
        timer(0);
        while(out_high<(out_height[8]-10) && !timer(4000))
        {
            out_high=outtake_slider.getCurrentPosition();
            if (step==0)
                if( out_high>out_height[2]) {outtake_handle.setPosition(out_handle[4]);step=1;}
        }

        outtake_handle.setPosition(out_handle[6]);
        pause(400);
        outtake_handle.setPosition(out_handle[4]);
        while ((hang_slider.getCurrentPosition() > -4470)) { // -4500


        }

        front_left.setPower(-0.25);
        front_right.setPower(-0.25);
        rear_left.setPower(-0.25);
        rear_right.setPower(-0.25);
        timer(0);
        while ((!right_flag || !left_flag) && !timer(3000)) {
            if (!right_touch.getState()) {
                front_right.setPower(0);
                rear_right.setPower(0);
                right_flag=true;
            }
            if (!left_touch.getState()) {
                front_left.setPower(0);
                rear_left.setPower(0);
                left_flag=true;
            }

        }
        stop_drive();
        hang_lock.setPosition(0.3);
        hang_slider.setTargetPosition(-3100); // -3100
        hang_slider.setVelocity(1500); //1500
        outtake_slide(out_height[0],out_speed[1]);

    }


    public void drone() {

        int out_high=0; int step=0;
        stop_drive();
        outtake_slide(out_height[8],out_speed[2]);

        timer(0);
        while(out_high<(out_height[8]-20) && !timer(2000))
        {
            out_high=outtake_slider.getCurrentPosition();
            if (step==0)
                if( out_high>out_height[2]) {outtake_handle.setPosition(out_handle[4]);step=1;}
        }

        outtake_handle.setPosition(out_handle[6]);
        pause(400);
        outtake_handle.setPosition(out_handle[4]);
        stop_drive();
        outtake_slide(out_height[0],out_speed[2]);

    }

    public void hang(){
        int out_high=0; int step=0;
        boolean left_flag=false, right_flag=false;
        stop_drive();
        hang_slider.setTargetPosition(-4520); //-4550
        hang_slider.setVelocity(2800); // 2800
        while ((hang_slider.getCurrentPosition() > -4470)) { // -4500


        }

        front_left.setPower(-0.28);
        front_right.setPower(-0.28);
        rear_left.setPower(-0.28);
        rear_right.setPower(-0.28);
        timer(0);
        while ((!right_flag || !left_flag) && !timer(3000)) {
            if (!right_touch.getState()) {
                front_right.setPower(0);
                rear_right.setPower(0);
                right_flag=true;
            }
            if (!left_touch.getState()) {
                front_left.setPower(0);
                rear_left.setPower(0);
                left_flag=true;
            }

        }
        stop_drive();
        if(timer(3000)) return;
        hang_lock.setPosition(0.3);
        hang_slider.setTargetPosition(-3100); // -3100
        hang_slider.setVelocity(1500); //1500
      //  outtake_slide(out_height[0],out_speed[2]);
    }



//    public void robot_hang() {
//
//       boolean left_flag=false, right_flag=false;
//        stop_drive();
//
//
//        hang_slider.setTargetPosition(-4520); //-4550
//        hang_slider.setVelocity(2800); // 2800
//        while (hang_slider.getCurrentPosition() > -4470) { // -4500
//
//        }
//      //  hang_slider.setVelocity(0);
//
//        front_left.setPower(-0.25);
//        front_right.setPower(-0.25);
//        rear_left.setPower(-0.25);
//        rear_right.setPower(-0.25);
//
//        while (!right_flag || !left_flag) {
//            if (!right_touch.getState()) {
//                front_right.setPower(0);
//                rear_right.setPower(0);
//                right_flag=true;
//            }
//            if (!left_touch.getState()) {
//                front_left.setPower(0);
//                rear_left.setPower(0);
//                left_flag=true;
//            }
//
//        }
//        hang_lock.setPosition(0.3);
//        hang_slider.setTargetPosition(-3100); // -3100
//        hang_slider.setVelocity(1500); //1500
//
//    }

    // bring slide back for transition

    void resetindex() {
        pix_full = false;
        drop_qty = 2;
        intake_rolling = false;
        intake_grabing = false;
        lifting = false;
        leveling = false;
        outtaking = false;
        done_step = 0;
        grab_step = 0;
        drop_step = 0;
        servo_step = 0;
        post_step = 0;
        align_step = 0;
        color_step = 0;
        roll_backing=false;
        roll_back_step=0;
        fine_move_level = 0;
        fine_height_level=0;
        down_post_step=0;
        forwarding=false;
        intake_step = 0;
        half_full = false;
        grab_folding=true;
        folding_step=0;
        intake_done_step=0;
    }


    public void forward(double power, long ms) {


        front_left.setPower(power);
        rear_left.setPower(power);
        front_right.setPower(power);
        rear_right.setPower(power);

        pause(ms);
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);

    }

    void   boutakepid ()

    {        double angle_gap, yaw, yaw_gain=0.04,max_turn=0.35 ,dis;//0.035,0.3
        double ygap,xgap,target_dis1=76 ,target_dis2=78,xtarget=70,ytarget=20,yrange=0,move_gain=0.04,max_move=0.3;//-78
        double target_dis=0,xrange=0,strafe_gain=0.06,max_strafe=0.3;
        boolean stop = false;
        int dir = 1;
        if(baseblue) dir=-1;
        if(volthi) max_move=0.26;
        while (!stop) {//!timer3(time)&&
            angle_gap = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
            yaw = Range.clip(angle_gap * yaw_gain, -max_turn, max_turn);
            ygap = ytarget - out_left_dis.getDistance(DistanceUnit.CM);
            if (Math.abs(ygap) < 1.5) {
                break;
            }
            if (!baseblue) dis = side_left_dis.getDistance(DistanceUnit.INCH);
            else dis = side_right_dis.getDistance(DistanceUnit.INCH);
            xgap = -(xtarget- dis);
            yrange =   Range.clip(ygap * move_gain, -max_move, max_move);
            xrange = dir * Range.clip(xgap * strafe_gain, -max_strafe, max_strafe);
            moveRobot(yrange, xrange, yaw, 1);
        }

        stop_drive();
    }

    private boolean timer(double period) {

        if (period == 0) {
            st = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st > period) return true;

        return false;

    }
    private boolean timer1(double period) {

        if (period == 0) {
            st1 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st1 > period) return true;

        return false;

    }
   boolean timer2(double period) {

        if (period == 0) {
            st2 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st2 > period) return true;

        return false;

    }
    boolean timer4(double period) {

        if (period == 0) {
            st4 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st4 > period) return true;

        return false;

    }
    boolean timerroollback(double period) {

        if (period == 0) {
            strb = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - strb > period) return true;

        return false;

    }


        boolean timer5(double period) {

            if (period == 0) {
                st5= runtime.milliseconds();
                return false;
            }

            if (runtime.milliseconds() - st5 > period) return true;

            return false;

        }





    boolean timer3(double period) {

        if (period == 0) {
            st3 = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - st3 > period) return true;

        return false;

    }

    boolean timerinput(double period) {

        if (period == 0) {
            stinput = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - stinput > period) return true;

        return false;

    }

    boolean timerconfirm(double period) {

        if (period == 0) {
            stconfirm = runtime.milliseconds();
            return false;
        }

        if (runtime.milliseconds() - stconfirm > period) return true;

        return false;

    }

    public void move(double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        rear_left.setPower(power);
        rear_right.setPower(power);
    }

    public void stop_drive() {
        front_left.setPower(0);
        front_right.setPower(0);
        rear_left.setPower(0);
        rear_right.setPower(0);
    }

    public void outtake_drop() {

        stop_drive();


        if(outtake_update()) return;
        double dis=out_left_dis.getDistance(DistanceUnit.MM);
        if (drop_qty == 2) {
          //  double gap = bot_dis.getDistance(DistanceUnit.MM) - 160;
          if(volthi)  move(-0.3);
          else  move(-0.32);
          pause(100);
        if (forcing) outtake_cam.setPosition(out_cam[0]);
        else outtake_cam.setPosition(out_cam[1]);
        pause(100);
        if(dis>280) pause(50);
        stop_drive();
        pause(250);
        drop_qty = 1;

        move(0.3);
        finalxy();
        pause(150);

        if (!forcing) {
                stop_drive();
                outtake_slide(finaly,out_speed[2]);
                outtake_move.setPosition(finalx);

                return;
            }
        }
        if (drop_qty == 1 && !forcing) {


            // forward(-0.22,200);
            move(-0.23);
            pause(100);
            outtake_cam.setPosition(out_cam[0]);
            pause(100);
            //outtake_cam.setPosition(out_cam[0]);

            stop_drive();
            pause(250);
            move(0.3);
            pause(150);
        }
//        currentx=0;currenty=0;
        drop_qty = 0;
        move(0.7);
        pause(100);
        outtake_slide(out_height[2], out_speed[2]);
        stop_drive();
    }


//


    void demo_grab() {

            stop_drive();
            intake_handle(1);
            intake_grab(0);
            intake_roller.setPower(0.75);
            intake_rolling = true;
            pause(400);//390
            intake_grab(1);

            if (!half_full){
                pause(300);//390
                intake_grab(0);
                pause(400);//400
                intake_grab(1);
            }

            timer2(0);
            pause(300);
            grab_fold();
        }




void grab() {
        double target = 95;

            if (grab_folding) {
                intake_handle(2);
                grab_folding = false;
                intake_rolla(true);//to do
                timer4(0);
                intake_grab(1);
                return;
            }

            double dis = 500;
            if (!grab_folding && timer4(400)) {
                stop_drive();
                timer(0);
                double error = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - base_align_angle;
                while(Math.abs(error) > 2 && !timer(1000)){
                    error = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - base_align_angle;
                    if (error > 0){
                        clockwise(0.22);
                    }
                    else{
                        clockwise(-0.22);
                    }
                }
                stop_drive();



                if (grab_dis.getDistance(DistanceUnit.MM) > 200){
                    target = 100;
                    move(0.22);
                }
                else {
                    move(0.25);
                }
                pause(100);
                timer3(0);
                while (dis > target && !timer3(2000)) {
                    dis = grab_dis.getDistance(DistanceUnit.MM);
                    if (dis < 160) {
                        move(0.16);
                    }
                }
                stop_drive();
                intake_handle(1);
                intake_grab(0);
                intake_roller.setPower(0.75);
                intake_rolling = true;
                pause(400);//390
                intake_grab(1);

                if (!half_full){
                    pause(300);//390
                    intake_grab(0);
                    pause(400);//400
                    intake_grab(1);
                }

                timer2(0);
                pause(300);
                move(-0.2);
                pause(150);
                forward(-0.5, 150);
                grab_fold();
            }


        }








    public void coord_adj(double x, double y) {

        double ax = Math.abs(x);
        double ay = Math.abs(y);
        if (ax<0.2&&ay<0.2){ adj_key_reset=true;return;}
        if(!adj_key_reset) return;
        if (ax < 0.6 && ay < 0.6) return;


        if (ax > ay) {
            if (x > 0) {

                    currentx++;
                    if (currenty%2 == 0 && currentx > 7) currentx = 7;
                    if (currenty%2 != 0 && currentx > 6) currentx = 6;

            }
            if (x < 0) {

                   currentx--;
                    if (currentx < 1) currentx = 1;
            }

        } else {
            if (y > 0){
                    currenty--;
                    if (currenty < 1) currenty = 1;
                }

            if (y < 0) {
                     currenty++;
                    if (currenty > 6) currenty = 6;

            }
        }
             adjusting=true;
            finalxy();
            outtake_slide(finaly, out_speed[1]);
            outtake_move.setPosition(finalx);
           pause(200);
            // start_time = runtime.milliseconds();
             fine_move_level = 0;
             fine_height_level=0;
             adjusting=false;
             adj_key_reset=false;
    }

    public void fine_adj(double x,double y) {

        double ax = Math.abs(x);
        double ay = Math.abs(y);
        if (ax<0.2&&ay<0.2){ fadj_key_reset=true;return;}
        if(!fadj_key_reset) return;
        if (ax < 0.5 && ay < 0.5) return;


        if (ax > ay) {
            if (x > 0) {

                fine_move_level++;

            }
            if (x < 0) {

                fine_move_level--;
            }

        } else {
            if (y > 0){

                fine_height_level--;
            }

            if (y < 0) {
                fine_height_level++;

            }
        }

        if (fine_move_level > 3) fine_move_level =3;
         else if (fine_move_level < -3) fine_move_level = -3;

         if (fine_height_level > 2) fine_move_level =2;
        else if (fine_move_level < -2) fine_move_level = -2;


        if(currenty%2==0) {
                if (currentx==1) {if (fine_move_level<0) fine_move_level=0;}
                if (currentx==7) {if (fine_move_level>0) fine_move_level=0;}
                fine_move_offset = fine_move_2x[currentx] / 3 * fine_move_level;
        }
        if(currenty%2==1) fine_move_offset= fine_move_1x[currentx]/3*fine_move_level;
        fine_height_offset=fine_height_level*80;
        if(currenty==7)  fine_height_offset=0;
        outtake_move.setPosition(finalx + fine_move_offset);
        outtake_slide(finaly+fine_height_offset, out_speed[1]);
        pause(100);
        fadj_key_reset=false;

    }


    public void outtake_slide(int targ, int vel) {
        if(targ<0) return;
        if (targ < 1801 ) {
            outtake_slider.setTargetPosition(targ);

            outtake_slider.setVelocity(vel);

        }
        else {
            outtake_slider.setTargetPosition(1800);
            outtake_slider.setVelocity(vel);
        }
    }



    public final void pause(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    public void moveRobot(double x, double y, double yaw, int intake) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        front_left.setPower(leftFrontPower*intake);
        front_right.setPower(rightFrontPower*intake);
        rear_left.setPower(leftBackPower*intake);
        rear_right.setPower(rightBackPower*intake);
    }


    public void pidouttake(double target ,int dir)

    {
       double dis,  xgap=10,x,xgain=0.05,xmax=0.24;

       while(xgap>2) {
           dis = out_left_dis.getDistance(DistanceUnit.CM);
           xgap = dis - target;

//       headingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));

//        yawError = desiredTag.ftcPose.x - 2.0; //-6.6
           x = Range.clip(xgap * xgain, -xmax, xmax);
           moveRobot(x, 0, 0, dir);
       }
       stop_drive();

//        strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//        turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

    }



    private boolean setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

//        if (visionPortal == null) {
//            return false;
//        }
 // pause(3000);
        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
////            telemetry.addData("Camera", "Waiting");
////            telemetry.addData("Current state",visionPortal.getCameraState());
////            telemetry.update();
           if ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                pause(20);
            }
           else  {
               ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
               if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                   exposureControl.setMode(ExposureControl.Mode.Manual);
                   pause(50);
               }
               exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
               pause(20);
               GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
               gainControl.setGain(gain);
               pause(20);
               april_ready=true;
               return true;
           }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();


        // Set camera controls unless we are stopping.
//        if (!isStopRequested())

        return false;
    }


    public boolean iniAprilTag(WebcamName cam) {

        if(init_setup) {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                    .build();
//"focalLength="822.317f, 822.317f"principalPoint="319.495f, 242.502f
            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(3);

            // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(cam)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTag)
                    .build();
            init_setup=false;
        }
      // return setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        return setManualExposure(6, 190);  // Use low exposure time to reduce motion blur
//to check

    }



    public boolean april_align( int id) {
        double dis = 0;
        double april_strafe;
//        double s_gain = SPEED_GAIN;


        targetFound = false;
        desiredTag = null;

        if (side_align){
            id = id+2;
            april_strafe = april_right_strafe;
        }
        else april_strafe = april_left_strafe;



            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {

                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {

                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == id)) {//(DESIRED_TAG_ID < 0) ||
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                      //  if(acal) {ax=desiredTag.ftcPose.x;ay=out_right_dis.getDistance(DistanceUnit.INCH)-10;acal=false;return true;}
                        targetFound = true;
                        break;  // don't look any further.
                    }

                }
            }


            if (targetFound) {
                dis = out_left_dis.getDistance(DistanceUnit.CM);

                if (dis > 1000) dis = 400;
                rangeError = dis - DESIRED_DISTANCE_T;
                headingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
                yawError = desiredTag.ftcPose.bearing + april_strafe;
//                yawError = desiredTag.ftcPose.x - 2.0; //-6.6
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);


                moveRobot(drive, strafe, turn,-1);//outake =-1, intake=1;

            }
            else {
                driving=true;
                return false;

            }




        return false;
    }


    public boolean husk_read() {
        // double x = 0;
        intake_handle(2);
        pause(800);
        int READ_PERIOD = 1,step=0;
        x=0;
        y=grab_dis.getDistance(DistanceUnit.MM);
        timer3(0);



        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);


        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
//            if (!rbg.huskyLens.knock()) {
//             //   telemetry.addData(">>", "Problem communicating with " + rbg.huskyLens.getDeviceName());
//            } else {
//                telemetry.addData(">>", "Press start to continue");
//            }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        //   telemetry.update();


        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (!timer3(800)) {
//            if (!rateLimit.hasExpired()) {
//               continue;
//            }
//            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */

            // HuskyLens.Block[] blocks = huskyLens.blocks();
            if(step==0)  { if (outtake_slider.getCurrentPosition()>(out_height[2]-5))
            {
                outtake_handle.setPosition(out_handle[4]);
                out_handle_in=true;
                outtake_cam.setPosition(out_cam[0]);
                step=1;
                timer2(0);
            }
            }
            if(step==1) {
                if (timer2(550)) {
                    outtake_slide(out_height[1], 1);
                    step=2;
                }
            }
            HuskyLens.Block[] blocks = huskyLens.blocks();
//            if (blocks[0].x > 0) x = blocks[0].x;
            // telemetry.addData("Block count", blocks.length);


            for (int i = 0; i < blocks.length; i++) {
                // telemetry.addData("Block", blocks[i].toString());
                if( blocks[i].x>0) x = blocks[i].x;


//                telemetry.addData("x", blocks[i].x);
//                telemetry.addData("y", blocks[i].y);

            }


            // telemetry.addData("x", x);
            //  telemetry.update();


            if (x > 0) HyawError = (x - husk_tar);// 110 iss the target x valule for alignment
            else HyawError=0;

            // HrangeError = grab_dis.getDistance(DistanceUnit.CM) - 25;//target distance to wall


            HheadingError = base_align_angle - imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));

            //Hdrive = Range.clip(HrangeError * HSPEED_GAIN, -HMAX_AUTO_SPEED, HMAX_AUTO_SPEED);
           // Hdrive=0;
            Hstrafe = Range.clip(-HyawError * HSTRAFE_GAIN, -HMAX_AUTO_STRAFE, HMAX_AUTO_STRAFE);
            Hturn = Range.clip(HheadingError * HTURN_GAIN, -HMAX_AUTO_TURN, HMAX_AUTO_TURN);


         //  moveRobot(Hdrive, Hstrafe, Hturn,1);

        }
        return false;
    }


    void coordinate_input(boolean tri, boolean cir, boolean cross,boolean square)
        {
//
            if((!tri&& !cir&& !cross&&!square)||(input_count>1)) return;
            int innum=0;
            if(tri) {
                innum=1 ;
            }
            else if (cir) {
                innum=2;
            }
            else if (cross) {
                innum=3;
            }
            else if (square) {
                innum=4;
            }
            else return;
            if(tempinput!= innum) {
                tempinput+=innum;
                input_count++;
                timerinput(0);
            }
           }
    void coordinate_input2(boolean tri, boolean cir, boolean cross,boolean square, boolean dpad_up,boolean dpad_right, boolean dpad_down)
    {
//
        if((!tri&& !cir&& !cross&&!square&&!dpad_up&&!dpad_right&&!dpad_down)) return;
        int innum=0;
        if(tri) {
            innum=1 ;
        }
        else if (cir) {
            innum=2;
        }
        else if (cross) {
            innum=3;
        }
        else if (square) {
            innum=4;
        }
        else if(dpad_up){
            innum = 5;
        }
        else if(dpad_right){
            innum = 6;
        }
        else if(dpad_down){
            innum = 7;
        }
        else return;
        tempinput = innum;
        timerinput(0);

    }


    boolean input_confirm(float f1x,float f1y)
    {
                if(f1x>0 && f1y > 0){
                    if (tempinput > 6) tempinput = 6;
                    input1y = tempinput;
                    cor_updating1=true;
                    tempinput = 0;
                    printing=true;
                    touching=false;
                    return true;

                }
                if (f1x>0 && f1y < 0){
                    input1x = tempinput;
                    cor_updating1=true;
                    tempinput = 0;
                    touching=false;
                    printing=true;
                    return true;

                }
                if(f1x<0 && f1y > 0){
                    if (tempinput > 6) tempinput = 6;
                    input2y = tempinput;
                    cor_updating2=true;
                    tempinput = 0;
                    touching=false;
                    printing=true;
                    return true;
                }
                if (f1x<0 &&f1y < 0){
                    input2x = tempinput;
                    cor_updating2=true;
                    tempinput = 0;
                    touching=false;
                    printing=true;
                    return true;
                }


            touching=false;
            return false;

        }
    boolean input_confirm2(boolean x1,boolean y1, boolean x2, boolean y2)
    {
        if(y1){
            input1y = tempinput;
            cor_updating1=true;
            tempinput = 0;
            input_count= 0;
            printing=true;
            touching=false;
            return true;

        }
        if (x1){
            input1x = tempinput;
            cor_updating1=true;
            tempinput = 0;
            input_count= 0;
            touching=false;
            printing=true;
            return true;

        }
        if(y2){
            input2y = tempinput;
            cor_updating2=true;
            tempinput = 0;
            input_count= 0;
            touching=false;
            printing=true;
            return true;
        }
        if (x2){
            input2x = tempinput;
            cor_updating2=true;
            tempinput = 0;
            input_count= 0;
            touching=false;
            printing=true;
            return true;
        }


        touching=false;
        return false;

    }

       boolean outtake_update() {
            if (prev_side_align!=side_align){
                cor_updating1 = true;
            }

            if((drop_qty==2 && cor_updating1 ) )
             {
                 stop_drive();
                finalxy();
                outtake_slide(finaly, 2);
                outtake_move.setPosition(finalx);
               pause(500);
               return true;
            }


        return false;
        }



}