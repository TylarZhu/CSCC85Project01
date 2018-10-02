/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 5 m/s at touchdown
	  * Maximum landing angle should be less than 10 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails,  life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8
PIDX_realizeInc(PLAT_X) - PLAT_X)
PIDX_realizeInc(PLAT_X) - PLAT_X)m' map, and disables the main thruster,
PIDX_realizeInc(PLAT_X) - PLAT_X)sensor.
PIDX_realizeInc(PLAT_X) - PLAT_X)
		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <iostream>
#include "Lander_Control.h"

using namespace std;



bool X_OK=true,Y_OK=true,VX_OK=true,VY_OK=true,TH_OK=true,Sonar_OK=true,Angle_Flag=true;

int T=15,l=0,n=1,Rotate_n=0,XT=15;

double Position_Y_test=0;

double S_X[30],X_Current=0,X_VX_Current=0,X_Past=0,RMS_X=0,RMS_Past_X=0,Start_X=0,Position_X_N,RMS_XT=0,RMS_VXT=0,sp_px=0.37,bound_x=5;
double S_Y[30],Y_Current=0,Y_VY_Current=0,Y_Past=0,RMS_Y=0,RMS_Past_Y=0,Start_Y=0,Position_Y_N,RMS_YT=0,RMS_VYT=0,sp_py=0.37,bound_y=5;
double S_VX[30],VX_Current=0,VX_X_Current=0,VX_Past=0,VX_X_Past=0,RMS_VX=0,RMS_Past_VX=0,Start_VX=0,Velocity_X_N,sp_vx=0.37,bound_vx=10;
double S_VY[30],VY_Current=0,VY_Y_Current=0,VY_Past=0,VY_Y_Past=0,RMS_VY=0,RMS_Past_VY=0,Start_VY=0,Velocity_Y_N,sp_vy=0.80,bound_vy=10;
double S_TH[30],TH_Current=0,TH_Past=0,RMS_TH=0,RMS_Past_TH=0,Start_TH=0,TH_Position_N,TH_PAST_Node=0,RMS_THT=0,Rotate_sp=0.442,bound_th=2;

void Velocity_A_X(void){
    double temp=0;
    if(l<T){
        VX_X_Current+=Position_X();
        
        S_VX[l]=Velocity_X();
        VX_Current+=Velocity_X();
        //        l=l+1;
        return;
    }else{
        VX_X_Current=VX_X_Current/l;
        VX_Current=VX_Current/l;
        
        //To calculation RMS
        for(int i=0;i<l;i++){
            RMS_VXT=RMS_VXT+(S_VX[i]-VX_Current)*(S_VX[i]-VX_Current);
        }
        
        RMS_VXT=RMS_VXT/l;
        RMS_VXT=sqrtf(RMS_VXT);
        RMS_VX=RMS_VXT;
        
        //compare two step RMS
        //        n=n+1;
        if(n<=2) temp = 0;
        else temp=fabs(RMS_VX-RMS_Past_VX)/fabs(RMS_Past_VX);
        cout << "RMS_PAST_VX/RMS_VX= " << temp << endl;
        
        RMS_Past_VX=RMS_VX;
        
        /*       if(temp<1 and n<3) {
         Velocity_X_N=VX_Current;
         printf("VX_Start =......... %f\n",Velocity_X_N);
         }else{
         Velocity_X_N =(X_Current-X_Past)/sp_vx;
         }*/
        
        if(temp>bound_vx && VX_OK) {
            VX_OK = false;
            cout << "Velocity X: Fail...." << endl;
        }
        
        if(!VX_OK){
            Velocity_X_N =(VX_X_Current-VX_X_Past)/sp_vx;//sp is the sampling time
        }
        
        
        
        /*
         if(RMS_VX>2 and VX_OK) {
         VX_OK = false;
         Start_VX = VX_Past;
         printf("VX: Fail....\n");
         printf("Start VX: %f\n",Start_VX);
         }*/
        
        RMS_VXT=0;
        VX_Current=Velocity_X();
        S_VX[0]=Velocity_X();
        VX_X_Past=VX_X_Current;
        VX_Past=VX_Current;
        
        //               l=1;
        
        return;
    }
}


void Velocity_A_Y(void){
    double temp=0;
    if(l<T){
        VY_Y_Current+=Position_Y();
        
        S_VY[l]=Velocity_Y();
        VY_Current+=Velocity_Y();
        //        l=l+1;
        return;
    }else{
        VY_Y_Current=VY_Y_Current/l;
        VY_Current=VY_Current/l;
        
        //To calculation RMS
        for(int i=0;i<l;i++){
            RMS_VYT=RMS_VYT+(S_VY[i]-VY_Current)*(S_VY[i]-VY_Current);
        }
        
        RMS_VYT=RMS_VYT/l;
        RMS_VYT=sqrtf(RMS_VYT);
        RMS_VY=RMS_VYT;
        
        if(n<=2) temp = 0;
        else temp=fabs(RMS_VY-RMS_Past_VY)/fabs(RMS_Past_VY);
        cout << "RMS_PAST_VX/RMS_VX= " << temp << endl;
        
        RMS_Past_VY=RMS_VY;
        
        if(temp>bound_vy && VY_OK) {
            VY_OK = false;
            cout << "Velocity Y: Fail...." << endl;
        }
        
        if(!VY_OK){
            Velocity_Y_N =(VY_Y_Current-VY_Y_Past)*(-1.0)/sp_vy;//sp is the sampling time
        }
        VY_Past=VY_Current;
        RMS_VYT=0;
        VY_Current=Velocity_Y();
        S_VY[0]=Velocity_Y();
        VY_Y_Past=VY_Y_Current;
        
        
        //               l=1;
        
        return;
    }
}

void Position_A_TH(void){
    double temp=0;
    double Angle_temp=0;
    if(l<T){
        //Angle convert to 360---> -angle;
        Angle_temp=Angle();
        if(fabs(Angle_temp-S_TH[l-1])>90){
            if(S_TH[l-1]<180) {
                Angle_temp=Angle_temp-360;
            }else{
                Angle_temp=360+Angle_temp;
            }
        }
        S_TH[l]=Angle_temp;
        TH_Current+=Angle_temp;
        //l=l+1;
        return;
    }else{
        TH_Current=TH_Current/l;
        
        //To calculation RMS
        for(int i=0;i<l;i++){
            RMS_THT=RMS_THT+(S_TH[i]-TH_Current)*(S_TH[i]-TH_Current);
        }
        RMS_THT=RMS_THT/l;
        RMS_THT=sqrtf(RMS_THT);
        RMS_TH=RMS_THT;
        
        //n=n+1;
        if(n<=2) temp = 0;
        else temp=fabs(RMS_TH-RMS_Past_TH)/fabs(RMS_Past_TH);
        //cout << "RMS_PAST_X/RMS_X= " << temp << endl;
        
        RMS_Past_TH=RMS_TH;
        if(temp>bound_th && TH_OK) {
            TH_OK = false;
            Start_TH = TH_Past;
            if(Start_TH<0) Start_TH=360+Start_TH;
            if(Start_TH>360) Start_TH=Start_TH-360;
            cout << "TH: Fail...." << endl;
            cout << "Start Position TH: " << Start_TH << endl;
        }
        
        
        if(!TH_OK){
            TH_Position_N = Start_TH + Rotate_sp;
            if(TH_Position_N<0) TH_Position_N=360+TH_Position_N;
            if(TH_Position_N>360) TH_Position_N=TH_Position_N-360;
        }else{
            
            TH_Position_N=TH_Current;
            if(TH_Position_N<0) TH_Position_N=360+TH_Position_N;
            if(TH_Position_N>360) TH_Position_N=TH_Position_N-360;
        }
        
        
        RMS_THT=0;
        TH_Past=TH_Current;
        Angle_temp=Angle();
        TH_Current=Angle_temp;
        S_TH[0]=Angle_temp;
        
        //l=1;
        return;
    }
}


void Exception(double, double);
void handelLeftThursterFail(double, double); 
double Too_close;

struct PID{
    double SetPosition;
    double ActualPosition;
    double err;
    double err_last;
    double err_next;
    double Kp, Ki, Kd;
    double voltage;
    double integral;
    double umax;
    double umin;
}pidX, pidY;

float PIDY_realizeInc(float optimalPosition) {
    pidY.ActualPosition = Position_Y();
    pidY.SetPosition = optimalPosition;
    pidY.err = pidY.SetPosition - pidY.ActualPosition;

    float Vlimit = pidY.Kp * (pidY.err - pidY.err_next)
                          + pidY.Ki * pidY.err 
                          + pidY.Kd * (pidY.err - 2 * pidY.err_next + pidY.err_last);
    
    pidY.ActualPosition += Vlimit;
    pidY.err_last = pidY.err_next;
    pidY.err_next= pidY.err;
    return pidY.ActualPosition;
}

float PIDX_realizeInc(float optimalPosition) {
    pidX.ActualPosition = Position_X();
    pidX.SetPosition = optimalPosition;
    pidX.err = pidX.SetPosition - pidX.ActualPosition;

    float Vlimit = pidX.Kp * (pidX.err - pidX.err_next)
                          + pidX.Ki * pidX.err 
                          + pidX.Kd * (pidX.err - 2 * pidX.err_next + pidX.err_last);
    
    pidX.ActualPosition += Vlimit;
    pidX.err_last = pidX.err_next;
    pidX.err_next = pidX.err;
    return pidX.ActualPosition;
}



void stay_X_degree(double X) {
    if (fabs(TH_Position_N-X)>2){
        double left = int(360 - TH_Position_N + X) % 360;
        double right = int(360 - X + TH_Position_N) % 360;
        if (left < right) Rotate(left);
        else Rotate(-right);
    }
}

void stay_zero_degree(void){
  stay_X_degree(0);
}

void Robust_Right_Thruster(double speed) {
    //printf("Want to active Right Thruster\n");
    Left_Thruster(0);
    if (Velocity_X()>(speed)) {
    if  (RT_OK) {

        if (!MT_OK) {
            if (LT_OK && TH_Position_N>265 && TH_Position_N<335) stay_X_degree(262);
            else stay_X_degree(30);
        }
        if (TH_Position_N>270 && TH_Position_N<320) return;
        Right_Thruster(speed);
    } else if ((!RT_OK) && MT_OK) {
        Main_Thruster(speed);
        stay_X_degree(360-90);
        
    } else {
        // Exceeded velocity limit, brake
        Right_Thruster(0);
        Left_Thruster(speed);
    }
    }
}

void Robust_Left_Thruster(double speed) {
    //printf("Want to active Left Thruster\n");
    Right_Thruster(0);
    if (Velocity_X()<speed){
    if (LT_OK) {

        if (!MT_OK) {
            if (RT_OK && TH_Position_N>25 && TH_Position_N<115) stay_X_degree(98);
            else stay_X_degree(360-30);
        }
        if (TH_Position_N<90 && TH_Position_N>40 ) return;
        Left_Thruster(speed);
    } else if (MT_OK) {
        stay_X_degree(90);
        Main_Thruster(speed);
    } else {
        Left_Thruster(0);
        Right_Thruster(speed);
    }
    }
}

void Robust_Main_Thruster(double VYlim){
    if (Velocity_Y() < VYlim){
        //printf("Want to active main Thruster\n");
        if (MT_OK){ 
            if (Position_X()<PLAT_X) stay_X_degree(8);
            else stay_X_degree(352);
            Main_Thruster(1);
        } else if (LT_OK && Position_X()<PLAT_X){
            stay_X_degree(278);
            Right_Thruster(0);
            if (TH_Position_N<90) return;
            Left_Thruster(1);
        } else if (RT_OK){
            stay_X_degree(82);
            Left_Thruster(0);
            if (TH_Position_N>270) return;
            Right_Thruster(1);
        }
    }else{
        Main_Thruster(0);
        Right_Thruster(0);
        Left_Thruster(0);
    }
}

void Lander_Control(void) {
    /*
    This is the main control function for the lander. It attempts
    to bring the ship to the location of the landing platform
    keeping landing parameters within the acceptable limits.

    How it works:

    - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
    - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
    - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

    As noted above, this function assumes everything is working
    fine.
    */

    /*************************************************
    TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
    DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
    **************************************************/
    double VXlim;
    double VYlim;

     /******Sensor Fail*************/
  if(l<T){//T is the period for sampling

      Velocity_A_X();
      Velocity_A_Y();
      Position_A_TH();
      l=l+1; //l is the sampling length
  }else{
      n=n+1;//test first

      Velocity_A_X();
      Velocity_A_Y();
      Position_A_TH();
      l=1;
  }
    
    if(VX_OK)Velocity_X_N=Velocity_X();//VX_Past;
    if(VY_OK)Velocity_Y_N=Velocity_Y();
    if(TH_OK)TH_Position_N=Angle();
    
    
    /*PIDX_realizeInc
    prPIDX_realizeInc: \n");
    prPIDX_realizeInc;
    prPIDX_realizeInc", Velocity_X());
    printf("VY: ");
    printf("%.2f\n", Velocity_Y());
    printf("position X:");
    printf("%.2f\n", Position_X()-PLAT_X);
    printf("position Y:");
    printf("%.2f\n", PLAT_Y-Position_Y());
    */

    // Set velocity limits depending on distance to platform.
    // If the module is far from the platform allow it to
    // move faster, decrease speed limits as the module
    // approaches landing. You may need to be more conservative
    // with velocity limits when things fail.

    //VXlim = 30 * fabs(PIDX_realizeInc(PLAT_X) - PLAT_X) / 1024;
    //VYlim = -20 * fabs(PIDY_realizeInc(PLAT_Y) - PLAT_Y) / 1024;
     if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y()>200) VYlim=-20;
 else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity
if (!MT_OK) {
    VYlim = VYlim / 2 ;
    VXlim = VXlim / 2 ;
}
    //printf("Too_close is:%.2f\n",Too_close);
    //if (Too_close == 0) VYlim = 0.7;
        
    
 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X())/fabs(Velocity_X_N)>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y_N)) VYlim=0;
/*
    printf("VXLim: ");
    printf("%.2f\n", VXlim);
    printf("VX: ");
    printf("%.2f\n", Velocity_X());
    
    printf("PositionX: ");
    printf("%.2f\n", fabs(Position_X() - PLAT_X));
    
    printf("PositionY: ");
    printf("%.2f\n", fabs(Position_Y() - PLAT_Y));
*/
    

    //Left_Thruster(0);
    //Right_Thruster(0);
    //Main_Thruster(0);
    // Ensure we will be OVER the platform when we land
    if (fabs(Position_Y() - PLAT_Y) < 25){
            stay_zero_degree();
            printf("Landing\n");
            //if (MT_OK) Robust_Main_Thruster(0); 
            return;
    }
    

    // IMPORTANT NOTE: The code below assumes all components working
    // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
    // fail. More likely, you will need a set of case-based code
    // chunks, each of which works under particular failure conditions.

    // Check for rotation away from zero degrees - Rotate first,
    // use thrusters only when not rotating to avoid adding
    // velocity components along the rotation directions
    // Note that only the latest Rotate() command has any
    // effect, i.e. the rotation angle does not accumulate
    // for successive calls.

    // Module is oriented properly, check for horizontal position
    // and set thrusters appropriately.
if (fabs(Position_X()-PLAT_X) > 50){
    if (Position_X()>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (Velocity_X_N>(-VXlim)) 
    Robust_Right_Thruster((VXlim+fmin(0,Velocity_X_N))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Robust_Left_Thruster(fabs(VXlim-Velocity_X_N));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (Velocity_X_N<VXlim) Robust_Left_Thruster((VXlim-fmax(0,Velocity_X_N))/VXlim);
  else
  {
   Left_Thruster(0);
   Robust_Right_Thruster(fabs(VXlim-Velocity_X_N));
  }
 }
}

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 /*
 printf("VYLim: ");
printf("%.2f\n", VYlim);
    printf("VY: ");
    printf("%.2f\n", Velocity_Y());
*/



 if (Velocity_Y_N>1) 
    Robust_Main_Thruster(VYlim);
 else Robust_Main_Thruster(VYlim);
 
}

void Safety_Override(void)
{
    /*
    This function is intended to keep the lander from
    crashing. It checks the sonar distance array,
    if the distance to nearby solid surfaces and
    uses thrusters to maintain a safe distance from
    the ground unless the ground happens to be the
    landing platform.

    Additionally, it enforces a maximum speed limit
    which when breached triggers an emergency brake
    operation.
    */

    /**************************************************
    TO DO: Modify this function so that it can do its
        work even if components or sensors
        fail
    **************************************************/

    /**************************************************
    How this works:
    Check the sonar readings, for each sonar
    reading that is below a minimum safety threshold
    AND in the general direction of motion AND
    not corresponding to the landing platform,
    carry out speed corrections using the thrusters
    **************************************************/

    double DistLimit;
    double Vmag;
    double dmin;

    // Establish distance threshold based on lander
    // speed (we need more time to rectify direction
    // at high speed)
    Vmag = Velocity_X_N * Velocity_X_N;
    Vmag += Velocity_Y_N * Velocity_Y_N;

    DistLimit = fmax(80, Vmag);
    if ((fabs(PLAT_X - Position_X()) < 150)&&(fabs(PLAT_Y - Position_Y())) < 200)
        return;
    // If we're close to the landing platform, disable
    // safety override (close to the landing platform
    // the Control_Policy() should be trusted to
    // safely land the craft)
   // if (fabs(PLAT_X - Position_X()) < 100)
    //    return;
    //if (fabs(PLAT_Y - Position_Y()) < 100)
    //    return;

    // Determine the closest surfaces in the direction
    // of motion. This is done by checking the sonar
    // array in the quadrant corresponding to the
    // ship's motion direction to find the entry
    // with the smallest registered distance

    // Horizontal direction.

    dmin=100000;
    if (Velocity_X_N > 0)
    {
        for (int i = 5; i < 14; i++) {
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin) {
                dmin = SONAR_DIST[i];
            }
        }
    } else {
        for (int i = 22; i < 32; i++) {
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
        }
    }
    // Determine whether we're too close for comfort. There is a reason
    // to have this distance limit modulated by horizontal speed...
    // what is it?
    if (dmin < DistLimit * fmax(.25, fmin(fabs(Velocity_X_N) / 5.0, 1))) { 
        // Too close to a surface in the horizontal direction
        //stay_zero_degree();
        if (Velocity_X_N > 0){

            Robust_Right_Thruster(2.0);
        } else {

            Robust_Left_Thruster(2.0);
        
        }
    }

    // Vertical direction
    dmin=100000;
    if (Velocity_Y_N>5) {      // Mind this! there is a reason for it...
        for (int i=0; i<5; i++) {
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
        }
        for (int i=32; i<36; i++) {
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
        }
    } else {
        for (int i=14; i<22; i++) {
            if (SONAR_DIST[i] > -1 && SONAR_DIST[i] < dmin)
                dmin = SONAR_DIST[i];
        }
    }
    if (dmin<DistLimit) {   // Too close to a surface in the horizontal direction

        Too_close = 0;
        if (Velocity_Y_N > 2.0) {
            Robust_Main_Thruster(0);
        } else {
            Robust_Main_Thruster(2.0);
        }
    }
}


