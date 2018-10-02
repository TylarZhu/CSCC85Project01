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

	  * Failure modes: 0 - Nothing ever fails, life is simple
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

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

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
#include <stdbool.h>
#include <iostream>
#include <math.h>

#include "Lander_Control.h"

/************Sensor Fail *********************/
bool X_OK=true,Y_OK=true,VX_OK=true,VY_OK=true,TH_OK=true,Sonar_OK=true,Angle_Flag=true;

int T=15,l=0,n=1,Rotate_n=0,XT=15;

float Position_Y_test=0;

float S_X[30],X_Current=0,X_VX_Current=0,X_Past=0,RMS_X=0,RMS_Past_X=0,Start_X=0,Position_X_N,RMS_XT=0,RMS_VXT=0,sp_px=0.37,bound_x=5;
float S_Y[30],Y_Current=0,Y_VY_Current=0,Y_Past=0,RMS_Y=0,RMS_Past_Y=0,Start_Y=0,Position_Y_N,RMS_YT=0,RMS_VYT=0,sp_py=0.37,bound_y=5;
float S_VX[30],VX_Current=0,VX_X_Current=0,VX_Past=0,VX_X_Past=0,RMS_VX=0,RMS_Past_VX=0,Start_VX=0,Velocity_X_N,sp_vx=0.37,bound_vx=10;
float S_VY[30],VY_Current=0,VY_Y_Current=0,VY_Past=0,VY_Y_Past=0,RMS_VY=0,RMS_Past_VY=0,Start_VY=0,Velocity_Y_N,sp_vy=0.80,bound_vy=10;
float S_TH[30],TH_Current=0,TH_Past=0,RMS_TH=0,RMS_Past_TH=0,Start_TH=0,TH_Position_N,TH_PAST_Node=0,RMS_THT=0,Rotate_sp=0.442,bound_th=2;

void Position_A_X(void){
  float temp=0;
  if(l<T){
      S_X[l]=Position_X();
      X_Current+=Position_X();
      X_VX_Current+=Velocity_X();
      return;
  }else{
      X_Current=X_Current/l;
      X_VX_Current=X_VX_Current/l;
      for(int i=0;i<l;i++){
          RMS_XT=RMS_XT+(S_X[i]-X_Current)*(S_X[i]-X_Current);
      }
      RMS_XT=RMS_XT/l;
      RMS_XT=sqrtf(RMS_XT);
      RMS_X=RMS_XT;
      //        n=n+1;
      if(n<=2) temp = 0;
      else temp=fabs(RMS_X-RMS_Past_X)/fabs(RMS_Past_X);
      printf("RMS_PAST_X/RMS_X= %f\n",temp);
      
      RMS_Past_X=RMS_X;
      
      
      /*   if(temp<1 and n<3) {
        Position_X_N=X_Current;
        printf("X_Start =......... %f",Position_X_N);
        }else{
        Position_X_N = Position_X_N + VX_Current*sp_px;
        }*/
      
      if(temp>bound_x and X_OK) {
          X_OK = false;
          Position_X_N = X_Past;
          printf("Position X: Fail....\n");
          printf("Start Position X: %f\n",Position_X_N);
      }
      
      if(!X_OK){
          Position_X_N = Position_X_N + X_VX_Current*sp_px;//sp is sampling time
      }
      
      RMS_XT=0;
      X_Past=X_Current;
      X_Current=Position_X();
      S_X[0]=Position_X();
      
      X_VX_Current=Velocity_X();
      //        S_X_VX[0]=Velocity_X();
      
      //        l=1;
      
      return;
  }
}

/*
the horizontal velocity sensor fail
*/
void Velocity_A_X(void){
  float temp=0;
  if(l<T){
    // add up all the current position
    VX_X_Current+=Position_X();
    S_VX[l]=Velocity_X();
    VX_Current+=Velocity_X();
    return;
  }else{
    // calculate the mean
    VX_X_Current=VX_X_Current/l;
    VX_Current=VX_Current/l;
    // calculate the SD
    for(int i = 0; i < l; i ++){
        RMS_VXT = RMS_VXT + (S_VX[i] - VX_Current) * (S_VX[i] - VX_Current);
    }
    RMS_VXT = RMS_VXT/l;
    RMS_VXT = sqrtf(RMS_VXT);
    RMS_VX = RMS_VXT;
    
    // calculate the rate of past SD to current SD
    if(n<=2) temp = 0;
    else temp = fabs(RMS_VX - RMS_Past_VX) / fabs(RMS_Past_VX);
    printf("RMS_PAST_VX/RMS_VX= %f\n",temp);
    
    RMS_Past_VX=RMS_VX;
    //is the rate of the SD is greater then 10, meaning the horizontal velocity sensor fail
    if(temp > bound_vx and VX_OK) {
      VX_OK = false;
      printf("velocity X: Fail....\n");
    }
  
    if(!VX_OK){
      // calculate the current velocity where the sp is the sampling time
      Velocity_X_N = (VX_X_Current-VX_X_Past)/sp_vx;
    }
    
    RMS_VXT=0;
    VX_Current=Velocity_X();
    S_VX[0]=Velocity_X();
    VX_X_Past=VX_X_Current;
    VX_Past=VX_Current;
    return;
  }
}


void Position_A_Y(void){
    float temp=0;
    if(l<T){
        S_Y[l]=Position_Y();
        Y_Current+=Position_Y();
        Y_VY_Current+=Velocity_Y();
        return;
    }else{
        Y_Current=Y_Current/l;
        Y_VY_Current=Y_VY_Current/l;
        //To calculation RMS
        for(int i=0;i<l;i++){
            RMS_YT=RMS_YT+(S_Y[i]-Y_Current)*(S_Y[i]-Y_Current);
        }
        RMS_YT=RMS_YT/l;
        RMS_YT=sqrtf(RMS_YT);
        RMS_Y=RMS_YT;
        
        if(n<=2) temp = 0;
        else temp=fabs(RMS_Y-RMS_Past_Y)/fabs(RMS_Past_Y);
        printf("RMS_PAST_Y/RMS_Y= %f\n",temp);
        
        RMS_Past_Y=RMS_Y;
        
        /*
         if(temp<1 and n<3) {
         Position_Y_test=Y_Past;
         printf("Y_Start =......... %f\n",Position_Y_test);
         }else{
         Position_Y_test = Position_Y_test + Y_VY_Current*(-1)*sp_py;
         }
         */
        
        if(temp>bound_y and Y_OK) {
            Y_OK = false;
            Position_Y_N = Y_Past;
            printf("Position Y: Fail....\n");
            printf("Start Position X: %f\n",Position_Y_N);
        }
        
        if(!Y_OK){
            Position_Y_N = Position_Y_N + Y_VY_Current*(-1)*sp_py;//sp is sampling time
        }
        /*
         printf("Positin_Y    : %f\n",Y_Current);
         //         printf("Position_Y_T : %f\n",Position_Y_test);
         printf("Position_Y_N : %f\n",Position_Y_N);
         printf("VY_Current   : %f\n",Y_VY_Current);
         printf("sp_y=        : %f\n",(Y_Current-Y_Past)/Y_VY_Current);
         
         printf("RMS_Y:    %f\n",RMS_Y);
         printf("------------------------------\n");
         */
        
        
        RMS_YT=0;
        Y_Past=Y_Current;
        Y_Current=Position_Y();
        S_Y[0]=Position_Y();
        Y_VY_Current=Velocity_Y();
        
        return;
    }
}

void Velocity_A_Y(void){
    float temp=0;
    if(l<T){
        VY_Y_Current+=Position_Y();
        
        S_VY[l]=Velocity_Y();
        VY_Current+=Velocity_Y();
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
        
        //compare two step RMS
        if(n<=2) temp = 0;
        else temp=fabs(RMS_VY-RMS_Past_VY)/fabs(RMS_Past_VY);
        printf("RMS_PAST_VY/RMS_VY= %f\n",temp);
        
        RMS_Past_VY=RMS_VY;
        
        if(temp>bound_vy and VY_OK) {
            VY_OK = false;
            printf("Velocity Y: Fail....\n");
        }
        
        if(!VY_OK){
            Velocity_Y_N =(VY_Y_Current-VY_Y_Past)*(-1.0)/sp_vy;//sp is the sampling time
        }
        VY_Past=VY_Current;
        RMS_VYT=0;
        VY_Current=Velocity_Y();
        S_VY[0]=Velocity_Y();
        VY_Y_Past=VY_Y_Current;
        
        
        return;
    }
}

void Position_A_TH(void){
    float temp=0;
    float Angle_temp=0;
    
    if(l<T){
        //Angle convert to 360---> -angle;
        Angle_temp=Angle();
        //        printf("Angle_temp      : %f\n",Angle_temp);
        if(l==0) {
            S_TH[0]=Angle_temp;
            TH_Current+=Angle_temp;
        }else{
            if(fabs(Angle_temp-S_TH[l-1])>90){
                if(S_TH[l-1]<180) {
                    Angle_temp=Angle_temp-360;
                }else{
                    Angle_temp=360+Angle_temp;
                }
                
            }
            S_TH[l]=Angle_temp;
            TH_Current+=Angle_temp;
        }
        
        //        printf("Angle_temp after: %f\n",Angle_temp);
        //     l=l+1;
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
        n=n+1;
        if(n<=2) temp = 0;
        else temp=fabs(RMS_TH-RMS_Past_TH)/fabs(RMS_Past_TH);
        printf("RMS_PAST_X/RMS_X= %f\n",temp);
        
        RMS_Past_TH=RMS_TH;
        /*        if(temp<1 and n<3) {
         Start_TH=TH_Current;
         printf("TH_Start =......... %f\n",Start_TH);
         }else{
         TH_Position_N = Start_TH + Rotate_n*Rotate_sp;
         if(TH_Position_N<0) TH_Position_N=360+TH_Position_N;
         if(TH_Position_N>360) TH_Position_N=TH_Position_N-360;
         }*/
        
        if(temp>bound_th and TH_OK) {
            TH_OK = false;
            Start_TH = TH_Past;
            if(Start_TH<0) Start_TH=360+Start_TH;
            if(Start_TH>360) Start_TH=Start_TH-360;
            printf("Position TH: Fail....\n");
            printf("Start Position TH: %f\n",Start_TH);
        }
        
        
        if(!TH_OK){
            TH_Position_N = Start_TH + Rotate_n*Rotate_sp;
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
        
       // l=1;
        return;
    }
}

/*************Sensor Fail End*************************/

//********************Thruster Fail*******************//



//For set degree
//check the Thruster state to set landing action

bool MT_OK_N = true,RT_OK_N=true,LT_OK_N=true;

void setMode(void)
{
    if(MT_OK && RT_OK && LT_OK) {
        MT_OK_N=MT_OK;
        RT_OK_N=RT_OK;
        LT_OK_N=LT_OK;
        return;
    
    }else{
    if(MT_OK){
        printf("MT_OK...\n");
        if(RT_OK) RT_OK_N=!RT_OK;
        if(LT_OK) LT_OK_N=!LT_OK;
    }else{
        printf("MT_Fail...\n");
        if(RT_OK){
            printf("RT_OK...\n");
            if(LT_OK){
                printf("LT_OK...\n");
                MT_OK_N=MT_OK;
                RT_OK_N=RT_OK;
                LT_OK_N=!LT_OK;
            }else{
                printf("LT_Fail...\n");
                MT_OK_N=MT_OK;
                RT_OK_N=RT_OK;
                LT_OK_N=LT_OK;
            }
            
        }else{
            printf("MT_Fail...\n");
            printf("RT_Fail...\n");
            printf("LT_OK...\n");
            MT_OK_N=MT_OK;
            RT_OK_N=RT_OK;
            LT_OK_N=LT_OK;
        }
        
    }
    }
    
}

void stay_X_degree(double X) {
    if (fabs(Angle()-X) > 1) {
        if (X >= 270 && Angle()<=90) Rotate(-Angle()+X-360);
        else if (Angle() >= X + 180) Rotate(360 - Angle() + X);
        else Rotate(X - Angle());
        return;
    }
}



void Robust_Right_Thruster(double power) {
    printf("Want to active Right Thruster\n");
    if  (RT_OK_N) {
        //RT is ok
        stay_X_degree(0);
/*        if(Angle()<350 and Angle()>5){
            Main_Thruster(0);
            Right_Thruster(0);
            Left_Thruster(0);
            return;
        }*/
        
        
        Left_Thruster(0);
        Right_Thruster(power);
     
    } else if (MT_OK_N) {//RT is failure. use MT(360-90) or LT (180)
        stay_X_degree(360-75);
        if(Angle()>290 and Angle()<275) return;
        Main_Thruster(power);
    } else {
        // Exceeded velocity limit, brake
        stay_X_degree(360-175);
        float x=Angle();
        printf("Want to active Right Thruster Ag = %f\n",x);
        if(Angle()>200 and Angle()<165){
            Left_Thruster(0);
            Right_Thruster(0);
            return;
        }
        
        Right_Thruster(0);
        Left_Thruster(power);
    }
    return;
}

void Robust_Left_Thruster(double power) {
    printf("Want to active Left Thruster\n");
    if  (LT_OK_N) {
        //LT is ok
        stay_X_degree(0);
        float x=Angle();
        printf("Want to active Left Thruster Ag = %f\n",x);
/*      if(Angle()<350 and Angle()>5){
            return;
        }*/
        Right_Thruster(0);
        Left_Thruster(power);
        
    } else if (MT_OK_N) {//LT is failure. use MT(90) or RT (180)
        stay_X_degree(75);
        if(Angle()>50 and Angle()<90) return;
        Main_Thruster(power);
    } else {
        // LT and MT is fail
        stay_X_degree(175);
        float x=Angle();
        printf("Want to active Left Thruster Ag = %f\n",x);
        if(Angle()<170 and Angle()>180){
            Left_Thruster(0);
            Right_Thruster(0);
            return;
        }
        Left_Thruster(0);
        Right_Thruster(power);
    }
    return;
}

void Robust_Main_Thruster(double power) {
    printf("Want to active Main Thruster\n");
    if  (MT_OK_N) {
        //LT is ok
        stay_X_degree(0);
 /*       if(Angle()<350 and Angle()>5){
            Main_Thruster(0);
            Right_Thruster(0);
            Left_Thruster(0);
            return;
        }*/
            Main_Thruster(power);
    } else if (RT_OK_N) {//MT is failure. use RT(90)
        stay_X_degree(90);
        float x=Angle();
        printf("Want to active Main Thruster Ag = %f\n",x);
        if(Angle()>105 and Angle()<75){
            Left_Thruster(0);
            Right_Thruster(0);
            return;
        }
        Left_Thruster(0);
        Right_Thruster(power);
    } else {
        // LT and MT is fail
        stay_X_degree(360-90);
        if(Angle()>340 and Angle()<255){
            Right_Thruster(0);
            Left_Thruster(0);
            return;
        }
        Right_Thruster(0);
        Left_Thruster(power);
    }
    return;
}

float last_d=40;//land distance

//*******************zhu*******************//


void Lander_Control(void)
{
    /******Sensor Fail*************/
    if(l<T){//T is the period for sampling
        Position_A_X();
        Position_A_Y();
        Velocity_A_X();
        Velocity_A_Y();
        l=l+1; //l is the sampling length
    }else{
        n=n+1;//test first
        Position_A_X();
        Position_A_Y();
        Velocity_A_X();
        Velocity_A_Y();
        l=1;
    }
    
    if(X_OK) Position_X_N=Position_X();//X_Past;
    if(Y_OK) Position_Y_N=Position_Y();//Y_Past;
    if(VX_OK)Velocity_X_N=Velocity_X();//VX_Past;
    if(VY_OK)Velocity_Y_N=Velocity_Y();
    if(TH_OK)TH_Position_N=Angle();
    
    
    
    /******************************/
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

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
    setMode();
 if (fabs(Position_X_N-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X_N-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y_N>200) VYlim=-20;
 else if (PLAT_Y-Position_Y_N>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X_N)/fabs(Velocity_X_N)>1.25*fabs(PLAT_Y-Position_Y_N)/fabs(Velocity_Y_N)) VYlim=0;

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

/* if (Angle()>1&&Angle()<359)
 {
  if (Angle()>=180) Rotate(360-Angle());
  else Rotate(-Angle());
  return;
 }*/
 //   stay_X_degree(0);
    
    
    //*******zhu***********//
    //Too close surface in Vertical direction, no action.
    printf("last distance X:.......%f\n",(PLAT_X-Position_X_N));
    printf("last distance Y:.......%f\n",(PLAT_Y-Position_Y_N));
    if((PLAT_X-Position_X_N)>320){
        Robust_Left_Thruster(1.0);
        return;
    }
    if((PLAT_X-Position_X_N)<-320){
        Robust_Right_Thruster(1.0);
        return;
    }
    if((PLAT_Y-Position_Y_N)>790){
        Robust_Main_Thruster(0);
        return;
    }
    
    if(fabs(PLAT_Y-Position_Y_N)<last_d){
        stay_X_degree(0);
        Left_Thruster(0);
        Right_Thruster(0);
        Main_Thruster(0);
        float x=Angle();
        printf("Want to Landing.... Ag = %f Y= %f\n",x,PLAT_Y-Position_Y_N);
        
        return;
    }
    
 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (Position_X_N>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
     if (Velocity_X_N>(-VXlim)) Robust_Right_Thruster((VXlim+fmin(0,Velocity_X_N))/VXlim);
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
   Robust_Right_Thruster(fabs(VXlim-Velocity_X_N));//Right_Thruster(fabs(VXlim-Velocity_X_N));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (Velocity_Y_N<VYlim) Robust_Main_Thruster(1.0);
 else Main_Thruster(0);
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
 Vmag=Velocity_X_N*Velocity_X_N;
 Vmag+=Velocity_Y_N*Velocity_Y_N;

 DistLimit=fmax(75,Vmag);

 // If we're close to the landing platform, disable
 // safety override (close to the landing platform
 // the Control_Policy() should be trusted to
 // safely land the craft)
 if (fabs(PLAT_X-Position_X_N)<150&&fabs(PLAT_Y-Position_Y_N)<150) return;

 // Determine the closest surfaces in the direction
 // of motion. This is done by checking the sonar
 // array in the quadrant corresponding to the
 // ship's motion direction to find the entry
 // with the smallest registered distance

 // Horizontal direction.
 dmin=1000000;
 if (Velocity_X_N>0)
 {
  for (int i=5;i<14;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=22;i<32;i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 // Determine whether we're too close for comfort. There is a reason
 // to have this distance limit modulated by horizontal speed...
 // what is it?
 if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X_N)/5.0,1)))
 { // Too close to a surface in the horizontal direction
     if(fabs(PLAT_Y-Position_Y_N)<30){
 
  if (Angle()>1&&Angle()<359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
     }
     
  if (Velocity_X_N>0){
   Robust_Right_Thruster(1.0);
   Left_Thruster(0.0);
  }
  else
  {
   Robust_Left_Thruster(1.0);
   Right_Thruster(0.0);
  }
 }

 // Vertical direction
 dmin=1000000;
 if (Velocity_Y_N>5)      // Mind this! there is a reason for it...
 {
  for (int i=0; i<5; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
  for (int i=32; i<36; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 else
 {
  for (int i=14; i<22; i++)
   if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
 }
 if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
 {

    if(fabs(PLAT_Y-Position_Y_N)<30){

  if (Angle()>1||Angle()>359)
  {
   if (Angle()>=180) Rotate(360-Angle());
   else Rotate(-Angle());
   return;
  }
    }
  if (Velocity_Y_N>2.0){
   Main_Thruster(0.0);
  }
  else
  {
   Robust_Main_Thruster(1.0);
  }
 }
}
