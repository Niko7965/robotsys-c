/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];
volatile int running=1;
int robot_port;
double speed;

FILE *fr;

double data_logging[6000][3];
double odometry_log[6000][3];
double laserpar_log[6000][10];
double linesensor_log[6000][8];
double theta;

//IRL values: 93, 52
const int line_white_value = 255;
const int line_black_value = 0;

int odometry_log_count;
int data_log_count;
int laserpar_log_count;
int linesensor_log_count;

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.2546	/* m */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define DEFAULT_ROBOTPORT	24902


typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
                double left_displacement;
		double right_displacement;
		double center_displacement;
		double position_x;
		double position_y;
		double angle;
		double delta_angle;
		} odotype;

/** tyupe definition for ir sensor posiutionb*/
typedef enum {
  LEFT = 0,
  FRONT_LEFT = 1,
  FRONT_CENTER = 2,
  FRONT_RIGHT = 3,
  RIGHT  = 4
} irposition;

double irsensorraw(irposition pos);
double irsensorcalibrated(irposition pos);

void reset_odo(odotype *p);
void update_odo(odotype *p);




/********************************************
* Motion control
*/

typedef struct{//input
                int cmd;
		int curcmd;
		int stoppingtype;
		double ir_max_dist;
		double ir_min_dist;
		double speedcmd;
	
		double dist;
		double angle;
		double left_pos,right_pos;
		// parameters
		//output
		double motorspeed_l,motorspeed_r; 
		double turning_radius;
		int finished;
		// internal variables
		double startpos;
		double w;
		
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn,mot_line,mot_line_l,mot_line_r,mot_turn_rad_l,mot_nop};
//stopping type
enum {distance,crossingline,distance_left,distance_mid,distance_right,min_ir_right};

void update_motcon(motiontype *p);	       


int line_position();
int follow_line(double speed,int time);
double line_map(double raw);
int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
double line_follow_simple();
double line_center_of_mass();
double line_center_left();
double line_center_right();
double squared_dist(double target, double value);
int turn_radial_left(double radius, double velocity, double angle, int time);
int crossing_black_line();
int should_stop();

void segfaulthandler(int sig)
{
//    perror(NULL);
   printf("Seg-error\n");
   exit(1);
}

void brokenpipehandler(int sig)
{
   printf("mrc: broken pipe \n");
  // savelog("log");
   exit(1);
}

void ctrlchandler(int sig)
{
   printf("mrc: ctrl-c \n");
   running=0;
}

typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {ms_init,ms_fwd,ms_turn,ms_end,ms_line,ms_turn_rad_l,ms_nop};

int main(int argc, char ** argv)
{



  int n=0,arg,time=0,opt,calibration;
  double dist=0,angle=0;
  theta = 0;

  data_log_count = 0;
  odometry_log_count = 0;
  linesensor_log_count = 0;
  calibration = 0;
  


  
//install sighandlers
   if (1){
     if (signal(SIGSEGV, segfaulthandler) == SIG_ERR) {
        perror("signal");
        exit(1);
     }
   }
   if (signal(SIGPIPE, brokenpipehandler) == SIG_ERR) {
      perror("signal");
      exit(1);
   }
   if (signal(SIGINT, ctrlchandler) == SIG_ERR) {
      perror("signal");
      exit(1);
   }
   robot_port=DEFAULT_ROBOTPORT;
    while (EOF != (opt = getopt(argc, argv, "ct:v:l:s:h:u"))) {
      switch (opt) {
        case 'c':
		calibration = 1;
   	 
    	break;
     
        case 's':
          if (optarg) {
	    int port;
	    port=  atoi(optarg);
            if (port!=0)
	       robot_port=port;
           
          } else
            exit(1);
         break;
     
        default:
         ;
       }
   }



  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",robot_port)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);

     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     //len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     len = sprintf(buf,"scanpush cmd = 'zoneobst'\n");
     laserpar_log_count = 0;
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w=0.2485;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
running=1; 
mission.state=ms_init;
mission.oldstate=-1;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
   
  for(int i = 0; i<10; i++){
  	laserpar_log[laserpar_log_count][i] = laserpar[i];
  }
  laserpar_log_count += 1;
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_odo(&odo);
  
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);

   data_logging[data_log_count][0] = mission.time;
   data_logging[data_log_count][1] = mot.motorspeed_l;
   data_logging[data_log_count][2] = mot.motorspeed_r;
	
   data_log_count += 1;
   
   if(calibration){
	//printf("%f, %f, %f \n",irsensorraw(LEFT),irsensorraw(FRONT_CENTER),irsensorraw(RIGHT));
	
	continue;
	
   }
   switch (mission.state) {
     case ms_init:
       n=4; dist=0.2;angle=90.0/180*M_PI;
       mission.state= ms_fwd;      
     break;
     case ms_nop:
	mission.state = ms_fwd;
     break;
     case ms_fwd:
	{
	
	switch (n){
		case 4:
			mot.stoppingtype = crossingline;
	       		if (fwd(dist,0.3,mission.time)){
				n -= 1;
				mission.state=ms_nop;
			}
		break;
		case 3:
			mot.stoppingtype = distance;
			if (fwd(0.2,0.1,mission.time)){
				mission.state=ms_turn;
			}
		break;
		case 5:
			
			mot.ir_min_dist = 1.2;
			angle = -90;
			mot.stoppingtype = min_ir_right;
			if (fwd(0.2,0.1,mission.time)){
				mission.state=ms_turn;
			}
		break;
		case 6:
			mot.stoppingtype = distance;
			if (fwd(dist,0.6,mission.time)){
				n -= 1;
				mission.state=ms_turn;
			}
		break;

	}
	
	
	}  
     break;
     case ms_line:
       mot.ir_max_dist = 0.1;
       mot.stoppingtype = distance_mid;
       if(follow_line(0.1,mission.time)){
		mission.state = ms_turn;
	}
       
     break;
  
     case ms_turn:
       if (turn(angle,0.3,mission.time)){
         n=n-1;
	 printf("%d",n);
         switch(n){
		case 0:
			
			mission.state = ms_fwd;
		break;
		case 1:
			n = 5;
			mission.state = ms_fwd;
			
		break;
		case 2:
			mission.state = ms_line;
		
		break;
		case 4:
			n = 6;
			mission.state = ms_fwd;
		break;
		

	}
	}
     break;    
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }  
/*  end of mission  */
 
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
    //    printf(" laser %f \n",laserpar[3]);
  time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}/* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
 
  fr = fopen("output.dat","w");
  for(int i = 0; i<data_log_count;i++){
     fprintf(fr,"%lf\t",data_logging[i][0]);
     fprintf(fr,"%lf\t",data_logging[i][1]); 
     fprintf(fr,"%lf \n",data_logging[i][2]); 
       
  }

  
  fclose(fr);
  
  fr = fopen("odo_output.dat","w");
	  for(int i = 0; i<odometry_log_count;i++){
	     fprintf(fr,"%lf\t",odometry_log[i][0]);
	     fprintf(fr,"%lf\t",odometry_log[i][1]); 
	     fprintf(fr,"%lf \n",odometry_log[i][2]); 
       
  }

  
  fclose(fr);

  fr = fopen("laserlog_output.dat","w");
	  for(int i = 0; i<laserpar_log_count;i++){
		for(int j = 0; j<10; j++){
		   fprintf(fr,"%lf\t",laserpar_log[i][j]);
	        }
                   fprintf(fr,"\n");
	     
       
           }

  
  fclose(fr);
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  p->angle = 0;
  p->position_x = 0;
  p->position_y = 2;

  
}

void update_odo(odotype *p)
{
  int delta;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  p->right_pos += delta * p->cr;
  p->right_displacement = delta * p->cr;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  p->left_pos += delta * p->cl;
  p->left_displacement = delta * p->cl;

  
  
  p->center_displacement = (p->right_displacement + p->left_displacement)/2;
  

  p->delta_angle = (p->right_displacement - p->left_displacement)/odo.w;
  
  p->angle += p->delta_angle;
  p->position_x += cos(p->angle) * p->center_displacement;
  p->position_y += sin(p->angle) * p->center_displacement;

  odometry_log[odometry_log_count][0] = p->position_x;
  odometry_log[odometry_log_count][1] = p->position_y;
  odometry_log[odometry_log_count][2] = p->angle;
  odometry_log_count += 1;
  
  
  
}


void update_motcon(motiontype *p){ 



if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
       p->startpos=(p->left_pos+p->right_pos)/2;
       p->curcmd=mot_move;
       break;
	
       case mot_line:
	p->curcmd = mot_line;
       break;

       case mot_line_l:
	p->curcmd = mot_line_l;
       break;
	
       case mot_line_r:
	p->curcmd = mot_line_r;
       break;
       
       case mot_turn:
         if (p->angle > 0) 
	    p->startpos=p->right_pos;
	 else
	    p->startpos=p->left_pos;
         p->curcmd=mot_turn;
       break;
       case mot_nop:
	break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
       p->motorspeed_l=0;
       p->motorspeed_r=0;
     break;
     case mot_move:
       
       if (should_stop()){
	  //printf("finished");
          p->finished=1;
	  p->motorspeed_l=0;
          p->motorspeed_r=0;
       }	  
       else {
	  double angular_correction = (0.1) * (theta - odo.angle);
	  double b = odo.w;
	  
	
	  p->motorspeed_l = p->speedcmd - (angular_correction * b)/2;
          p->motorspeed_r = p->speedcmd + (angular_correction * b)/2;


       }
     break;
     case mot_line:
	{
		double ls = line_center_of_mass();
		double a = ls * (0.6);
		double b = odo.w;
		//printf("%lf",ls);
		//printf("%lf \t",(a*b)/2);
		//printf("%lf \n",speed);

		p->motorspeed_l = p->speedcmd + (a*b)/2;
		p->motorspeed_r = p->speedcmd - (a*b)/2;
		if (should_stop()){
		  //printf("finished");
		  p->finished=1;
		  p->motorspeed_l=0;
		  p->motorspeed_r=0;
       		}
	}
	
     break;
     case mot_line_l:
	{
		double ls = line_center_left();
		double a = ls * (0.6);
		double b = odo.w;
		//printf("%lf",ls);
		//printf("%lf \t",(a*b)/2);
		//printf("%lf \n",speed);

		p->motorspeed_l = p->speedcmd + (a*b)/2;
		p->motorspeed_r = p->speedcmd - (a*b)/2;
	}
	
     break;
     case mot_line_r:
	{
		double ls = line_center_right();
		double a = ls * (0.6);
		double b = odo.w;
		//printf("%lf",ls);
		//printf("%lf \t",(a*b)/2);
		//printf("%lf \n",speed);

		p->motorspeed_l = p->speedcmd + (a*b)/2;
		p->motorspeed_r = p->speedcmd - (a*b)/2;
	}
	
     break;
     case mot_nop:
     break;

     case mot_turn_rad_l:
	speed = p -> speedcmd;
	double radius = p -> turning_radius;
	//Deal with checking for completion
	p->motorspeed_l = speed * radius /(radius -odo.w/2);
	p->motorspeed_r = speed * (radius - odo.w)/(radius-odo.w/2);

     
     case mot_turn:
       speed = p->speedcmd;
       //printf("Doing turn, speed: %d \n",speed);
       if (p->angle>0){
	  if (p->right_pos-p->startpos < (p->angle*p->w)/2){
	      p->motorspeed_r=speed /3 ;
	      p->motorspeed_l=-speed /3;
	  }
	  else {	     
            p->motorspeed_r=0;
            p->motorspeed_l=0;
            p->finished=1;
	  }
	}
	else {
         
	  if (p->left_pos-p->startpos < (fabs(p->angle)*p->w)/2){
	      p->motorspeed_r= -speed /3;
	      p->motorspeed_l= speed /3 ;
	  }
	  else {	     
            p->motorspeed_l=0;
	    p->motorspeed_r=0;
            p->finished=1;
	  }
	}

     break;
   }   
}   

int should_stop(){
	//printf("should stop called");
	switch(mot.stoppingtype){
		case distance:
			return ((mot.right_pos+mot.left_pos)/2 - mot.startpos > mot.dist);
		break;
		case crossingline:
			return crossing_black_line();
		break;
		case distance_mid:
			return irsensorcalibrated(FRONT_CENTER) <= mot.ir_max_dist;
		break;
		case min_ir_right:
			return irsensorcalibrated(RIGHT) >= mot.ir_min_dist;
		break;
			
	}
	return 0;
	
	
}

int follow_line(double speed,int time){
	if(time == 0){
		mot.cmd = mot_line;
		mot.speedcmd = speed;
		return 0;

	}
	
 
	
	return mot.finished;
	
}

int fwd(double dist, double speed,int time){
   if(speed - mot.speedcmd <= 0.5 / 100){
	mot.speedcmd = speed;
   }
   else{
        mot.speedcmd += 0.5 / 100;
   }

   /*
   double remaining_dist = dist - ((mot.right_pos+mot.left_pos)/2 - mot.startpos);
   double v_max = sqrt(2 * 0.5 * remaining_dist);

   if(mot.speedcmd > v_max){
      mot.speedcmd = v_max;
   }
   */

   if (time==0){ 
     mot.cmd=mot_move;
     mot.dist=dist;
     return 0;
   }
 
   else
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speedcmd=speed;
     mot.angle=angle;
     theta += angle;
     double pi = 2*acos(0.0);
     theta = fmod(theta,2*pi);
     return 0;
   }
   else
     return mot.finished;
}

int turn_radial_left(double radius, double velocity, double angle, int time){
	if (time==0){ 
		mot.cmd=mot_turn_rad_l;
		mot.speedcmd=velocity;
		mot.angle=angle;
		theta += angle;
		double pi = 2*acos(0.0);
		theta = fmod(theta,2*pi);
		return 0;
	}
	else
		return mot.finished;
}




void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}

double line_map(double raw){
	return (raw - line_black_value) / line_white_value;	
	
}

int line_position(){
	int lowest_index = 0;
	double lowest_value = 1;
	for(int i = 0; i<8; i++){
		double value = line_map(linesensor -> data[i]);
		if(value < lowest_value){
			lowest_value = value;
			lowest_index = i;
	
		}
	}
	return lowest_index;
}



double line_center_of_mass(){
	//Upper and lower parts of the formula: x_c = sum(x_i * I_i) / sum(I_i)
	double upper = 0.0;
	double lower = 0.0;
	//double floor_value = 0.7;

	for(int i = 0; i<8; i++){
		double value =  (double) linesensor -> data[i];
		value = line_map(value);

		//printf("v: %lf",value);
		value = 1 - value;
		//value = squared_dist(floor_value,value);
		
		upper += value * i;
		lower += value;
	}
	//printf("\n");
	

	return (-1*(upper/lower)) + 3.5;
}

//Like center of mass, but weights left sensors more
double line_center_left(){
	//Upper and lower parts of the formula: x_c = sum(x_i * I_i) / sum(I_i)
	double upper = 0;
	double lower = 0;
	

	//Calibrate this to match real robots meassurements
	double floor_value = 1 - 0.5;
	

	for(int i = 0; i<8; i++){
		
		double value = (line_map(linesensor -> data[i]));
	
		value = squared_dist(floor_value,value);
		
		//printf("%lf \t",value);
		//Weight function:
		double ival = (double) i;
		double weight =  (ival/8) * 0.4 + 0.5;

		//double weight = ((ival/8)*(ival/8)) + 0.05;
		
		value *= weight;
		
		//printf("s: %lf \t",value);
		//printf("v: %lf \t",value);
		upper += (value) * i;
		lower += value;

	}
	
	//printf("\n");
	//printf("\n dir: %lf \n",(-1 * upper/lower +3.5));
	//we multiply by -1, cause the sensors go from right to left
	return ((-1 * (upper/lower)) + 3.5);

}


//Like center of mass, but weights right sensors more
double line_center_right(){
	//Upper and lower parts of the formula: x_c = sum(x_i * I_i) / sum(I_i)
	double upper = 0;
	double lower = 0;
	

	//Calibrate this to match real robots meassurements
	//double floor_value = 1 - 0.5;
	

	for(int i = 0; i<8; i++){
		
		double value = (line_map(linesensor -> data[i]));
	
		//value = squared_dist(floor_value,value);
		
		//printf("%lf \t",value);
		//Weight function:
		double ival = (double) (7-i);
		double weight =  (ival/8) * 0.4 + 0.5;
		//double weight = 1 - (ival/8);

		
		
		value *= weight;
		
		printf("s: %lf \t",value);
		//printf("v: %lf \t",value);
		upper += (value) * i;
		lower += value;

	}
	
	//printf("\n");
	//printf("\n dir: %lf \n",(-1 * upper/lower +3.5));
	//we multiply by -1, cause the sensors go from right to left
	return ((-1 * (upper/lower)) + 3.5);

}


double squared_dist(double target, double value){
	double x = (target - value);
	return x*x;
}


int crossing_black_line()
  {
    double cmb = 0;
    for (int i = 0; i < 8; i++)
    {
      double value = (linesensor -> data[i]);
      cmb += value / 255;
    }
    //printf("line combine: %lf \n",cmb);
    return (cmb < 2);
  }


    
  
double irsensorraw(irposition pos){
  return getinputref("irsensor",inputtable)->data[pos];
}

double irsensorcalibrated(irposition pos){
  //Ka -> vector
  // Kb -> value
  // IRout = Ka / (IR - Kb)

  //Sim environment values:
  double KaLeft = 13.4392;
  double KaCenter = 13.4392;
  double KaRight = 13.4392;

  double KbLeft = 78.3631;
  double KbCenter = 78.3631;
  double KbRight = 78.3631;

  double Kb;
  double Ka;

  if (pos==RIGHT){
    Kb = KbRight;
    Ka = KbRight;
  } else if (pos==LEFT){
    Kb = KbLeft;
    Ka = KaLeft;
  } else if (pos==FRONT_RIGHT||pos==FRONT_LEFT||pos==FRONT_CENTER){
    Kb  = KbCenter;
    Ka = KaCenter;
  }
  //printf("doin ir");
  printf("ir %lf \n",Ka/(irsensorraw(pos)-Kb));
  return (Ka/(irsensorraw(pos)-Kb));
}

