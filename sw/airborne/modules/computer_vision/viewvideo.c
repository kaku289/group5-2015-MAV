/*
 * Copyright (C) 2012-2014 The Paparazzi Community
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/computer_vision/viewvideo.c
 *
 * Get live images from a RTP/UDP stream
 * and save pictures on internal memory
 *
 * Works on Linux platforms
 */

// Own header
#include "modules/computer_vision/viewvideo.h"

#include "firmwares/rotorcraft/navigation.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>


// UDP RTP Images
#include "modules/computer_vision/lib/udp/socket.h"
// Video
#include "modules/computer_vision/lib/v4l/video.h"
#include "modules/computer_vision/cv/resize.h"
#include "modules/computer_vision/cv/encoding/jpeg.h"
#include "modules/computer_vision/cv/encoding/rtp.h"

// Threaded computer vision
#include <pthread.h>

// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// Output socket can be defined from an offset
#ifdef VIDEO_SOCK_OUT_OFFSET
#define VIDEO_SOCK_OUT (5000+VIDEO_SOCK_OUT_OFFSET)
#endif

#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif

#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 4999
#endif

// Downsize factor for video stream
#ifndef VIDEO_DOWNSIZE_FACTOR
#define VIDEO_DOWNSIZE_FACTOR 4
#endif

// From 0 to 99 (99=high)
#ifndef VIDEO_QUALITY_FACTOR
#define VIDEO_QUALITY_FACTOR 50
#endif

// Frame Per Seconds
#ifndef VIDEO_FPS
#define VIDEO_FPS 4.
#endif

void viewvideo_run(void) {}

// take shot flag
int viewvideo_shot = 0;

/////////////////////////////////////////////////////////////////////////
// COMPUTER VISION THREAD

pthread_t computervision_thread;
volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void *data);
void *computervision_thread_main(void *data)
{
  // Video Input
  struct vid_struct vid;
  vid.device = (char *)"/dev/video1";
  vid.w = 1280;
  vid.h = 720;
  vid.n_buffers = 4;
  if (video_init(&vid) < 0) {
    printf("Error initialising video\n");
    computervision_thread_status = -1;
    return 0;
  }

  // Video Grabbing
  struct img_struct *img_new = video_create_image(&vid);

  // Video Resizing
  uint8_t quality_factor = VIDEO_QUALITY_FACTOR;
  uint8_t dri_jpeg_header = 0;
  int microsleep = (int)(1000000. / VIDEO_FPS);

  struct img_struct small;
  small.w = vid.w / VIDEO_DOWNSIZE_FACTOR;
  small.h = vid.h / VIDEO_DOWNSIZE_FACTOR;
  small.buf = (uint8_t *)malloc(small.w * small.h * 2);

  // Video Compression
  //uint8_t *jpegbuf = (uint8_t *)malloc(vid.h * vid.w * 2);

  // Network Transmit
  //struct UdpSocket *vsock;
  //vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);

  // Create SPD file and make folder if necessary
  //FILE *sdp;
  //if (system("mkdir -p /data/video/sdp") == 0) {
  //  sdp = fopen("/data/video/sdp/x86_config-mjpeg.sdp", "w");
  //  if (sdp != NULL) {
  //    fprintf(sdp, "v=0\n");
  //    fprintf(sdp, "m=video %d RTP/AVP 26\n", (int)(VIDEO_SOCK_OUT));
  //    fprintf(sdp, "c=IN IP4 0.0.0.0");
  //    fclose(sdp);
  //  }
 // }

  // file index (search from 0)
  int file_index = 0;

  // time
  struct timeval last_time;
  gettimeofday(&last_time, NULL);

  while (computer_vision_thread_command > 0) {
    // compute usleep to have a more stable frame rate
    struct timeval time;
    gettimeofday(&time, NULL);
    int dt = (int)(time.tv_sec - last_time.tv_sec) * 1000000 + (int)(time.tv_usec - last_time.tv_usec);
    if (dt < microsleep) { usleep(microsleep - dt); }
    last_time = time;

    // Grab new frame
    video_grab_image(&vid, img_new);

//******************************************************************************************
//******************************************************************************************
//*******************************MY CODE****************************************************
//**** Detection Strategy*******
printf("our own code ...\n");puts("**********************************************");puts("");

//int HEIGHT=img_new->h, WIDTH=img_new->w;
int HEIGHT=720, WIDTH=1280;
int N=HEIGHT*WIDTH, counter, i, j;
uint8_t *source = img_new->buf;
int y[921600]={0};
int im[720][1280]={0}, Obstacle[1280]={0};
int c1=0,c2;
float avg[1280]={0.0}, variance[1280]={0.0};
int s[1280]={0};
float s1, threshold=0.006;
int cuttop=100, cutbottom=100;


for ( counter=0; counter < N; counter++)
  {
    //im[counter/720][counter%720]=(int)source[2*counter+1];
    im[counter/1280][counter%1280]=(int)source[2*counter+1];

  }


for (i=0; i<WIDTH; i++)
  {
	for (j=cuttop; j<HEIGHT-cutbottom; j++)
	{
		s[i] = s[i] + im[j][i];
	}
	avg[i] = (float)s[i]/(720-cuttop-cutbottom);
	//printf("%d\t%f\n", s[i], avg[i]);
  }

for (i = 0; i<WIDTH; i++)
{
	s1=0.0;	
	for (j=cuttop; j<HEIGHT-cutbottom; j++)
	{
		s1 = s1 + (im[j][i] - avg[i])*(im[j][i] - avg[i]);
	}
	variance[i] = s1/(720-cuttop-cutbottom)/255.0/255.0;
	if (variance[i]<=0.006)
	{
		Obstacle[c1]=i;	
		c1++;
		//printf("%d\t", i);
	}
	//printf("%f\t%f\t%f\n", s1, avg[i], variance[i]);
}
//printf("%d\t%d\t%d\t%d\t%d\t",Obstacle[0],Obstacle[319],Obstacle[639],Obstacle[959],Obstacle[1279] );
int poleStarting, minWidth=5, margin=150;
float margin2=0.25, mypsi;
struct EnuCoor_i mypos;

bool_t dum;
mypsi = stateGetNedToBodyEulers_f()->psi;
mypsi = mypsi*180.0/3.14;

printf("heading angle : %f\n",mypsi);
if (c1==0)
{ 
	dum=nav_set_heading_deg(mypsi+90.0);
	puts("No Pole found");
}
c2=0;
poleStarting=Obstacle[0];
for (i=1; i<WIDTH; i++)
{
	if (abs(Obstacle[i]-Obstacle[i-1])<minWidth)
	{
		c2+=abs(Obstacle[i]-Obstacle[i-1]);
		//printf("%d\t",Obstacle[i]);
	}
	else if (c2 < minWidth)
	{
		c2=0;
		poleStarting=Obstacle[i];
		
	}
	else {break;}
}
printf("pole width: %d\n",c2);

if (poleStarting+c2/2 > WIDTH/2-margin && poleStarting+c2/2 < WIDTH/2+margin)
{
	if ((float)c2/(float)WIDTH>margin2)
	{
		dum=nav_set_heading_deg(mypsi+180.0);
		puts("Too close to the obstacle");	
		//return 2; // To stop moving towards the obstacle
	}
	else
	{
	printf("%s\n","I found obstacle in center and ready to move towards it");
	
	waypoints[5].x = stateGetPositionEnu_i()->x + (int)(100*sin(mypsi*3.14/180.0));
	waypoints[5].y = stateGetPositionEnu_i()->y + (int)(100*cos(mypsi*3.14/180.0));

	//printf("%f\t%f\n",stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y);
	}
	//return 0; // To move towards the obstacle
}
//>> INT32_POS_FRAC
else if (poleStarting+c2/2 > WIDTH/2-margin) 
{
dum=nav_set_heading_deg(mypsi+15.0);
puts("I have to turn right");

//return -1;
} // To yaw left or right
else if (poleStarting+c2/2 < WIDTH/2+margin)
{
dum=nav_set_heading_deg(mypsi-15.0);
puts("I have to turn left");
//return 1;
}  // To yaw left or right
else
{
puts("Don't know what to do");
dum=nav_set_heading_deg(mypsi-180);
}
//printf("%f\t%f\n",stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y);
//waypoints[5].x = waypoints[5].x + (int)(100*sin(3.14/2));//(int)(100*sin(mypsi*3.14/180.0));
//waypoints[5].y = waypoints[5].y + (int)(100*cos(3.14/2));//(int)(100*cos(mypsi*3.14/180.0));

//waypoints[5].x = waypoints[5].x + 40;
//waypoints[5].x = mypos.x + (int)100*sin(1.7);

//uint8_t wp_id="WP_GOAL";
//waypoints[wp_id].x = waypoints[wp_id].x + 130;
//     waypoints[wp_id].y = waypoints[wp_id].y;



//stateGetNedToBodEulers_f()->psi
//firmware/rotorcrat/stablitzation/stablization_attitude_rc_setpoint.c
//navdata.h 

//***************************************
//waypoints[wp_id].x = waypoints[wp_id].x + 256;
//        waypoints[wp_id].y = waypoints[wp_id].y;
/*float heading0=0

//dheading defined in previous code
	
heading = heading+dheading
dummy=nav_set_heading_deg(heading);*/
/*
while (finsearch==0) 
	{
	
	if ((Obstacle[ccc]-Obstacle[ccc-1])<=(maxstep) && (Obstacle[ccc]-Obstacle[ccc-1])>0)
		{
		//puts("a");
		foundpole = 1;
		}
	else if ( foundpole==0 )
		{            
		//puts("b");
		startpole = ccc;
		foundpole=0;
		}
	else if ( foundpole && (Obstacle[ccc]-Obstacle[startpole])>=(minwidth))
		{            
		endpole = (ccc-1);
		finsearch = 1;
		found=1;
		//puts("c");
		}
	else if ( foundpole && (Obstacle[ccc]-Obstacle[startpole])<(minwidth))
		{            
		startpole=ccc;
		foundpole = 0;
		//puts("d");
		}
	
	ccc=ccc+1;

	if (ccc>(c1-1)-10 && foundpole && finsearch==0 && (ccc-startpole)>=minwidth)
		{
		finsearch = 1;
		endpole = ccc-1;
		found =1;
		//puts("e");
		}

	else if ( ccc>(c1-1)-10 && finsearch==0)
		{
		finsearch = 1;
		found = 0;
		//puts("f");
		}


		
	
	}

*/
/*
puts("startpole,endpole");
printf("%d\t%d\t",Obstacle[startpole],Obstacle[endpole] );
////**************Start Output***********
float pospole,pospolei;
int wpole;
if (found =1)
	{			
	wpole = Obstacle[endpole]-Obstacle[startpole];
	//wpole = Obstacle[1001]-Obstacle[1];
	pospole=1280.0/2.0-((Obstacle[endpole]+Obstacle[startpole])/2.0);	//positive to the left of the center
	//round pospole to nearest integer			
	pospolei =(pospole >= 0 ? (long)(pospole+0.5) : (long)(pospole-0.5));
	//
	printf("width of pole: %d\n",wpole );
	//puts("center of pole(zero at center, positive to left):");
	//printf("%f",pospolei);
	}
else if (found=0)
	{puts("No pole found");}

int cmargin =100;
int yawc;	//yawcommand: +1 for yaw movement to the right, -1 to the left, and 0 for no yawing
if (pospolei>cmargin)
	{yawc= -1;}
else if (pospolei<cmargin)
	{yawc= 1;}
else 
	{yawc=0;}
puts("yawcommand(+1 right, -1 left):");
printf("%d\n",yawc);
*/
//end of code for pole detection
//**************************End of output****************************


  //printf("%d\t%d\n",y[1], y[3]);
   /*for (int y = 0; y < img_new->h; y++) 
  {
     
  }*/

  /*for (int y = 0; y < img_new->h; y++) 
  {
    for (int x = 0; x < img_new->w; x += 2) 
    {
	printf(" image x %d y %d\n", x,y);
        printf(" pixel value uint8_t %d\n", *source[x]);
      // Color Check:
    }
  }*/

// ****** MOving Strategy *******
//waypoints[wp_id].x = waypoints[wp_id].x + 256;
//        waypoints[wp_id].y = waypoints[wp_id].y;
//	dummy=nav_set_heading_deg(90.0);







//********************************END MY CODE***********************************************
//***************************/****************************************************************
//******************************************************************************************

    // Save picture on disk
    /*if (computer_vision_thread_command == 2) {
      uint8_t *end = encode_image(img_new->buf, jpegbuf, 99, FOUR_TWO_TWO, vid.w, vid.h, 1);
      uint32_t size = end - (jpegbuf);
      FILE *save;
      char save_name[128];
      if (system("mkdir -p /data/video/images") == 0) {
        // search available index (max is 99)
        for (; file_index < 99; file_index++) {
          printf("search %d\n", file_index);
          sprintf(save_name, "/data/video/images/img_%02d.jpg", file_index);
          // test if file exists or not
          if (access(save_name, F_OK) == -1) {
            printf("access\n");
            save = fopen(save_name, "w");
            if (save != NULL) {
              fwrite(jpegbuf, sizeof(uint8_t), size, save);
              fclose(save);
            } else {
              printf("Error when opening file %s\n", save_name);
            }
            // leave for loop
            break;
          } else {
            printf("file exists\n");
          }
        }
      }
      computer_vision_thread_command = 1;
      viewvideo_shot = 0;
    }
    */
    // Resize
    //resize_uyuv(img_new, &small, VIDEO_DOWNSIZE_FACTOR);

    // JPEG encode the image:
    //uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    //uint8_t *end = encode_image(small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
    //uint32_t size = end - (jpegbuf);

    // Send image with RTP
    //printf("Sending an image ...%u\n", size);
    //send_rtp_frame(
    //  vsock,            // UDP
    //  jpegbuf, size,    // JPEG
    //  small.w, small.h, // Img Size
    //  0,                // Format 422
    //  quality_factor,   // Jpeg-Quality
    //  dri_jpeg_header,  // DRI Header
    //  1                 // 90kHz time increment
    //);
    // Extra note: when the time increment is set to 0,
    // it is automaticaly calculated by the send_rtp_frame function
    // based on gettimeofday value. This seems to introduce some lag or jitter.
    // An other way is to compute the time increment and set the correct value.
    // It seems that a lower value is also working (when the frame is received
    // the timestamp is always "late" so the frame is displayed immediately).
    // Here, we set the time increment to the lowest possible value
    // (1 = 1/90000 s) which is probably stupid but is actually working.
  }
  printf("Thread Closed\n");
  video_close(&vid);
  computervision_thread_status = -100;
  return 0;
}

void viewvideo_start(void)
{
  computer_vision_thread_command = 1;
  int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
  if (rc) {
    printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
  }
}

void viewvideo_stop(void)
{
  computer_vision_thread_command = 0;
}

int viewvideo_save_shot(void)
{
  if (computer_vision_thread_command > 0) {
    computer_vision_thread_command = 2;
  }
  return 0;
}

