#ifndef NODE1_H_
#define NODE1_H_

#include "ros/ros.h"
#include <iostream>
#include <bits/stdc++.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "iris/BSRX.h"
#include "iris/BSTX.h"
#include "iris/data.h"
#include <termios.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define RAD2DEG 57.295780
#define DEG2RAD 0.017452925

using namespace std;
using namespace cv;

Mat img;
Mat foto_resize, foto_color, tresh, foto_flip, foto_edit;

ros::Publisher pub_msg;

int stat;
int prev_stat;
int x_tujuan;
int y_tujuan;
int linear_vel = 5;
int angular_vel = 8;
int x_ball_pos;
int y_ball_pos;
int radius = 100;
bool check = false;
int countRotate = 0;
int buffer_theta;
int buffer_x;
int buffer_y;
bool getToBall = false;
iris::BSTX msg;

int getch();
void reset();
void status1();
void status2();
void status3();
void status4();
void cllbackPc2Bs(const iris::BSRXConstPtr &msg);
int robotAngleToPoint();
int getVelx();
int getVely();
int getVelXRotation();
int getVelYRotation();
void cllbckTimer10hz(const ros::TimerEvent &e);
void ball_threshold(Mat foto);


#endif