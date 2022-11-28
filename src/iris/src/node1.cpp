#include "iris/node1.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node1");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spn;

    Mat img = imread("/home/dimasf/Desktop/modul_ros_programming/fp_22/src/iris/resources/gam3.jpg");
    ball_threshold(img);

    ros::Subscriber sub1 = nh.subscribe("/bs2pc_telemetry", 1000, cllbackPc2Bs);
    pub_msg = nh.advertise<iris::BSTX>("/pc2bs_telemetry", 10);

    ros::Timer timer1 = nh.createTimer(ros::Duration(1), cllbckTimer10hz);

    spn.spin();

    return 0;
}

void cllbckTimer10hz(const ros::TimerEvent &e)
{

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (prev_stat != stat)
        {
            prev_stat = stat;
            reset();
        }
        if (stat == 1)
        {
            status1();
        }
        if (stat == 2)
        {
            status2();
        }
        if (stat == 3)
        {
            status3();
        }
        if (stat == 4)
        {
            status4();
        }

        pub_msg.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int robotAngleToPoint()
{
    return int(atan2(msg.bola_y - msg.pos_y, msg.bola_x - msg.pos_x) * RAD2DEG);
}
int getVelx()
{
    return linear_vel * (cos(atan2(msg.bola_y - msg.pos_y, msg.bola_x - msg.pos_x)));
}
int getVely()
{
    return linear_vel * (sin(atan2(msg.bola_y - msg.pos_y, msg.bola_x - msg.pos_x)));
}

int getVelXRotation()
{
    return linear_vel * cos(atan2(msg.bola_y - msg.pos_y, msg.bola_x - msg.pos_x) - M_PI_2);
}

int getVelYRotation()
{
    return linear_vel * sin(atan2(msg.bola_y - msg.pos_y, msg.bola_x - msg.pos_x) - M_PI_2);
}

void getTheta()
{
    while (msg.pos_theta > 180)
    {
        msg.pos_theta -= 360;
    }
    while (msg.pos_theta < -180)
    {
        msg.pos_theta += 360;
    }
}

void status1()
{
    ROS_INFO("=======CASE 1======");
    getToBall = false;

    int c = getch();
    ROS_INFO("key: %d", c);

    if (c == 119)
    {
        msg.pos_x += linear_vel;
        msg.pos_y += 0;
        pub_msg.publish(msg);
    }
    if (c == 115)
    {
        msg.pos_x -= linear_vel;
        msg.pos_y += 0;
        pub_msg.publish(msg);
    }
    if (c == 97)
    {
        msg.pos_y -= linear_vel;
        msg.pos_x += 0;
        pub_msg.publish(msg);
    }
    if (c == 100)
    {
        msg.pos_y += linear_vel;
        msg.pos_x += 0;
        pub_msg.publish(msg);
    }
    if (c == 118)
    {
        msg.pos_y += 0;
        msg.pos_x += 0;
        pub_msg.publish(msg);
    }
    if (c == 122)
    {
        msg.pos_theta += angular_vel;
        msg.pos_x += 0;
        msg.pos_y += 0;
        getTheta();
        pub_msg.publish(msg);
    }
    if (c == 120)
    {
        msg.pos_theta -= angular_vel;
        msg.pos_x += 0;
        msg.pos_y += 0;
        getTheta();
        pub_msg.publish(msg);
    }
}

void status2()
{
    ROS_INFO("=======CASE 2======");
    getToBall = false;

    int theta;
    int vel_x;
    int vel_y;

    msg.bola_x = x_ball_pos;
    msg.bola_y = y_ball_pos;

    if (abs(msg.bola_x - msg.pos_x) > 10 && abs(msg.bola_y - msg.pos_y) > 10)
    {
        theta = robotAngleToPoint();
        vel_x = getVelx();
        vel_y = getVely();
        msg.pos_theta = theta;
        msg.v_x = vel_x;
        msg.v_y = vel_y;
        msg.pos_x += vel_x;
        msg.pos_y += vel_y;
        getTheta();
    }
    else
    {
        ROS_INFO("masuk sini");
        msg.pos_x += 0;
        msg.pos_y += 0;
        pub_msg.publish(msg);
    }
}

void status3()
{
    ROS_INFO("=======CASE 3======");
    getToBall = false;

    msg.bola_x = x_tujuan;
    msg.bola_y = y_tujuan;

    int theta;
    int vel_x;
    int vel_y;

    if (abs(msg.bola_x - msg.pos_x) > 10 && abs(msg.bola_y - msg.pos_y) > 10)
    {
        theta = robotAngleToPoint();
        vel_x = getVelx();
        vel_y = getVely();
        msg.pos_theta = theta;
        msg.v_x = vel_x;
        msg.v_y = vel_y;
        msg.pos_x += vel_x;
        msg.pos_y += vel_y;
        getTheta();
    }
    else
    {
        msg.pos_x += 0;
        msg.pos_y += 0;
        pub_msg.publish(msg);
    }
}

void status4()
{
    ROS_INFO("=======CASE 4======");
    msg.bola_x = x_ball_pos;
    msg.bola_y = y_ball_pos;

    int theta;
    int vel_x;
    int vel_y;

    if (getToBall)
    {

        if (abs(msg.bola_x - msg.pos_x) > 10 && abs(msg.bola_y - msg.pos_y) > 10)
        {
            theta = robotAngleToPoint();
            vel_x = getVelx();
            vel_y = getVely();
            msg.pos_theta = theta;
            msg.v_x = vel_x;
            msg.v_y = vel_y;
            msg.pos_x += vel_x;
            msg.pos_y += vel_y;
            getTheta();
        }
        else
        {
            msg.pos_x += 0;
            msg.pos_y += 0;
            pub_msg.publish(msg);
        }
    }

    if (sqrt((msg.bola_x - msg.pos_x) * (msg.bola_x - msg.pos_x) + (msg.bola_y - msg.pos_y) * (msg.bola_y - msg.pos_y)) > 100 && !check && !getToBall)
    {
        theta = robotAngleToPoint();
        vel_x = getVelx();
        vel_y = getVely();
        msg.pos_theta = theta;
        msg.v_x = vel_x;
        msg.v_y = vel_y;
        msg.pos_x += vel_x;
        msg.pos_y += vel_y;
        buffer_theta = theta;
        buffer_x = msg.pos_x;
        buffer_y = msg.pos_y;
        getTheta();
    }
    else if (!getToBall)
    {
        check = true;
        theta = robotAngleToPoint();
        vel_x = getVelXRotation();
        vel_y = getVelYRotation();

        msg.pos_theta = theta;
        msg.v_x = vel_x;
        msg.v_y = vel_y;
        msg.pos_x += vel_x;
        msg.pos_y += vel_y;

        if (sqrt((buffer_x - msg.pos_x) * (buffer_x - msg.pos_x) + (buffer_y - msg.pos_y) * (buffer_y - msg.pos_y)) < 10 || abs(buffer_theta - theta) < 3)
        {
            countRotate++;
        }
        getTheta();
        if (countRotate == 6)
        {
            check = false;
            countRotate = 0;
            getToBall = true;
        }
    }
}

void reset()
{
    msg.pos_x = 10;
    msg.pos_y = 10;
    msg.pos_theta = 0;
    msg.bola_x = 0;
    msg.bola_y = 0;
}

void cllbackPc2Bs(const iris::BSRXConstPtr &msg)
{
    stat = msg->status;
    x_tujuan = msg->x_tujuan;
    y_tujuan = msg->y_tujuan;
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int c = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return c;
}

void ball_threshold(Mat foto)
{
    Point2f center;
    float radius;
    resize(foto, foto_resize, Size(900, 600));
    cvtColor(foto_resize, foto_color, COLOR_BGR2HSV);
    int L_H = 0;
    int U_H = 85;
    int L_S = 0;
    int U_S = 255;
    int L_V = 0;
    int U_V = 255;
    inRange(foto_color, Scalar(L_H, L_S, L_V), Scalar(U_H, U_S, U_V), tresh);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(tresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
        vector<Point> contours_lagi;
        contours_lagi = contours[i];
        minEnclosingCircle(contours_lagi, center, radius);
        if (radius >= 30 && radius <= 110)
        {
            circle(foto_resize, center, radius, Scalar(255, 0, 0), 3);
            x_ball_pos = center.y;
            y_ball_pos = center.x;
        }
    }
}
