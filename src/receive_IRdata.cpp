#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <string>

serial::Serial IR_module_sp;
ros::Publisher pub_serial_IR;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "IRdata_publisher");
    ros::NodeHandle n;
    ros::Publisher ros_serial_IRdata_pub = n.advertise<std_msgs::Int16>("serial_IRdata_msg", 100);
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //choose the name of serial to open
    IR_module_sp.setPort("/dev/ttyUSB0");
    //set the baudrate = 115200
    IR_module_sp.setBaudrate(115200);
    //set timeout
    IR_module_sp.setTimeout(to);
    try
    {
        //open the IR module serial
        IR_module_sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open serial port .");
        return -1;
    }
    if(IR_module_sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100);

    while( ros::ok() )
    {
        std_msgs::Int16 IR_msg;
        //get byte num of buff
        size_t byte_num = IR_module_sp.available();
        uint8_t rec_buffer[byte_num];
        if(byte_num != 0)
        {
           // printf("byte_num = %ld\n", byte_num);
            IR_module_sp.read(rec_buffer, byte_num);
            IR_msg.data = atoi((const char*)rec_buffer);
            ros_serial_IRdata_pub.publish(IR_msg);
            //ROS_INFO_STREAM("Recvied IR data = ." << IR_msg);
            //ROS_INFO_STREAM("Recvied IR data[] = ." << IR_msg.data);
            //printf("Recvied IR data = %ld\r\n", IR_msg.data);
        }
        else
        {
            //ROS_INFO_STREAM("Recvied none of serial IR data.");
        }
        
        loop_rate.sleep();
    }
    return 0;
}