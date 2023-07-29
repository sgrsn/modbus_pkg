#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <modbus/modbus.h>

class ModbusNode 
{
public:
  ModbusNode(int baudrate=9600, char parity='N', int data_bit=8, int stop_bit=1)
      : nh_("~")
  {
    std::string topic_name;
    nh_.param<std::string>("out_topic_name", topic_name, "/out");
    pub_ = nh_.advertise<std_msgs::Float64>(topic_name, 10);

    std::string device;
    nh_.param<std::string>("device", device, "/dev/ttyUSB0");
    ctx_ = modbus_new_rtu(device.c_str(), baudrate, parity, data_bit, stop_bit);
    if (ctx_ == NULL) {
      ROS_ERROR("Unable to create the libmodbus context");
      exit(EXIT_FAILURE);
    }

    int server_id;
    nh_.param<int>("server_id", server_id, 0);
    modbus_set_slave(ctx_, server_id);

    if (modbus_connect(ctx_) == -1) {
      ROS_ERROR("Connection failed: %s", modbus_strerror(errno));
      modbus_free(ctx_);
      exit(EXIT_FAILURE);
    }

    ROS_INFO("usb-module connect");
  }

  void spin() 
  {
    ros::Rate loop_rate(500);
    while (ros::ok()) 
    {
      uint16_t tab_reg[32];

      int rc = modbus_read_input_registers(ctx_, 0, 6, tab_reg);
      if (rc == -1) {
        ROS_ERROR("%s", modbus_strerror(errno));
        modbus_close(ctx_);
        modbus_free(ctx_);
        exit(EXIT_FAILURE);
      }

      std_msgs::Float64 msg;
      int data = (int32_t)tab_reg[4] | ((int32_t)tab_reg[5]<<16);
      msg.data = (double)data;
      pub_.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }

    modbus_close(ctx_);
    modbus_free(ctx_);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  modbus_t *ctx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "modbus_node");

  ModbusNode node(9600, 'N', 8, 1);
  node.spin();

  return 0;
}
