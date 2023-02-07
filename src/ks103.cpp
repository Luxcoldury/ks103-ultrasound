#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <thread>
#include <mutex>

#define SENSOR_NUM_MAX 20

int noise_filtering;
std::mutex i2c_mtx;

bool set_i2c_register(int fd, unsigned char addr, unsigned char reg, unsigned char value) {
    i2c_mtx.lock();
    unsigned char outbuf[2];
    struct i2c_rdwr_ioctl_data  packets;
    struct i2c_msg  messages[1];

    messages[0].addr    = addr;
    messages[0].flags   = 0;
    messages[0].len     = sizeof(outbuf);
    messages[0].buf     = outbuf;

    // The first byte indicates which register we'll write
    outbuf[0] = reg;

    // The second byte indicates the value to write
    outbuf[1] = value;

    // Transfer the i2c packets to the kernel and verify it worked
    packets.msgs = messages;
    packets.nmsgs = 1;
    if ( ioctl(fd, I2C_RDWR, &packets) < 0 ) {
        perror("Unable to send data");
        i2c_mtx.unlock();
        return false;
    }

    i2c_mtx.unlock();
    return true;
}

bool get_i2c_register(int fd, unsigned char addr, unsigned char reg, unsigned char *val) {
    i2c_mtx.lock();
    unsigned char inbuf, outbuf;
    struct i2c_rdwr_ioctl_data  packets;
    struct i2c_msg  messages[2];

    // In order to read a register, we first do a "dummy write" by writing
    // 0 bytes to the register we want to read from.
    outbuf = reg;
    messages[0].addr    = addr;
    messages[0].flags   = 0;
    messages[0].len     = sizeof(outbuf);
    messages[0].buf     = &outbuf;

    messages[1].addr    = addr;
    messages[1].flags   = I2C_M_RD;     // | I2C_M_NOSTART
    messages[1].len     = sizeof(inbuf);
    messages[1].buf     = &inbuf;

    // Send the request to the kernel and get the result back
    packets.msgs    = messages;
    packets.nmsgs   = 2;
    if (ioctl(fd, I2C_RDWR, &packets) < 0 ) {
        perror("Unable to send data");
        i2c_mtx.unlock();
        return false;
    }
    i2c_mtx.unlock();
    *val = inbuf;
    return true;
}

bool get_distance(int fd, unsigned char addr, int *distance) {
    unsigned char distance_high;
    unsigned char distance_low;

    set_i2c_register(fd, addr, 2, 0xbc);
    usleep(100000);     // 100ms
    if (!get_i2c_register(fd, addr, 2, &distance_high))
        return false;
    *distance = distance_high << 8;
    if (!get_i2c_register(fd, addr, 3, &distance_low))
        return false;
    *distance |= distance_low;

    return true;
}

bool change_address(int fd, unsigned char addr, unsigned char new_addr) {
    if (!set_i2c_register(fd, addr, 2, 0x9a)) {
        return false;
    }
    usleep(2000);
    if (!set_i2c_register(fd, addr, 2, 0x92)) {
        return false;
    }
    usleep(2000);
    if (!set_i2c_register(fd, addr, 2, 0x9e)) {
        return false;
    }
    usleep(2000);
    if (!set_i2c_register(fd, addr, 2, new_addr<<1)) {
        return false;
    }
    usleep(200000);

    return true;

}

void check_and_publish(int i2c_handle, int address, ros::Publisher pub){
  
  set_i2c_register(i2c_handle, address, 2, 0x69+noise_filtering); // Set noise filtering to level (0~6)
  usleep(100000); // 100ms

  ros::Rate loop_rate(1);

  sensor_msgs::Range msg;
  msg.radiation_type= sensor_msgs::Range::ULTRASOUND;
  msg.field_of_view=0;
  msg.min_range=0.020;
  msg.max_range=11.280;

  while(ros::ok()){
    int distance;
    if(get_distance(i2c_handle,address,&distance)){
      // printf("dis:%d",distance);
      msg.range=distance/1000.0f;
      pub.publish(msg);
    }
  
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
	int i2c_handle;

  if ((i2c_handle = open("/dev/i2c-1", O_RDWR)) < 0) 
  {
    ROS_ERROR("Failed to open I2C bus of Ultrasound");
    exit(1);
  }

  ros::init(argc, argv, "ultrasound");
  ros::NodeHandle n("~");

  // ros::Publisher pub = n.advertise<sensor_msgs::Range>("range", 1000);

  // ros::Rate loop_rate(1);
  // int distance;
  // sensor_msgs::Range msg;
  // msg.radiation_type= sensor_msgs::Range::ULTRASOUND;
  // msg.field_of_view=0;
  // msg.min_range=0.020;
  // msg.max_range=11.280;
      
  // while(ros::ok()){
  //   if(get_distance(i2c_handle,0x70,&distance)){
  //     ROS_INFO("dis:%d",distance);
  //     msg.range=distance/1000.0f;
  //     pub.publish(msg);
  //   }
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }



  std::vector<int> sensor_address_vec;
  std::vector<ros::Publisher> pub_vec;
  std::vector<std::thread> threads;

  if (ros::ok()){
    n.param("noise_filtering",noise_filtering,1);
    // printf("nf:%d",noise_filtering);

    for(int i=0;i<SENSOR_NUM_MAX;i++){    
      int sensor_address;
      if(n.getParam("address/"+std::to_string(i+1),sensor_address)){
        sensor_address_vec.push_back(sensor_address);
        // printf("add:%x",sensor_address);
        ros::Publisher pub_tmp = n.advertise<sensor_msgs::Range>("range/"+std::to_string(i+1), 1000);
        pub_vec.push_back(pub_tmp);
      }else{
        break;
      }
    }
  }

  int sensor_count=sensor_address_vec.size();

  if (ros::ok())
  {
    for(int i=0;i<sensor_count;i++){
      // check_and_publish(i2c_handle, sensor_address_vec[i], pub_vec[i]);
      std::thread thread(check_and_publish, i2c_handle, sensor_address_vec[i], pub_vec[i]);
      threads.push_back(std::move(thread));
    }

    for (std::thread &th: threads){
        th.join();
    }
    // printf("before c&p");
    // check_and_publish(i2c_handle, sensor_address_vec[0], pub_vec[0]);

  }

  return 0;
}
