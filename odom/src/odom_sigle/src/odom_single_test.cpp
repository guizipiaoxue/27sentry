// 单雷达 DLIO 测试输入节点：将 192.168.1.3 的 Livox CustomMsg 转为 PointCloud2。
// 默认输入 /livox/lidar_192_168_1_3，默认输出 /odom_single/points_raw。
#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
class OdomSingleInput final : public rclcpp::Node {
 public:
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  OdomSingleInput() : Node("odom_single_test") {
    const auto input=declare_parameter<std::string>("input_topic", "/livox/lidar_192_168_1_3");
    const auto output=declare_parameter<std::string>("output_topic", "/odom_single/points_raw");
    const auto frame=declare_parameter<std::string>("frame_id", "lidar");
    publisher_=create_publisher<sensor_msgs::msg::PointCloud2>(output,rclcpp::SensorDataQoS());
    subscription_=create_subscription<CustomMsg>(input,rclcpp::SensorDataQoS(),[this,frame](CustomMsg::ConstSharedPtr m){publish(*m,frame);});
    RCLCPP_INFO(get_logger(),"Livox input: %s -> PointCloud2 output: %s",input.c_str(),output.c_str());
  }
 private:
  void publish(const CustomMsg &msg,const std::string &frame){
    const std::size_t n=std::min<std::size_t>(msg.points.size(),msg.point_num);
    sensor_msgs::msg::PointCloud2 c; c.header=msg.header; if(!frame.empty()) c.header.frame_id=frame; c.height=1; c.width=static_cast<std::uint32_t>(n); c.is_bigendian=false; c.is_dense=true;
    sensor_msgs::PointCloud2Modifier mod(c); mod.setPointCloud2Fields(5,"x",1,sensor_msgs::msg::PointField::FLOAT32,"y",1,sensor_msgs::msg::PointField::FLOAT32,"z",1,sensor_msgs::msg::PointField::FLOAT32,"intensity",1,sensor_msgs::msg::PointField::FLOAT32,"timestamp",1,sensor_msgs::msg::PointField::FLOAT64); mod.resize(n);
    sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z"),i(c,"intensity"); sensor_msgs::PointCloud2Iterator<double> t(c,"timestamp");
    for(std::size_t k=0;k<n;++k,++x,++y,++z,++i,++t){const auto&p=msg.points[k]; *x=p.x;*y=p.y;*z=p.z;*i=static_cast<float>(p.reflectivity);*t=static_cast<double>(msg.timebase)+static_cast<double>(p.offset_time);}
    publisher_->publish(std::move(c));
  }
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_; rclcpp::Subscription<CustomMsg>::SharedPtr subscription_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<OdomSingleInput>());rclcpp::shutdown();return 0;}
