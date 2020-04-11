本人小白，项目中使用cartographer进行机器人的定位。cartographer_ros中没有发布机器人在地图坐标系中的位姿topic，只发布各frame间的tf变换。若通过tf变换获取位姿坐标，测试发现不管激光频率多高（20~100hz），获取的位姿频率只有5h左右。因此对源码进行简单修改，获取高频率的位姿topic并在rviz中根据机器人当前位置进行重定位。

 ## 准备工作
 
 ### 源码安装
 首先对cartographer进行源码安装，相关链接如下：

- cartographer项目源码[cartographer](https://github.com/googlecartographer/cartographer)

- cartographer_ros项目源码[cartographer_ros](https://github.com/googlecartographer/cartographer_ros)

- 源码安装教程[源码安装](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html)
### demo运行
cartographer_rosd的demo运行[demo](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)，主要参考.launch运行文件和.lua配置文件。需要注意的是需要开启pure localization功能。
## 位姿获取
### tf坐标变换-不修改源码
通过tf变换获取位姿坐标，获取的位姿频率只有5hz左右，但不用修改源码。

建立tf变换节点，发布2D位姿topic。
```c++
ros::NodeHandle nh;
ros::Publisher _pose_pub;
_pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose_nav", 10);

tf::StampedTransform transform;
    tf::TransformListener listener;
    try
    {
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    //输出位置信息
    pos_now.x = static_cast<double>(transform.getOrigin().x());
    pos_now.y = static_cast<double>(transform.getOrigin().y());
    pos_now.theta = tf::getYaw(transform.getRotation());
    _pose_pub.publish(pos_now);
```
### 修改源码
在cartographer_ros/cartographer_ros/cartographer_ros/node.h添加::ros::Publisher _pose_pub。对node.cc进行修改：
```cpp
_pose_pub = node_handle_.advertise<geometry_msgs::Pose2D>("pose_nav", 10);//100hz  
geometry_msgs::Pose2D pos_now;

if (trajectory_data.published_to_tracking != nullptr) {
      if (trajectory_data.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_data.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_data.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_data.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_data.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        tf_broadcaster_.sendTransform(stamped_transforms);

        geometry_msgs::Transform transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_data.published_to_tracking));
        //********修改部分：添加位姿发布**********/
        pos_now.x = static_cast<double>(transform.translation.x);
        pos_now.y = static_cast<double>(transform.translation.y);
        pos_now.theta = tf::getYaw(transform.rotation);
        _pose_pub.publish(pos_now);
      } 
```
## rviz重定位设置
### 订阅/initialpose话题
rviz中的“2D Pose Estimate”可发布/initialpose话题，通过点击地图位置可以发布相应位置的topic，包括想x,y和theta。
```cpp
_pose_init_sub = nh.subscribe("/initialpose", 1000, &NavNode::init_pose_callback, this);
```

### 重定位设置
重定位功能通过调用API设置，参考[API](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html)。
```cpp
void NavNode::init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = tf2::getYaw(msg->pose.pose.orientation);
    ros::NodeHandle nh;

    ros::ServiceClient client_traj_finish = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    cartographer_ros_msgs::FinishTrajectory srv_traj_finish;
    srv_traj_finish.request.trajectory_id = traj_id;
    if (client_traj_finish.call(srv_traj_finish))
    {
        ROS_INFO("Call finish_trajectory %d success!", traj_id);
    }
    else
    {
        ROS_INFO("Failed to call finish_trajectory service!");
    }

    ros::ServiceClient client_traj_start = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    cartographer_ros_msgs::StartTrajectory srv_traj_start;
    srv_traj_start.request.configuration_directory = "xxx";//.lua文件所在路径
    srv_traj_start.request.configuration_basename = "xxx.lua";//lua文件
    srv_traj_start.request.use_initial_pose = 1;
    srv_traj_start.request.initial_pose = msg->pose.pose;
    srv_traj_start.request.relative_to_trajectory_id = 0;
    if (client_traj_start.call(srv_traj_start))
    {
        // ROS_INFO("Status ", srv_traj_finish.response.status)
        ROS_INFO("Call start_trajectory %d success!", traj_id);
        traj_id++;
    }
    else
    {
        ROS_INFO("Failed to call start_trajectory service!");
    }
}
```
完毕，若有问题，欢迎沟通，大家共同学习。
