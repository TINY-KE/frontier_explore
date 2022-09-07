/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01; 
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  // 通过rviz输入/initialpose，可以修改机器人在map中的global_pose，因此会影响到frontier的搜索。
  global_pose_sub_ = private_nh_.subscribe("/initialpose",1000, &Explore::global_pose_rviz, this);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  private_nh_.param("only_viusalize_frontier", only_viusalize_frontier,true);
  if(!only_viusalize_frontier)
  {
    ROS_INFO("Waiting to connect to move_base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");
  }

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),   /* ros::Duration为每一周期的时长，当前为3秒一次。 */
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

// 回调函数
void  Explore::global_pose_rviz( const geometry_msgs::PoseWithCovarianceStamped  &msg ){
  global_pose.position = msg.pose.pose.position;
  global_pose.orientation = msg.pose.pose.orientation;
  ROS_ERROR("Receive robot global pose from rviz");
}

// 可视化边界
void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1.0;
  yellow.g = 1.0;
  yellow.b = 0;
  yellow.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;     /* 这里用了&符号 */
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(1.2);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;  /* frontiers被排序后，front为cost最小的frontier */

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {   /* 对每一个 frontier 分别处理 */
    // frontier上的每个cell
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {}; 
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;  
    if (goalOnBlacklist(frontier.centroid)) {  /* 到打不了的（黑名单中的）边界为红色，否则为蓝色  */
      m.color = red;
    } else {
      m.color = blue;  
    }
    markers.push_back(m);
    ++id;
    
    // frontier的initial cell
    // m.type = visualization_msgs::Marker::SPHERE;
    // m.id = int(id);
    // m.pose.position = frontier.initial;
    // // scale frontier according to its cost (costier frontiers will be smaller)
    // double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5); 
    // m.scale.x = scale/2.0;
    // m.scale.y = scale/2.0;
    // m.scale.z = scale/2.0;
    // m.points = {};
    // m.color = green;
    // markers.push_back(m);
    // ++id;

    // frontier的centroid cell TODO:  应该将goal也可视化出来。颜色用黄色
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale2 = std::min(std::abs(min_cost * 0.2 / frontier.cost), 0.5); 
    m.scale.x = scale2;
    m.scale.y = scale2;
    m.scale.z = scale2;
    m.points = {};
    m.color = yellow;
    markers.push_back(m);
    ++id; 
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused    /* 可能之前的marker比现在多。所以多的marker没有被替代掉，得手动删除。 */
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

// TODO: 为什么不起作用？？
void Explore::rviz_goal( const geometry_msgs::Point& goal ){
    visualization_msgs::Marker m;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "frontiers";
    m.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    m.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    m.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    m.pose.position.x = goal.x;
    m.pose.position.y = goal.y;
    m.pose.position.z = goal.z;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.y = 0.0;
    m.pose.orientation.z = 0.0;
    m.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    m.scale.x = 4.0;
    m.scale.y = 3.0;
    m.scale.z = 3.0;

    // Set the color -- be sure to set alpha to something non-zero!
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(1.3);
    marker_array_publisher_.publish(m);
}
void Explore::rviz_clear(){
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    visualization_msgs::Marker m;
    
    int id = 0;
    m.action = visualization_msgs::Marker::DELETE;
    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = costmap_client_.getRobotPose().position;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.points = {};
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;
    markers.push_back(m);
    id ++;
    // lives forever
    m.lifetime = ros::Duration(0);
    m.frame_locked = true;
    for (; id < 50; ++id) {
      m.id = int(id);
      markers.push_back(m);
    }
    marker_array_publisher_.publish(markers_msg);
}
void Explore::makePlan()
{ 
  ROS_DEBUG("start make plan");


// ****************一：从costmap中计算出，所有的frontiers
  // find frontiers   TODO:这是做什么？ 如果不实时更新global_pose（only_viusalize_frontier为true时），则frontier选择时，始终是离起始点最近的frontier。
  if(!only_viusalize_frontier)
    global_pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(global_pose.position);  /*  返回的是 vector<Frontier>。   关键：找到边界的挑选准则,是直接选择最近的吗？？ */
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    // TODO:应该发送一个空topic，清空rviz中的marker
    rviz_clear();
    return;
  }

  // publish froniers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }


// ****************二：从frontiers找到，不在黑名单的target_position
  // find non blacklisted（TODO: 有些黑名单中的goal，可能过后又能抵达了呢？？） frontier   
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f  /* 形参。对应的实参为frontiers中的各项 */) {
                         return goalOnBlacklist(f.centroid);  
                         /* 这个函数返回的是true时，f.centroid在黑名单。   */
                        /* 又因为是find if not， 所以为false的时候，return。*/
                       });
  if (frontier == frontiers.end()) {
    stop();
    return;
  }
  geometry_msgs::Point target_position = frontier->centroid;  /* 关键： 终点是边界的中心。 但可视化是 最近点。  */



// ****************三：对target_position进行最后的检查：（1）target_position是否是新的goal  （2）到新target_position的距离，是否比旧target更近。满足其中一条，则切换到新target。
  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  // TODO: 原来：新search到的frontier，只要比当前正前往的distance近，就换目标，是不合理的。应该有有个比例。
  // 例如改为（prev_distance_ / frontier->min_distance） > 1.4   
  // TODO:  是不是 exploring_timer_ 这个周期函数中，已经设置的stop()，也能起到这个作用
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  

// ****************四：黑名单：如果一个goal无法到达（或者花了很久 也无法到达），则加入 黑名单。
  // black list if we've made no progress for a long time  
  if (ros::Time::now() - last_progress_/* 只有!same_goal时，last_progress_才会被重置 */ 
            > progress_timeout_) 
  {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  // we don't need to do anything if we still pursuing the same goal   检查目标店是否和上一次重复
  if (same_goal) {
    return;
  }



// ****************五：TODO:   可视化目标点goal
  rviz_goal(target_position);
// ****************六： movebase发送目标点goal
  // send goal to move_base if we have something new to pursue 
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });


}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing      /* 到过的goal，进入黑名单 */
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())   /* && ： 即要同时满足 x靠近，  y也靠近。  */
      return true;   /* 在黑名单 */
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");   
  }
  
  // TODO: 这是做什么？
  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
