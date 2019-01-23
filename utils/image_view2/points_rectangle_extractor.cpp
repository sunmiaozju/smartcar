#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

//#include <image_view2/PointArrayStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

class PointsRectExtractor
{
  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
                                                           geometry_msgs::PolygonStamped > PolygonApproxSyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
                                                           geometry_msgs::PointStamped > PointApproxSyncPolicy;

private:
  ros::NodeHandle pnode_;
  ros::Publisher pub_;
  int queue_size;
  int crop_width;

  message_filters::Subscriber < sensor_msgs::PointCloud2 > points_sub_;
  message_filters::Subscriber < geometry_msgs::PolygonStamped > rect_sub_;
  message_filters::Subscriber < geometry_msgs::PointStamped > point_sub_;

  boost::shared_ptr < message_filters::Synchronizer < PolygonApproxSyncPolicy > > sync_a_polygon_;
  boost::shared_ptr < message_filters::Synchronizer < PointApproxSyncPolicy > > sync_a_point_;

public:
  PointsRectExtractor() : pnode_("~"), queue_size(200), crop_width(2)
  {
    rect_sub_.subscribe (pnode_, "rect", queue_size);
    point_sub_.subscribe (pnode_, "point", queue_size);
    points_sub_.subscribe (pnode_, "points", queue_size);

    sync_a_polygon_ = boost::make_shared < message_filters::Synchronizer< PolygonApproxSyncPolicy > > (queue_size);
    sync_a_polygon_->connectInput (points_sub_, rect_sub_);
    sync_a_polygon_->registerCallback (boost::bind (&PointsRectExtractor::callback_polygon, this, _1, _2));

    sync_a_point_ = boost::make_shared < message_filters::Synchronizer< PointApproxSyncPolicy > > (queue_size);
    sync_a_point_->connectInput (points_sub_, point_sub_);
    sync_a_point_->registerCallback (boost::bind (&PointsRectExtractor::callback_point, this, _1, _2));

    pub_ = pnode_.advertise< sensor_msgs::PointCloud2 > ("output", 1);
  }

  void callback_point(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                      const geometry_msgs::PointStampedConstPtr& array_ptr) {
    int st_x = array_ptr->point.x - crop_width;
    int st_y = array_ptr->point.y - crop_width;
    int ed_x = array_ptr->point.x + crop_width;
    int ed_y = array_ptr->point.y + crop_width;

    int wd = points_ptr->width;
    int ht = points_ptr->height;
    int rstep = points_ptr->row_step;
    int pstep = points_ptr->point_step;
    if (st_x < 0) st_x = 0;
    if (st_y < 0) st_x = 0;
    if (ed_x >= wd) ed_x = wd - 1;
    if (ed_y >= ht) ed_y = ht - 1;

    sensor_msgs::PointCloud2 pt;
    pt.header = points_ptr->header;
    pt.width = ed_x - st_x + 1;
    pt.height = ed_y - st_y + 1;
    pt.row_step = pt.width * pstep;
    pt.point_step = pstep;
    pt.is_bigendian = false;
    pt.fields = points_ptr->fields;
    pt.is_dense = false;
    pt.data.resize(pt.row_step * pt.height);

    unsigned char * dst_ptr = &(pt.data[0]);

    for (size_t idx_y = st_y; idx_y <= ed_y; idx_y++) {
      for (size_t idx_x = st_x; idx_x <= ed_x; idx_x++) {
        const unsigned char * src_ptr = &(points_ptr->data[idx_y * rstep + idx_x * pstep]);
        memcpy(dst_ptr, src_ptr, pstep);
        dst_ptr += pstep;
      }
    }
    pub_.publish (pt);
  }

  void callback_polygon(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                        const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
    // Solve all of perception here...
    ROS_DEBUG("callback");
#if 0
    if(pub_.getNumSubscribers()==0 ){
      ROS_INFO("number of subscribers is 0, ignoring image");
      return;
    }
#endif

    if (array_ptr->polygon.points.size() > 1) {
      int st_x = array_ptr->polygon.points[0].x;
      int st_y = array_ptr->polygon.points[0].y;
      int ed_x = array_ptr->polygon.points[1].x;
      int ed_y = array_ptr->polygon.points[1].y;
      int wd = points_ptr->width;
      int ht = points_ptr->height;
      int rstep = points_ptr->row_step;
      int pstep = points_ptr->point_step;

      sensor_msgs::PointCloud2 pt;
      pt.header = points_ptr->header;
      pt.width = ed_x - st_x + 1;
      pt.height = ed_y - st_y + 1;
      pt.row_step = pt.width * pstep;
      pt.point_step = pstep;
      pt.is_bigendian = false;
      pt.fields = points_ptr->fields;
      pt.is_dense = false;
      pt.data.resize(pt.row_step * pt.height);

      unsigned char * dst_ptr = &(pt.data[0]);

      for (size_t idx_y = st_y; idx_y <= ed_y; idx_y++) {
        for (size_t idx_x = st_x; idx_x <= ed_x; idx_x++) {
          const unsigned char * src_ptr = &(points_ptr->data[idx_y * rstep + idx_x * pstep]);
          memcpy(dst_ptr, src_ptr, pstep);
          dst_ptr += pstep;
        }
      }
      pub_.publish (pt);
    }
  }
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "points_rectangle_extractor");

  if(!ros::master::check())
    return -1;

  PointsRectExtractor vs;

  ros::spin();

  return 0;
}
