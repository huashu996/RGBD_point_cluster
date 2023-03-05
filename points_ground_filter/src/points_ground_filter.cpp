#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>
typedef pcl::PointXYZRGB PointT;

class GroundRemoverNode {
private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string sub_topic_;
    std::string pub_topic_;
public:
	bool init();
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
        //转换ros消息
        pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*input_cloud_msg, *input_cloud);
        //点云下采样
	pcl::PointCloud<PointT>::Ptr new_input_cloud(new pcl::PointCloud<PointT>);
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(input_cloud);
	sor.setLeafSize(0.03f, 0.03f, 0.03f);//设置滤波时创建的体素体积为1 cm3的立方体
	sor.filter(*new_input_cloud);
       
        //-----------方法一---------------------//
        // Create filtering object to remove points below z-axis
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(new_input_cloud);
        //pass.setFilterFieldName("z");
        //pass.setFilterLimits(0.0, 20.0);
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
        pass.filter(*filtered_cloud);

        // Segment ground plane
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(filtered_cloud);
        seg.segment(*inliers, *coefficients);

        // Extract ground plane points
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>);
        extract.filter(*ground_cloud);

        // Remove ground plane points from input cloud
        pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
        extract.setNegative(true);
        extract.filter(*output_cloud);
        //-----------方法一---------------------//
       

        /*
        //-----------方法二---------------------//
	// Create PMF filter object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndicesPtr ground (new pcl::PointIndices);
	pcl::copyPointCloud(*new_input_cloud, *cloud_xyz);//该算法只能用xyz和xyzi点云
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf_filter;
	pmf_filter.setInputCloud(cloud_xyz);
	pmf_filter.setMaxWindowSize(15); // Set maximum window size for morphological filter
	pmf_filter.setSlope(0.1f); // Set slope for max window size determination
	pmf_filter.setInitialDistance(0.1f); // Set initial distance for distance threshold
	pmf_filter.setMaxDistance(1.0f); // Set maximum distance for distance threshold
	pmf_filter.extract(ground->indices);
	// 从标号到点云
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud_xyz);
	extract.setIndices(ground);
	extract.filter(*output_cloud);
	//-----------方法二---------------------//
	*/
	
	
	// 进行坐标系转换
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	// 在 X 轴上定义一个 2.5 米的平移.
	transform.translation() << 0.0, 0.0, 0.0;
	// 和前面一样的旋转; Z 轴上旋转 theta 弧度
	float theta = M_PI; // 弧度角
	//float theta = 0; // 弧度角
	transform.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitY()));
	transform.rotate (Eigen::AngleAxisf (-M_PI/2, Eigen::Vector3f::UnitZ()));
	// 打印变换矩阵
	printf ("\nMethod #2: using an Affine3f\n");
	std::cout << transform.matrix() << std::endl;
	pcl::transformPointCloud(*output_cloud, *output_cloud, transform);//用pcl不要用pcl_ros
	output_cloud->header.frame_id = "word";
	
	
	// 将PCL格式的点云转换为ROS消息
	sensor_msgs::PointCloud2 output_cloud_msg;
	output_cloud_msg.header.frame_id = output_cloud->header.frame_id;
	pcl::toROSMsg(*output_cloud, output_cloud_msg);
        // Publish output point cloud
        pub_.publish(output_cloud_msg);
        //发布TF坐标系 红x 绿y 蓝z
	static tf::TransformBroadcaster br;
	tf::Transform transform2;
	//平移
	transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	//旋转
	tf::Quaternion q;
	//q.setRPY(0, 0, M_PI_2);
	q.setRPY(0, 0, 0);
	transform2.setRotation(q);
	//发布
	br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "word", "ground"));
    }

};
bool GroundRemoverNode::init()//定义ImageEnhancement类的成员函数
{
	ros::NodeHandle nh_;//开启节点对象nh
	nh_.param<std::string>("sub_topic", sub_topic_ , "/camera/depth/color/points");
	nh_.param<std::string>("pub_ground_topic", pub_topic_ , "/points_ground");
	// Subscribe to input point cloud
	sub_ = nh_.subscribe(sub_topic_, 1, &GroundRemoverNode::pointCloudCallback, this);
	// Advertise output point cloud
	pub_ = nh_.advertise<pcl::PointCloud<PointT>>(pub_topic_, 1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "points_ground_filter_node");
    GroundRemoverNode node;
    node.init();//调用类的成员函数
    ros::spin();
    return 0;
}
