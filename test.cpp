#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include<pcl/common/common.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <random> 
using namespace std;
// 函数：在两点之间插入点  

void FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float min_z, float max_z)
{
	// 创建PassThrough滤波器对象
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");  // 设置要过滤的字段为Z轴
	pass.setFilterLimits(min_z, max_z);  // 设置Z轴的范围
	pass.filter(*cloud_filtered);  // 执行滤波，并将结果保存到cloud_filtered
}
void PointCloudBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr  normals(new  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setRadiusSearch(0.2);  // 法向量的半径
	normalEstimation.compute(*normals);

	/*pcl计算边界*/
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //声明一个boundary类指针，作为返回值
	boundaries->resize(cloud->size()); //初始化大小
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //声明一个BoundaryEstimation类
	boundary_estimation.setInputCloud(cloud); //设置输入点云
	boundary_estimation.setInputNormals(normals); //设置输入法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
	boundary_estimation.setSearchMethod(kdtree_ptr); //设置搜寻k近邻的方式
	boundary_estimation.setKSearch(200); //设置k近邻数量
	boundary_estimation.setAngleThreshold(M_PI * 0.5); //设置角度阈值，大于阈值为边界
	boundary_estimation.compute(*boundaries); //计算点云边界，结果保存在boundaries中
	for (size_t i = 0; i < boundaries->size(); i++)
	{
		int flag = static_cast<int>(boundaries->points[i].boundary_point); 
		if (flag == 1)
			cloud_out->push_back(cloud->points[i]);
	}
	cout << "边界点云的点数   ：  " << cloud_out->size() << endl;
}
double PlaneSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	//创建拟合模型
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers内点的索引
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setModelType(pcl::SACMODEL_CIRCLE3D);	//设置拟合模型参数
	seg.setMethodType(pcl::SAC_RANSAC);			//拟合方法：随机采样法
	seg.setDistanceThreshold(0.2);				//设置点到直线距离阈值
	seg.setMaxIterations(10000);				//最大迭代次数
	seg.setInputCloud(cloud_in);				//输入点云
	seg.segment(*inliers, *coefficients);		//拟合点云
	/*for (int i = 1; i<n; i++)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_in);
		extract.setIndices(inliers);
		extract.setNegative(true);  // 移除内点
		extract.filter(*cloud_in);

		// 第二次拟合平面
		seg.setInputCloud(cloud_in);
		seg.segment(*inliers, *coefficients);
	}*/
	pcl::copyPointCloud(*cloud_in, inliers->indices, *cloud_out);
	double z_sum = 0.0;
	for (const auto& point : *cloud_out)
	{
		z_sum += point.z;
	}
	double z_average = z_sum / cloud_out->size();
	cout << "平面点云z轴平均值：" << z_average << endl;
	return z_average;
}
void Circle3DSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	
	//创建拟合模型
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers内点的索引
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	//seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);	//设置拟合模型参数
	seg.setMethodType(pcl::SAC_RANSAC);			//拟合方法：随机采样法
	seg.setDistanceThreshold(0.3);				//设置点到直线距离阈值
	seg.setMaxIterations(10000);				//最大迭代次数
	//seg.setRadiusLimits(8, 12);				//半径限制
	seg.setRadiusLimits(8, 14);
	seg.setInputCloud(cloud_in);				//输入点云
	seg.segment(*inliers, *coefficients);		//拟合点云
	//------------------模型系数---------------------
	cout << "拟合3d圆的模型系数为：" << endl;
	cout << "X：" << coefficients->values[0] << endl;
	cout << "Y：" << coefficients->values[1] << endl;
	cout << "Z：" << coefficients->values[2] << endl;
	cout << "R：" << coefficients->values[3] << endl;
	cout << "nx：" << coefficients->values[4] << endl;
	cout << "ny：" << coefficients->values[5] << endl;
	cout << "nz：" << coefficients->values[6] << endl;

	pcl::copyPointCloud(*cloud_in, inliers->indices, *cloud_out);
}
int main()
{
	double z = 0.0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("H:/Chrome下载/2.pcd", *cloud_in);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	coefficients->values.resize(7);
	//FilterPointCloud(cloud_in, cloud_out1, 640, 680);
	FilterPointCloud(cloud_in, cloud_out1, 100, 900);
	z = PlaneSegment(cloud_out1, coefficients, cloud_out2);
	cout << z << endl;
	FilterPointCloud(cloud_out1, cloud_out2, z-49, z-35);
	PointCloudBoundary(cloud_out2, cloud_out3);
	Circle3DSegment(cloud_out3, coefficients, cloud_out);

	cout << "滤波前的点云数量：" << cloud_in->points.size() << endl;
	cout << "滤波后的点云数量：" << cloud_out->points.size() << endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("3D_Viewer"));

	int v1(0);
	view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	view->setBackgroundColor(0, 0, 0, v1);
	view->addText("input cloudpoint", 10, 10, "v1", v1);
	int v2(0);
	view->createViewPort(0.5, 0.0, 1, 1.0, v2);
	view->setBackgroundColor(0.1, 0.1, 0.1, v2);
	view->addText("output cloudpoint", 10, 10, "v2", v2);

	view->addPointCloud<pcl::PointXYZ>(cloud_out3, "cloud_in", v1);
	view->addPointCloud<pcl::PointXYZ>(cloud_out, "cloud_out", v2);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud_in", v1);
	view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_out", v2);
	view->addCircle(*coefficients, "circle",v2);
	view->addCoordinateSystem(1.0);
	view->initCameraParameters();
	while (!view->wasStopped())
	{
		view->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;                                                                                                                                                                           
}
