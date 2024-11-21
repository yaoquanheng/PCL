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
// ������������֮������  

void FilterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float min_z, float max_z)
{
	// ����PassThrough�˲�������
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("z");  // ����Ҫ���˵��ֶ�ΪZ��
	pass.setFilterLimits(min_z, max_z);  // ����Z��ķ�Χ
	pass.filter(*cloud_filtered);  // ִ���˲�������������浽cloud_filtered
}
void PointCloudBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr  normals(new  pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setSearchMethod(tree);
	normalEstimation.setRadiusSearch(0.2);  // �������İ뾶
	normalEstimation.compute(*normals);

	/*pcl����߽�*/
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>); //����һ��boundary��ָ�룬��Ϊ����ֵ
	boundaries->resize(cloud->size()); //��ʼ����С
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimation; //����һ��BoundaryEstimation��
	boundary_estimation.setInputCloud(cloud); //�����������
	boundary_estimation.setInputNormals(normals); //�������뷨��
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ptr(new pcl::search::KdTree<pcl::PointXYZ>);
	boundary_estimation.setSearchMethod(kdtree_ptr); //������Ѱk���ڵķ�ʽ
	boundary_estimation.setKSearch(200); //����k��������
	boundary_estimation.setAngleThreshold(M_PI * 0.5); //���ýǶ���ֵ��������ֵΪ�߽�
	boundary_estimation.compute(*boundaries); //������Ʊ߽磬���������boundaries��
	for (size_t i = 0; i < boundaries->size(); i++)
	{
		int flag = static_cast<int>(boundaries->points[i].boundary_point); 
		if (flag == 1)
			cloud_out->push_back(cloud->points[i]);
	}
	cout << "�߽���Ƶĵ���   ��  " << cloud_out->size() << endl;
}
double PlaneSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	//�������ģ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers�ڵ������
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	//seg.setModelType(pcl::SACMODEL_CIRCLE3D);	//�������ģ�Ͳ���
	seg.setMethodType(pcl::SAC_RANSAC);			//��Ϸ��������������
	seg.setDistanceThreshold(0.2);				//���õ㵽ֱ�߾�����ֵ
	seg.setMaxIterations(10000);				//����������
	seg.setInputCloud(cloud_in);				//�������
	seg.segment(*inliers, *coefficients);		//��ϵ���
	/*for (int i = 1; i<n; i++)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_in);
		extract.setIndices(inliers);
		extract.setNegative(true);  // �Ƴ��ڵ�
		extract.filter(*cloud_in);

		// �ڶ������ƽ��
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
	cout << "ƽ�����z��ƽ��ֵ��" << z_average << endl;
	return z_average;
}
void Circle3DSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
	
	//�������ģ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers�ڵ������
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	//seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);	//�������ģ�Ͳ���
	seg.setMethodType(pcl::SAC_RANSAC);			//��Ϸ��������������
	seg.setDistanceThreshold(0.3);				//���õ㵽ֱ�߾�����ֵ
	seg.setMaxIterations(10000);				//����������
	//seg.setRadiusLimits(8, 12);				//�뾶����
	seg.setRadiusLimits(8, 14);
	seg.setInputCloud(cloud_in);				//�������
	seg.segment(*inliers, *coefficients);		//��ϵ���
	//------------------ģ��ϵ��---------------------
	cout << "���3dԲ��ģ��ϵ��Ϊ��" << endl;
	cout << "X��" << coefficients->values[0] << endl;
	cout << "Y��" << coefficients->values[1] << endl;
	cout << "Z��" << coefficients->values[2] << endl;
	cout << "R��" << coefficients->values[3] << endl;
	cout << "nx��" << coefficients->values[4] << endl;
	cout << "ny��" << coefficients->values[5] << endl;
	cout << "nz��" << coefficients->values[6] << endl;

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
	pcl::io::loadPCDFile<pcl::PointXYZ>("H:/Chrome����/2.pcd", *cloud_in);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	coefficients->values.resize(7);
	//FilterPointCloud(cloud_in, cloud_out1, 640, 680);
	FilterPointCloud(cloud_in, cloud_out1, 100, 900);
	z = PlaneSegment(cloud_out1, coefficients, cloud_out2);
	cout << z << endl;
	FilterPointCloud(cloud_out1, cloud_out2, z-49, z-35);
	PointCloudBoundary(cloud_out2, cloud_out3);
	Circle3DSegment(cloud_out3, coefficients, cloud_out);

	cout << "�˲�ǰ�ĵ���������" << cloud_in->points.size() << endl;
	cout << "�˲���ĵ���������" << cloud_out->points.size() << endl;
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
