#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include "CustomInteractor.h"

/************************************************************************************************************************/
/*																														*/
/*																														*/
/*		running this method results in opening new PCLVisualizer window with our own CustomInteractor interaction style */
/*																														*/
/*																														*/
/************************************************************************************************************************/

int testmain(void)
{
	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis. The colour for
	// the XYZRGB cloud will gradually go from red to green to blue.
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);



	/************************************************************************************************************************/
	/*																														*/
	/*																														*/
	/*		passing CustomInteractor to PCLVisualizer constructor															*/
	/*																														*/
	/*																														*/
	/************************************************************************************************************************/

	std::cout << "This interactor style tries to introduce classic FPS movement control system into PCL/VTK\n" << std::endl;
	std::cout << "W - move forwards" << std::endl;
	std::cout << "S - move backwards" << std::endl;
	std::cout << "A - move sideways left" << std::endl;
	std::cout << "D - move sideways right" << std::endl;
	std::cout << "click once - start rotating camera" << std::endl;
	std::cout << "click one more time - stop rotating camera\n" << std::endl;

	std::cout << "R - reset camera position\n" << std::endl;

	std::cout << "TODO: - prevent cursor leaving PCLVisualizer window (lock it at center and make it invisible)" << std::endl;

	pcl::visualization::PCLVisualizerInteractorStyle* style = new r3d::visualization::CustomInteractor();
	char *abcde[] = { "These", "are", "some", "strings" };
	int argc2 = 0;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(argc2, abcde, "locked camera axis", style, true));
	
	//viewer->setFullScreen(true);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(basic_cloud_ptr, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	//viewer->setCameraPosition(0, 0, -1, 0, 0, -0.95
	//setCameraParameters(const Camera &camera, 0);
	std::vector<pcl::visualization::Camera> cameras;
	viewer->getCameras(cameras);

	pcl::visualization::Camera cam = cameras[0];

	//std::cout << cameras.size() << std::endl;
	
	cam.pos[0] = 0;
	cam.pos[1] = 0;
	cam.pos[2] = -1;

	cam.focal[0] = 0;
	cam.focal[1] = 0;
	cam.focal[2] = -0.75;
	viewer->setCameraParameters(cam, 0);
	viewer->setSize(1366, 768);

	/*! \brief main loop
	*/
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	return 0;
}