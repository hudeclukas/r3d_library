#include "stdafx.h"
#include "CloudRotationTest.h"
#include "Wall.h"
#include "Rectangle.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>
#include <vector>
#include <math.h>
#include <random>
#include <pcl/surface/concave_hull.h>
#include <ParameterGrabber.h>

r3d::mtds::CloudRotationTest::CloudRotationTest()
	: Method("Rotate cloud - 3", "RotateCloud")
{
	_config_name = ("CloudRotationTest");
}


r3d::mtds::CloudRotationTest::~CloudRotationTest()
{
}

void r3d::mtds::CloudRotationTest::setConfiguration(TiXmlNode* param) {
	if (param != NULL && param->FirstChild() != NULL) {
		m_minPointsCount = std::atoi(PGrabber::getParameter(param, "MinPointsCount"));
		m_sphereSize = std::atof(PGrabber::getParameter(param, "SphereSize"));
	}
}

bool r3d::mtds::CloudRotationTest::calculate()
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ply = m_dataSet.getXYZData().makeShared();




	pcl::PointCloud<pcl::PointXYZ>::Ptr remainingCloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr wallsX(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr wallsY(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr wallsZ(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr wall(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr> > wx;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr> > wy;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr> > wz;

	std::vector<pcl::ModelCoefficients> cx;
	std::vector<pcl::ModelCoefficients> cy;
	std::vector<pcl::ModelCoefficients> cz;

	// filtrovanie - downsampling = zmensenie hustoty point cloudu
	/*pcl::VoxelGrid<pcl::PointXYZ> sor;
	//pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(ply);
	sor.setLeafSize(0.03f, 0.03f, 0.03f);
	sor.filter(*ply);*/

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	float theta = -M_PI / 2;// / 4;
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*ply, *cloud, transform_2);

	klaud = cloud->makeShared();
	// here we will store plane coefficients for floor, ceiling and walls
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

	//	now we start with segmentation

	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());	// <- points that are part of a wall will be put into this variable
	pcl::ExtractIndices<pcl::PointXYZ> extract;	//	we need this for extracting certain points (walls) from point cloud

	//	set up segmentation object.
	segmentation.setOptimizeCoefficients(false);
	segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setMaxIterations(1600);					//	try changing these parameters if walls are not detected properly
	segmentation.setDistanceThreshold(0.03);
	segmentation.setEpsAngle(pcl::deg2rad(3.0));




	int pointcount = m_minPointsCount;//3850;

	//	segmentate and extract first wall
	segmentation.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));	// find plane perpendicular to this vector (floor and ceiling)

	for (int i = 0; i<5; i++) {
		segmentation.setInputCloud(cloud);

		segmentation.segment(*inliers2, *coefficients);			// -> after this line, inliers2 will hold indexes of those points, which are part of newly detected wall.
		cx.push_back(*coefficients);
		//new pcl::ModelCoefficients();

		// coefficients will contain plane coefficients of that wall
		extract.setInputCloud(cloud);
		extract.setIndices(inliers2);
		extract.setNegative(false);
		extract.filter(*wall);

		if (wall->width > pointcount)
			wx.push_back(wall->makeShared());
		std::cout << wall->width << std::endl;

		extract.setNegative(true);
		extract.filter(*cloud);

	} //while (wall->width > pointcount);
	std::cout << "----------" << std::endl;
	segmentation.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));

	for (int i = 0; i<5; i++) {
		//do {
		segmentation.setInputCloud(cloud);
		segmentation.segment(*inliers2, *coefficients);			// -> after this line, inliers2 will hold indexes of those points, which are part of newly detected wall.
		cy.push_back(*coefficients);
		// coefficients will contain plane coefficients of that wall
		extract.setInputCloud(cloud);
		extract.setIndices(inliers2);
		extract.setNegative(false);
		extract.filter(*wall);

		if (wall->width > pointcount)
			wy.push_back(wall->makeShared());
		std::cout << wall->width << std::endl;

		extract.setNegative(true);
		extract.filter(*cloud);
	} //while (wall->width > pointcount);

	std::cout << "----------" << std::endl;
	segmentation.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
	for (int i = 0; i<5; i++) {
		segmentation.setInputCloud(cloud);
		segmentation.segment(*inliers2, *coefficients);			// -> after this line, inliers2 will hold indexes of those points, which are part of newly detected wall.
		cz.push_back(*coefficients);
		// coefficients will contain plane coefficients of that wall
		extract.setInputCloud(cloud);
		extract.setIndices(inliers2);
		extract.setNegative(false);
		extract.filter(*wall);

		if (wall->width > pointcount)
			wz.push_back(wall->makeShared());
		std::cout << wall->width << std::endl;

		extract.setNegative(true);
		extract.filter(*cloud);
	}

	remainingCloud = cloud->makeShared();

	std::vector<pcl::PointXYZ> pole_bodov;
	std::vector<int> koeficientyx;
	std::vector<int> koeficientyy;
	std::vector<int> koeficientyz;

	// vzajomne pretinanie
	for (int i = 0; i< wx.size(); i++) {
		for (int j = 0; j< wy.size(); j++) {
			for (int k = 0; k< wz.size(); k++) {
				Eigen::Vector4f rovina1(cx[i].values.data());
				Eigen::Vector4f rovina2(cy[j].values.data());
				Eigen::Vector4f rovina3(cz[k].values.data());

				Eigen::Vector3f roh;

				// intersection method with planes (room walls) to get particular corners
				pcl::threePlanesIntersection(rovina1, rovina2, rovina3, roh);

				pcl::PointXYZ bod;
				bod.getVector3fMap() = roh;


				if (true) {
					int l = 0;
					int pointcounterx = 0;
					int pointcountery = 0;
					int pointcounterz = 0;
					float velkostgule = m_sphereSize;

					while (l < wx[i]->size()) {

						double vzd = sqrt(
							pow(wx[i]->points[l].x - bod.x, 2) +
							pow(wx[i]->points[l].y - bod.y, 2) +
							pow(wx[i]->points[l].z - bod.z, 2)
							);

						if (vzd <velkostgule) {

							wallsX->push_back(wx[i]->points[l]);
							pointcounterx++;
						}
						l++;
					}
					l = 0;

					while (l < wy[j]->size()) {
						double vzd = sqrt(
							pow(wy[j]->points[l].x - bod.x, 2) +
							pow(wy[j]->points[l].y - bod.y, 2) +
							pow(wy[j]->points[l].z - bod.z, 2)
							);
						if (vzd <velkostgule) {

							wallsY->push_back(wy[j]->points[l]);
							pointcountery++;
						}
						l++;
					}

					l = 0;
					while (l < wz[k]->size()) {
						double vzd = sqrt(
							pow(wz[k]->points[l].x - bod.x, 2) +
							pow(wz[k]->points[l].y - bod.y, 2) +
							pow(wz[k]->points[l].z - bod.z, 2)
							);
						if (vzd <velkostgule) {

							wallsZ->push_back(wz[k]->points[l]);
							pointcounterz++;
						}
						l++;
					}

					if (pointcounterx > 0 && pointcountery>0 && pointcounterz > 0) {
						//viewer->addSphere(bod,0.1,0.9,0.9,0.1,std::to_string(i)+std::to_string(j)+std::to_string(k));
						koeficientyx.push_back(i);
						koeficientyy.push_back(j);
						koeficientyz.push_back(k);
						pole_bodov.push_back(bod);
					}
				}
			}
		}
	}


	std::array<pcl::PointCloud<pcl::PointXYZ>, 20> polygonyX;
	std::array<pcl::PointCloud<pcl::PointXYZ>, 20> polygonyY;
	std::array<pcl::PointCloud<pcl::PointXYZ>, 20> polygonyZ;
	pcl::ConvexHull<pcl::PointXYZ> cHull;

	for (int i = 0; i<pole_bodov.size(); i++){
		polygonyX[koeficientyx[i]].points.push_back(pole_bodov[i]);
		polygonyY[koeficientyy[i]].points.push_back(pole_bodov[i]);
		polygonyZ[koeficientyz[i]].points.push_back(pole_bodov[i]);
	}

	for (int i = 0; i<20; i++){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pol(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr polX(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr polY(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr polZ(new pcl::PointCloud<pcl::PointXYZ>);

		for (int j = 0; j<polygonyX[i].points.size(); j++){
			//viewer->addSphere(polygonyX[i].points[j],0.15,0.3,0.4,0.5,"gx"+std::to_string(i)+std::to_string(j));
			pol->points.push_back(polygonyX[i].points[j]);
		}

		cHull.setInputCloud(pol);
		cHull.reconstruct(*polX);

		if (polX->width > 3){
			r3d::obj::Wall stena(*polX);
			m_dataSet.addWall(stena);
		}
		//viewer->addPolygon<pcl::PointXYZ>(polX->makeShared(), 0.6, 0.3, 0.4, "polygonx" + std::to_string(i)); //added for presentation
		pol->points.clear();

		for (int j = 0; j<polygonyY[i].points.size(); j++){
			//viewer->addSphere(polygonyY[i].points[j],0.15,0.3,0.4,0.5,"gy"+std::to_string(i)+std::to_string(j));
			pol->points.push_back(polygonyY[i].points[j]);
		}

		cHull.setInputCloud(pol);
		cHull.reconstruct(*polY);

		if (polY->width > 3){
			r3d::obj::Wall stena2(*polY);
			m_dataSet.addWall(stena2);
		}
		//viewer->addPolygon<pcl::PointXYZ>(polY->makeShared(), 0.3, 0.7, 0.4, "polygony" + std::to_string(i)); //added for presentation
		pol->points.clear();

		for (int j = 0; j<polygonyZ[i].points.size(); j++) {
			//viewer->addSphere(polygonyZ[i].points[j],0.15,0.3,0.4,0.5,"gz"+std::to_string(i)+std::to_string(j));
			pol->points.push_back(polygonyZ[i].points[j]);
		}

		cHull.setInputCloud(pol);
		cHull.reconstruct(*polZ);

		if (polZ->width > 3){
			r3d::obj::Wall stena3(*polZ);
			m_dataSet.addWall(stena3);
		}
		//viewer->addPolygon<pcl::PointXYZ>(polZ->makeShared(), 0.3, 0.3, 0.8, "polygonz" + std::to_string(i)); //added for presentation
		pol->points.clear();
	}
	return true;
}

void r3d::mtds::CloudRotationTest::visualise(pcl::visualization::PCLVisualizer* viewer){
	
	int size = polygon.size();

	for (int i = 0; i<m_dataSet.getWalls().size(); i++) {
		std::string s = std::to_string(i);
		float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX); // random colour of polygons
		viewer->addPolygon<pcl::PointXYZ>(m_dataSet.getWalls()[i].getShape().getCoordinates().makeShared(), r, 1 - r, r, s);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, 0.2, 1 - r, s);
	}

	viewer->addPointCloud(klaud, "klaud");
	viewer->setRepresentationToSurfaceForAllActors();
}
