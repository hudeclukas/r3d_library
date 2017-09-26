#include "stdafx.h"

#include "ReconstructionWidget.h"

#include <BirkysNormalSegmentation.h>
#include <AnalyticalPlanesReconstruction.h>
#include <PlanePointsExamination.h>
#include <ColorGenerator.h>


r3d::ReconstructionWidget::ReconstructionWidget()
{
	m_widgetName = "Reconstruction (full)";
	m_segmentedBorder = nullptr;
	m_segmentedPlanes = nullptr;
}


r3d::ReconstructionWidget::~ReconstructionWidget()
{
}

void r3d::ReconstructionWidget::visualise(pcl::visualization::PCLVisualizer* viewer)
{
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_segmentedPlanes);
	viewer->addPointCloud<pcl::PointXYZRGB>(m_segmentedPlanes, "XYZRGB");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "XYZRGB");
	viewer->addPointCloud<pcl::PointXYZ>(m_segmentedBorder, "cchulls");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cchulls");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 1.0f, "cchulls");

	auto i = 1;
	for each (obj::Wall wall in m_dataSet.getWalls())
	{
		auto wallCloud = wall.getShape().getCoordinates().makeShared();
		pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> handler(wallCloud);
		viewer->addPointCloud<pcl::PointXYZ>(wallCloud, handler, "wall" + std::to_string(i));
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "wall" + std::to_string(i));
		viewer->addPolygon<pcl::PointXYZ>(wallCloud, (1.0 / 255.0) * wall.getR(), (1.0 / 255.0) * wall.getG(), (1.0 / 255.0) * wall.getB(), "shape" + std::to_string(i));
		//viewer->setRepresentationToSurfaceForAllActors();
		++i;
		//std::ofstream log;
		//log.open("wallcorner" + std::to_string(i) + ".log");
		//for (auto j = 0; j < wallCloud->size(); ++j)
		//{
		//	log << (*wallCloud)[j] << std::endl;
		//}
		//log.close();
	}

	////vizualizacia pre Katkine outliers
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorInliers(m_planeInliers,0,0,255);
	//viewer->addPointCloud<pcl::PointXYZRGB>(m_planeInliers, colorInliers, m_dataSet.getName() + "XYZRGB-I");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_dataSet.getName() + "XYZRGB-I");
	//
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorOutliers(m_planeOutliers);
	//viewer->addPointCloud<pcl::PointXYZRGB>(m_planeOutliers, colorOutliers, m_dataSet.getName() + "XYZRGB-O");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, m_dataSet.getName() + "XYZRGB-O");

}

bool r3d::ReconstructionWidget::calculate()
{
	std::ofstream log;
	time_t time = std::time(nullptr);
	char * dt = ctime(&time);
	std::string file;
	std::string name("Reconstruct_");
	std::string type(".log");
	file = name + std::to_string(time) + type;
	log.open(file);

	mtds::BirkysNormalSegmentation birkys;
	mtds::AnalyticalPlanesReconstruction reconstruction;

	birkys.setConfiguration(m_configNode->FirstChildElement(birkys._config_name));
	reconstruction.setConfiguration(m_configNode->FirstChildElement(reconstruction._config_name));

	log << "Reconstruction - started <at> " << dt << endl;
	log << "Segmentation step - clusters extraction start <at> " << dt << endl;

	std::vector <pcl::PointIndices> clusters;
	birkys.setData(m_dataSet.getXYZData());
	//problem with vtk dataset... wrong normals
	//if (!m_dataSet.getNormalData().empty())
	//{
	//	birkys.setData(m_dataSet.getNormalData(), "normals");
	//	log << "Growing Region - normals set"<< endl;
	//}
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud = birkys.segmentClusters(&clusters);

	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Segmentation step - clusters extracted size:" << clusters.size() << " <at> " << dt << endl;

	m_segmentedPlanes = coloredCloud;

	auto result = reconstruction.reconstruct(coloredCloud, clusters, m_dataSet.getWalls());
	if (result == false) 
	{
		return false;
	}
	auto clustersCoefficients = reconstruction.getClusterCoefficients();
	auto standardDeviations = reconstruction.getStandardDeviations();
	m_segmentedBorder = reconstruction.getSegmentedBorder();

	/*!< Begin of outlier segmentation */
	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Outliers examination step - clusters examination started <at> " << dt << endl;

	r3d::mtds::PlanePointsExamination examiner(coloredCloud, clusters, clustersCoefficients, standardDeviations);
	examiner.setConfiguration(m_configNode->FirstChildElement(examiner._config_name));
	examiner.computePlanePoints();

	m_planeSegments = examiner.getSegmentedClustersPlanes();
	//m_outliersSegments = examiner.getSegmentedOutliers();

	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Outliers examination step - clusters examination finished; segmented_planes = " << m_planeSegments.size() << " <at> " << dt << endl;

	
	if (m_planeSegments.size() > 0)
	{
		m_planeInliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		m_planeOutliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		for each (r3d::prims::PlaneSegment segment in (m_planeSegments))
		{
			/*!< Color outliers */
			ColorGenerator::paintPointCloud(segment.outliers);
			/*!<Merge inliers in one cloud and outliers in one cloud*/
			(*m_planeInliers).insert((*m_planeInliers).end(), segment.inliersPoints.begin(), segment.inliersPoints.end());
			(*m_planeOutliers).insert((*m_planeOutliers).end(), segment.outliers.begin(), segment.outliers.end());
		}
	}

	auto foundPotencialObjects = examiner.findPotentialObjectsFromOutliers();
	for each (obj::Object* obj in foundPotencialObjects)
	{
		m_dataSet.addObject(*obj);
	}

	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Outliers examination step - outliers examined <at> " << dt << endl;
	log.close();

	return true;
}
