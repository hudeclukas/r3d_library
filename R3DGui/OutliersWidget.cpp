#include "stdafx.h"
#include "OutliersWidget.h"
#include "BirkysNormalSegmentation.h"
#include "PlanePointsExamination.h"
#include "ColorGenerator.h"
#include "DataStatistics.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>


r3d::OutliersWidget::OutliersWidget()
{
	m_widgetName = "Outliers (segmentation)";
}


r3d::OutliersWidget::~OutliersWidget()
{
}

bool r3d::OutliersWidget::calculate()
{
	mtds::BirkysNormalSegmentation birkys;
	birkys.setConfiguration(m_configNode->FirstChildElement(birkys._config_name));
	birkys.setData(m_dataSet.getXYZData());
	
	if (!m_dataSet.getNormalData().empty())
	{
		birkys.setData(m_dataSet.getNormalData());
	}

	std::vector <pcl::PointIndices> clusters;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr coloredCloud = birkys.segmentClusters(&clusters);

	std::vector<Eigen::Vector4f> coefficients;
	std::vector<long double> standardDeviations;
	data::DataStatistics dataStatistics;
	for (auto i = 0; i < clusters.size(); ++i)
	{
		Eigen::Vector4f coeffs;
		CommonUtil::planeCoefficients<pcl::PointXYZRGB>(*coloredCloud, clusters[i], coeffs);
		coefficients.push_back(coeffs);
		dataStatistics.setCloud(coloredCloud, clusters[i]);
		dataStatistics.setPlaneCoefs(coeffs);
		dataStatistics.computeRoughness();
		standardDeviations.push_back(dataStatistics.getStandardDeviation());
	}
	mtds::PlanePointsExamination examiner(coloredCloud, clusters, coefficients, standardDeviations);
	examiner.setConfiguration(m_configNode->FirstChildElement(examiner._config_name));
	examiner.computePlanePoints();

	auto planeSegments = examiner.getSegmentedClustersPlanes();
	

	if (planeSegments.size() > 0)
	{
		m_planeInliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		m_planeOutliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		for each (r3d::prims::PlaneSegment segment in (planeSegments))
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
	return true;
}

void r3d::OutliersWidget::visualise(pcl::visualization::PCLVisualizer* viewer)
{
	//vizualizacia pre Katkine outliers
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorInliers(m_planeInliers,0,0,255);
	viewer->addPointCloud<pcl::PointXYZRGB>(m_planeInliers, colorInliers, m_dataSet.getName() + "XYZRGB-I");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_dataSet.getName() + "XYZRGB-I");
	
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> colorOutliers(m_planeOutliers);
	viewer->addPointCloud<pcl::PointXYZRGB>(m_planeOutliers, colorOutliers, m_dataSet.getName() + "XYZRGB-O");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, m_dataSet.getName() + "XYZRGB-O");

}
