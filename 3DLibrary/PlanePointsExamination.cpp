#include "stdafx.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/filesystem.hpp>
#include "PlaneSegment.h"
#include "BirkysNormalSegmentation.h"
#include "PlanePointsExamination.h"


r3d::mtds::PlanePointsExamination::PlanePointsExamination()
	: Method("Outlier points examiner", "OutlierSegmentation")
{
	m_InliersDistanceParameter = 3.0;
	m_OutliersSearchRadiusParameter = 2.0;
	m_FilterMeanK = 10.0;
	m_FilterStDevMulTresh = 1.5;
	m_GRNumberOfNeighbours = 10;
	m_GRColorThreshold = 0.0;
	m_GRMinClusterSize = 10;
}


r3d::mtds::PlanePointsExamination::PlanePointsExamination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<pcl::PointIndices>& clusters, std::vector<Eigen::Vector4f>& coefficients, std::vector<long double> standardDeviation)
	: Method("Outlier points examiner", "OutlierSegmentation")
{
	m_cloudData = cloud;
	m_segmentedCloudClusters = clusters;
	m_segmentedClustersPlaneCoefficients = coefficients;
	m_stdDeviation = standardDeviation;

	m_InliersDistanceParameter = 3.0;
	m_OutliersSearchRadiusParameter = 2.0;
	m_FilterMeanK = 10.0;
	m_FilterStDevMulTresh = 1.5;
	m_GRNumberOfNeighbours = 10;
	m_GRColorThreshold = 0.0;
	m_GRMinClusterSize = 10;
}


r3d::mtds::PlanePointsExamination::~PlanePointsExamination()
{
}


void r3d::mtds::PlanePointsExamination::setConfiguration(TiXmlNode* param)
{
	m_InliersDistanceParameter = std::atof(param->FirstChildElement("InliersDistanceParameter")->GetText());
	m_OutliersSearchRadiusParameter = std::atof(param->FirstChildElement("OutliersSearchRadiusParameter")->GetText());
	m_FilterMeanK = std::atof(param->FirstChildElement("FilterMeanK")->GetText());
	m_FilterStDevMulTresh = std::atof(param->FirstChildElement("FilterStDevMulTresh")->GetText());
	m_GRNumberOfNeighbours = std::atoi(param->FirstChildElement("GRNumberOfNeighbours")->GetText());
	m_GRColorThreshold = std::atof(param->FirstChildElement("GRColorThreshold")->GetText());
	m_GRMinClusterSize = std::atoi(param->FirstChildElement("GRMinClusterSize")->GetText());
}


/*!
* \brief Computes coefficients and separates inliers and outliers for every plane segment in dataset
*/
void r3d::mtds::PlanePointsExamination::computePlanePoints()
{
	if (m_planeInliers) m_planeInliers->clear();
	else m_planeInliers = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGB>>();
	if (m_planeOutliers) m_planeOutliers->clear();
	else m_planeOutliers = boost::make_shared <pcl::PointCloud<pcl::PointXYZRGB>>();

	boost::filesystem::path dir("Histograms\\");
	create_directory(dir);
	std::ofstream log;
	log.open("planepointsexamination.log");
	time_t time = std::time(nullptr);
	char *dt = ctime(&time);

	log << "Plane Points Examination - started <at> " << dt << std::endl;

	std::ofstream histogramOutput;
	auto fileOrder = 0;
	int segmentOrder = 0;
	for each (pcl::PointIndices planeIndices in m_segmentedCloudClusters)
	{
		/*if (planeindices.indices.size() < 500)
		{
			continue;
		}*/

		histogramOutput.open("Histograms/histogram" + std::to_string(fileOrder++) + ".csv");
		histogramOutput << "Distance;Number of points;\n";

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Plane Points Examination - extract cluster points <at> " << dt << std::endl;
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		r3d::prims::PlaneSegment segment;
		//prepare extractor
		extract.setInputCloud(m_cloudData);
		extract.setIndices(boost::make_shared<pcl::PointIndices>(planeIndices));
		//extract inliers = segment points
		extract.setNegative(false);
		extract.filter(segment.cloudData);

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Plane Points Examination - cluster points extracted <at> " << dt << std::endl;

		pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p((new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(segment.cloudData.makeShared())));
		segment.coefficients = m_segmentedClustersPlaneCoefficients[segmentOrder];
		// find inliers statisticaly according to the distance from plane
		model_p->selectWithinDistance(segment.coefficients, m_InliersDistanceParameter*m_stdDeviation[segmentOrder], segment.inliers);


		extract.setInputCloud(segment.cloudData.makeShared());
		pcl::PointIndices::Ptr indices = boost::make_shared<pcl::PointIndices>();
		indices->indices = segment.inliers;
		extract.setIndices(indices);
		//extract inliers = segment points
		extract.setNegative(false);
		extract.filter(segment.inliersPoints);
		(*m_planeInliers).insert((*m_planeInliers).end(), segment.inliersPoints.begin(), segment.inliersPoints.end());


		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Plane Points Examination - segment inliers = " << segment.inliers.size() << " <at> " << dt;
		log << "Plane Points Examination - segment inliers computed; threshold=" << m_InliersDistanceParameter*m_stdDeviation[segmentOrder] << " <at> " << dt << std::endl;

		segment.computeOutliers();

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Plane Points Examination - segment outliers computed <at> " << dt << std::endl;

		m_segmentedClustersPlanes.push_back(segment);


		std::vector<int> labelsOutliers;
		labelsOutliers.resize(segment.outliers.points.size());
		for (int i = 0; i<labelsOutliers.size(); i++)
		{
			labelsOutliers[i] = 0;
		}
		std::set<int> labels;
		int l = 1;
		std::set<int> eraseLabels;
		int useLabel;
		pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(segment.outliers.makeShared());
		float radius = m_OutliersSearchRadiusParameter*m_stdDeviation[segmentOrder];
		for (int i = 0; i< segment.outliers.points.size(); i++)
		{
			std::vector<int> indicesRadiusSearch;
			std::vector<float> radiusSearchSquaredDistances;
			if (kdtree.radiusSearch(segment.outliers.points[i], radius, indicesRadiusSearch, radiusSearchSquaredDistances)>0)
			{
				eraseLabels.clear();
				for (int j : indicesRadiusSearch)
				{
					if (labelsOutliers[j] != 0)
					{
						eraseLabels.insert(labelsOutliers[j]);
					}
				}
				if (labelsOutliers[i] != 0)
				{
					eraseLabels.insert(labelsOutliers[i]);
				}
				if (eraseLabels.empty())
				{
					useLabel = l;
					labels.insert(l);
					l++;
				}
				else{
					useLabel = *(eraseLabels.begin());
					for (std::set<int>::iterator it = ++(eraseLabels.begin()); it != eraseLabels.end(); ++it)
					{
						labels.erase(*it);
						for (int m = 0; m<labelsOutliers.size(); m++)
						{
							if (labelsOutliers[m] == *it)
								labelsOutliers[m] = useLabel;
						}
					}
				}
				for (int j : indicesRadiusSearch)
				{
					labelsOutliers[j] = useLabel;
				}
				labelsOutliers[i] = useLabel;
			}
		}

		for (int label : labels)
		{
			pcl::PointCloud<pcl::PointXYZRGB> temp_segment;
			for (int i = 0; i<labelsOutliers.size(); i++)
			{
				if (labelsOutliers[i] == label)
					temp_segment.push_back(segment.outliers.points[i]);
			}
			m_segmentedOutliers.push_back(temp_segment);
		}
		segmentOrder++;
		(*m_planeOutliers).insert((*m_planeOutliers).end(), segment.outliers.begin(), segment.outliers.end());
	}
	log.close();
}

std::vector<r3d::obj::Object*> r3d::mtds::PlanePointsExamination::findPotentialObjectsFromOutliers()
{
	/*!< Filter out some noise from the outliers */
	if (m_planeInliers->size() != 0)
	{
		pcl::PointCloud<pcl::PointXYZ>* outliersXYZ = new pcl::PointCloud<pcl::PointXYZ>();
		pcl::copyPointCloud(*m_planeOutliers, *outliersXYZ);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(outliersXYZ->makeShared());
		sor.setMeanK(m_FilterMeanK);
		sor.setStddevMulThresh(m_FilterStDevMulTresh);
		std::vector<int> indeces;
		sor.filter(indeces);

		pcl::PointCloud<pcl::PointXYZRGB>* potencialObjectsCloud = new pcl::PointCloud<pcl::PointXYZRGB>();
		potencialObjectsCloud->resize(indeces.size());
		pcl::copyPointCloud<pcl::PointXYZRGB>(*m_planeOutliers, indeces, *potencialObjectsCloud);


		std::vector<obj::Object*> foundObjects;
		if (potencialObjectsCloud->size() != 0)
		{
			/*!< Seperate different potencial objects from each other by region growing */
			std::vector<pcl::PointIndices> outlierClusters;
			pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
			reg = BirkysNormalSegmentation::regionGrowingRGB(potencialObjectsCloud->makeShared(), m_GRNumberOfNeighbours, m_GRColorThreshold, m_GRMinClusterSize, &outlierClusters);
			m_planeOutliers = reg.getColoredCloud();

			/*!< Save each pointCloud of potencial object into a vector of Object in ClosedSpace*/
			for (int i = 0; i < outlierClusters.size(); i++)
			{
				r3d::obj::Object* potencialObject = new r3d::obj::Object();
				pcl::PointCloud<pcl::PointXYZ>* objectCloud = new pcl::PointCloud<pcl::PointXYZ>();
				pcl::copyPointCloud(*m_planeOutliers, outlierClusters.at(i).indices, *objectCloud);
				potencialObject->setObjectData(*objectCloud);
				foundObjects.push_back(potencialObject);
			}
		}
		return foundObjects;
	}
	return {};
}
