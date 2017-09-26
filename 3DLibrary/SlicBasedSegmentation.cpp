#include "stdafx.h"
#include "SlicBasedSegmentation.h"
#include "BirkysNormalSegmentation.h"
#include "ParameterGrabber.h"

r3d::mtds::SlicBasedSegmentation::SlicBasedSegmentation()
	: Method("SLIC", "SlicBasedSegmentation")
	, m_OMPNumberOfCores(4)
	, m_NENumberOfNeighbours(100)
	, m_NumberOfIterations(5)
	, m_SuperpointCompactness(10.0)
	, m_GridSize(0.1)
{
	m_segmented_cloud = nullptr;
}


r3d::mtds::SlicBasedSegmentation::~SlicBasedSegmentation()
{
}

std::vector<pcl::PointXYZ> r3d::mtds::SlicBasedSegmentation::estimateBoundaryBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointXYZ min, max;
	min.x = cloud->points[0].x;
	min.y = cloud->points[0].y;
	min.z = cloud->points[0].z;
	max = min;
	for (int i = 1; i < cloud->points.size(); i++)
	{
		min.x = min.x > cloud->points[i].x ? cloud->points[i].x : min.x;
		min.y = min.y > cloud->points[i].y ? cloud->points[i].y : min.y;
		min.z = min.z > cloud->points[i].z ? cloud->points[i].z : min.z;

		max.x = max.x < cloud->points[i].x ? cloud->points[i].x : max.x;
		max.y = max.y < cloud->points[i].y ? cloud->points[i].y : max.y;
		max.z = max.z < cloud->points[i].z ? cloud->points[i].z : max.z;
	}

	std::vector<pcl::PointXYZ> boundaryPoints;
	boundaryPoints.push_back(min);
	boundaryPoints.push_back(max);

	return boundaryPoints;
}

void r3d::mtds::SlicBasedSegmentation::setConfiguration(TiXmlNode* param) 
{
	if (param != nullptr && param->FirstChild() != nullptr) 
	{
		m_OMPNumberOfCores = std::atoi(PGrabber::getParameter(param, "OMPNumberOfCores"));
		m_NENumberOfNeighbours = std::atoi(PGrabber::getParameter(param, "NENumberOfNeighbours"));
		m_GridSize = std::atof(PGrabber::getParameter(param, "GridSize"));
		m_SuperpointCompactness = std::atof(PGrabber::getParameter(param, "SuperpointCompactness"));
		m_NumberOfIterations = std::atoi(PGrabber::getParameter(param, "NumberOfIterations"));
	}
}


bool r3d::mtds::SlicBasedSegmentation::segmentClusters(space::ClosedSpace dataSet, pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloudO, std::vector <pcl::PointIndices>* clustersO)
{
	/*!< Loading needed data and define needed variables*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = dataSet.getXYZData().makeShared();
	pcl::PointCloud<pcl::Normal>::Ptr normals = dataSet.getNormalData().makeShared();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
	std::vector<pcl::PointXYZ> boundaryPoints;
	srand(time(nullptr));
	/*!< Calculate normals if they are not avaible*/
	if (normals->empty())
	{
		normals = BirkysNormalSegmentation::estimateNormals(cloud, m_OMPNumberOfCores, m_NENumberOfNeighbours); // TODO spravit config premenne
	}

	/*!< Get the boundary box of the dataset and generate seeds equally thru over the dataset*/
	boundaryPoints = estimateBoundaryBox(cloud);
	// TODO popremyslaj, ze podla velkosti  boundary boxu spravis pocet seedov
	pcl::PointCloud<pcl::PointXYZ>::Ptr seeds(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr seedsNormals(new pcl::PointCloud<pcl::Normal>);
	for (float i = boundaryPoints.at(0).x + m_GridSize; i < boundaryPoints.at(1).x; i += m_GridSize)
	{
		for (float j = boundaryPoints.at(0).y + m_GridSize; j < boundaryPoints.at(1).y; j += m_GridSize)
		{
			for (float k = boundaryPoints.at(0).z + m_GridSize; k < boundaryPoints.at(1).z; k += m_GridSize)
			{
				seeds->points.push_back(pcl::PointXYZ(i, j, k));
			}
		}
	}
	seedsNormals->resize(seeds->points.size());
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	std::vector<int> kNearest;
	std::vector<float> kSqrDistance;
	kdtree.setInputCloud(cloud);
	for (int i = 0; i < seedsNormals->points.size(); i++)
	{
		kdtree.nearestKSearch(seeds->points[i], 1, kNearest, kSqrDistance);
		seedsNormals->points[i] = normals->points[kNearest.at(0)];
	}

	/*!< SLIC loop*/
	std::vector<pcl::PointIndices> labeledPoints;

	for (int i = 0; i < m_NumberOfIterations - 1; i++)
	{
		labeledPoints = assignPointsToNearestSeeds(cloud, normals, seeds, seedsNormals, m_GridSize, m_SuperpointCompactness);
		calculateMeanAtributesOfClusters(cloud, normals, seeds, seedsNormals, labeledPoints);
	}
	labeledPoints = assignPointsToNearestSeeds(cloud, normals, seeds, seedsNormals, m_GridSize, m_SuperpointCompactness);

	pcl::RGB labelColor;
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *cloudRGB);
	for (int i = 0; i < labeledPoints.size(); i++)
	{
		if (labeledPoints.at(i).indices.size() != 0)
		{
			labelColor.r = rand() % 256;
			labelColor.g = rand() % 256;
			labelColor.b = rand() % 256;
			for (int j = 0; j < labeledPoints.at(i).indices.size(); j++)
			{
				cloudRGB->points[labeledPoints.at(i).indices[j]].r = labelColor.r;
				cloudRGB->points[labeledPoints.at(i).indices[j]].g = labelColor.g;
				cloudRGB->points[labeledPoints.at(i).indices[j]].b = labelColor.b;
			}
		}
	}

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg = BirkysNormalSegmentation::regionGrowingRGB(cloudRGB, 10,0,600, clustersO);

	*coloredCloudO = *reg.getColoredCloud();

	return true;
}

std::vector<pcl::PointIndices> r3d::mtds::SlicBasedSegmentation::assignPointsToNearestSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudXYZ,
	pcl::PointCloud<pcl::Normal>::Ptr &cloudNormals, pcl::PointCloud<pcl::PointXYZ>::Ptr &seedsXYZ, pcl::PointCloud<pcl::Normal>::Ptr &seedsNormals, float gridSize, float compactness)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	std::vector<int> kNearest;
	std::vector<float> kSqrDistance;
	std::vector<pcl::PointIndices>* labeledPoints = new std::vector<pcl::PointIndices>();
	labeledPoints->resize(seedsXYZ->points.size());

	kdtree.setInputCloud(seedsXYZ);
	float difXYZ, difNormals, dif, minDif = 0.0;
	int nearestIndex = 0;
	for (int i = 0; i < cloudXYZ->size(); i++)
	{
		kdtree.nearestKSearch(cloudXYZ->at(i), 27, kNearest, kSqrDistance);
		for (int j = 0; j < kNearest.size(); j++) /*!< Loop over 27 neigbour seeds and find the nearest one*/
		{
			difXYZ = kSqrDistance.at(j);
			difNormals = pow(seedsNormals->at(kNearest.at(j)).normal_x - cloudNormals->at(i).normal_x, 2) +
				pow(seedsNormals->at(kNearest.at(j)).normal_y - cloudNormals->at(i).normal_y, 2) +
				pow(seedsNormals->at(kNearest.at(j)).normal_z - cloudNormals->at(i).normal_z, 2);
			difNormals = difNormals > 2.0 ? sqrt(4.0 - difNormals) : sqrt(difNormals); /*!< handle inverse normals*/
			dif = difNormals + (compactness / gridSize) * difXYZ;
			if (j == 0 || minDif > dif)
			{
				minDif = dif;
				nearestIndex = j;
			}
		}
		labeledPoints->at(kNearest.at(nearestIndex)).indices.push_back(i);
	}

	return *labeledPoints;
}

void r3d::mtds::SlicBasedSegmentation::calculateMeanAtributesOfClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudXYZ, pcl::PointCloud<pcl::Normal>::Ptr &cloudNormals, pcl::PointCloud<pcl::PointXYZ>::Ptr seedsXYZ, pcl::PointCloud<pcl::Normal>::Ptr seedsNormals, std::vector<pcl::PointIndices> &clusters)
{
	pcl::PointXYZ meanXYZ;
	pcl::Normal meanNormal;
	for (int i = 0; i < clusters.size(); i++)
	{
		meanXYZ.x = 0.0;
		meanXYZ.y = 0.0;
		meanXYZ.z = 0.0;
		meanNormal.normal_x = 0.0;
		meanNormal.normal_y = 0.0;
		meanNormal.normal_z = 0.0;
		int clusterSize = clusters.at(i).indices.size();
		for (int j = 0; j < clusterSize; j++)
		{
			int index = clusters.at(i).indices.at(j);
			meanXYZ.x += cloudXYZ->at(index).x;
			meanXYZ.y += cloudXYZ->at(index).y;
			meanXYZ.z += cloudXYZ->at(index).z;
			meanNormal.normal_x += cloudNormals->at(index).normal_x;
			meanNormal.normal_y += cloudNormals->at(index).normal_y;
			meanNormal.normal_z += cloudNormals->at(index).normal_z;
		}
		seedsXYZ->at(i).x = meanXYZ.x / float(clusterSize);
		seedsXYZ->at(i).y = meanXYZ.y / float(clusterSize);
		seedsXYZ->at(i).z = meanXYZ.z / float(clusterSize);
		seedsNormals->at(i).normal_x = meanNormal.normal_x / float(clusterSize);
		seedsNormals->at(i).normal_y = meanNormal.normal_y / float(clusterSize);
		seedsNormals->at(i).normal_z = meanNormal.normal_z / float(clusterSize);
	}
}