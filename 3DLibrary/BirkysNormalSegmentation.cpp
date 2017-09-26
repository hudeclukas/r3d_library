#include "stdafx.h"
#include "ParameterGrabber.h"
#include "BirkysNormalSegmentation.h"


r3d::mtds::BirkysNormalSegmentation::BirkysNormalSegmentation()
	: Method("Birkys Segmentation", "BirkysNormalSegmentation"),
m_OMPNumberOfCores(4)
, m_NENumberOfNeighbours(100)
, m_NumberOfDominantRegions(3)
, m_HistogramDeletionArea(30)
, m_GRNumberOfNeighbours(10)
, m_PointColorThreshold(0)
, m_MinClusterSize(600)
, m_NumberOfPlains(6)
, m_PointsPerHalfSphere(5000)
, m_DistanceThreshold(0.01)
, m_DominantNormalsSearchRadius(0.25)
{
	m_coeffs = nullptr;
}


r3d::mtds::BirkysNormalSegmentation::~BirkysNormalSegmentation()
{
}

void r3d::mtds::BirkysNormalSegmentation::setConfiguration(TiXmlNode* param)
{
	if (param != nullptr && param->FirstChild() != nullptr)
	{
		m_OMPNumberOfCores = std::atoi(PGrabber::getParameter(param, "OMPNumberOfCores"));
		m_NENumberOfNeighbours = std::atoi(PGrabber::getParameter(param, "NENumberOfNeighbours"));
		m_NumberOfDominantRegions = std::atoi(PGrabber::getParameter(param, "NumberOfDominantRegions"));
		m_HistogramDeletionArea = std::atoi(PGrabber::getParameter(param, "HistogramDeletionArea"));
		m_GRNumberOfNeighbours = std::atoi(PGrabber::getParameter(param, "GRNumberOfNeighbours"));
		m_PointColorThreshold = std::atoi(PGrabber::getParameter(param, "PointColorThreshold"));
		m_DistanceThreshold = std::atoi(PGrabber::getParameter(param, "MinClusterSize"));
		m_NumberOfPlains = std::atoi(PGrabber::getParameter(param, "NumberOfPlains"));
		m_DistanceThreshold = std::atof(PGrabber::getParameter(param, "DistanceThreshold"));
		m_PointsPerHalfSphere = std::atoi(PGrabber::getParameter(param, "PointsPerHalfSphere"));
		m_DominantNormalsSearchRadius = std::atof(PGrabber::getParameter(param, "DominantNormalsSearchRadius"));
	}
}

pcl::PointCloud<pcl::Normal>::Ptr r3d::mtds::BirkysNormalSegmentation::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ, unsigned short numberOfCores, unsigned short numberOfNeighbours)
{
	/*!< Normal estimation */
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(numberOfCores);
	ne.setInputCloud(cloudXYZ);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	ne.setKSearch(numberOfNeighbours);
	ne.compute(*normals);
	return normals;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::createHistogramOfNormals(pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	/*!< Create histogram of normals */
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal3DHistogram(new pcl::PointCloud<pcl::PointXYZ>);
	normal3DHistogram->resize(normals->points.size());
	for (int i = 0; i < normals->points.size(); i++)
	{
		/*! We are taking into account only half of the sphere, the opposite directions are the same for us*/
		if (normals->points[i].normal_x < 0)
		{
			normals->points[i].normal_x *= -1;
			normals->points[i].normal_y *= -1;
			normals->points[i].normal_z *= -1;
		}

		normal3DHistogram->points[i].x = normals->points[i].normal_x;
		normal3DHistogram->points[i].y = normals->points[i].normal_y;
		normal3DHistogram->points[i].z = normals->points[i].normal_z;
	}

	return normal3DHistogram;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::createHalfSphere(unsigned short pointsPerHalfSphere)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr halfSphere(new pcl::PointCloud<pcl::PointXYZ>());
	auto pointsPerSphere = pointsPerHalfSphere * 2; /*!< setting this to pointsPerHalfSphere give us a whole sphere*/
	const float radius = 1.0; /*!< the radius of our sphere, we need always a half sphere with radius 1.0 for our method*/

	halfSphere->resize(pointsPerHalfSphere);
	const auto rnd = 1;
	float offset = 2.0 / static_cast<float>(pointsPerSphere);
	float increment = 3.1415926 * (3.0 - sqrt(5.0));

	for (auto i = 0; i < pointsPerHalfSphere; i++)
	{
		float x = ((i * offset) - 1) + (offset / 2.0);
		float r = sqrt(1 - pow(x, 2));

		float phi = ((i + rnd) % pointsPerSphere) * increment;
		float y = cos(phi) * r;
		float z = sin(phi) * r;

		halfSphere->points[i].x = x * radius * -1;
		halfSphere->points[i].y = y * radius * -1;
		halfSphere->points[i].z = z * radius * -1;
	}

	return halfSphere;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::findDominantNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr normalHistogram, unsigned short numberOfDominantNormals, float dominantNormalsSearchRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr halfSphere)
{
	pcl::PointXYZ searchPoint;
	pcl::PointXYZ invSearchPoint;
	pcl::PointXYZ dominantPoint;
	std::vector<int> pointIdxRadiusSearch; /*!< Neighbours indeces of the searched point*/
	std::vector<int> invPointIdxRadiusSearch;  /*!< Neighbours indeces of the inverse searched point*/
	std::vector<int> dominantPointIdxRadiusSearch; /*!< Neighbours indeces of the dominant normal*/
	std::vector<float> pointRadiusSquaredDistance; /*!< not used - needed because it is neccessery parameter for a used function*/
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int max;
	int numberOfNeigbours;
	pcl::PointCloud<pcl::PointXYZ>::Ptr dominantNormals(new pcl::PointCloud<pcl::PointXYZ>());
	dominantNormals->resize(numberOfDominantNormals);

	/*!< find the most dominant normal*/
	for (size_t j = 0; j < numberOfDominantNormals; j++)
	{
		kdtree.setInputCloud(normalHistogram);
		max = 0;
		for (size_t i = 0; i < halfSphere->points.size(); i++)
		{
			searchPoint = halfSphere->points[i];
			numberOfNeigbours = kdtree.radiusSearch(searchPoint, dominantNormalsSearchRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

			/*!< search the neigbours of the inverse normal too - overflow*/
			if (searchPoint.x <= dominantNormalsSearchRadius) /*!< x coordinate should be always positive*/
			{
				/*!< set the inverse searchPoint*/
				invSearchPoint.x = searchPoint.x * -1;
				invSearchPoint.y = searchPoint.y * -1;
				invSearchPoint.z = searchPoint.z * -1;
				numberOfNeigbours += kdtree.radiusSearch(invSearchPoint, dominantNormalsSearchRadius, invPointIdxRadiusSearch, pointRadiusSquaredDistance);
				/*!< append the neighbours of the inverse normal to the neighbours of the non-inverse normal*/
				for (size_t k = 0; k < invPointIdxRadiusSearch.size(); k++)
				{
					pointIdxRadiusSearch.push_back(invPointIdxRadiusSearch.at(k));
				}
			}

			/*!< store the currently most dominant normal data*/
			if (max < numberOfNeigbours)
			{
				max = numberOfNeigbours;
				dominantPoint = searchPoint;
				dominantPointIdxRadiusSearch = pointIdxRadiusSearch;
			}
		}

		/*!< store the dominant normal*/
		dominantNormals->points[j].x = dominantPoint.x;
		dominantNormals->points[j].y = dominantPoint.y;
		dominantNormals->points[j].z = dominantPoint.z;

		/*!< Delete all points in the neighbourhood of the dominant normal*/
		pcl::IndicesPtr indices(new std::vector <int>(dominantPointIdxRadiusSearch));
		pcl::ExtractIndices<pcl::PointXYZ> eifilter(false);
		eifilter.setInputCloud(normalHistogram);
		eifilter.setIndices(indices);
		eifilter.setNegative(true); /*!< True - extract remained indeces instead of removed ones*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliners(new pcl::PointCloud<pcl::PointXYZ>);
		eifilter.filter(*inliners);
		normalHistogram = inliners;
	}
	return dominantNormals;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr r3d::mtds::BirkysNormalSegmentation::labelPointsAccordingToDominantNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr dominantNormals)
{
	/*!< Generating color map*/
	pcl::PointCloud<pcl::RGB>::Ptr colorMap(new pcl::PointCloud<pcl::RGB>);
	colorMap->resize(dominantNormals->points.size());
	srand(time(nullptr));
	for (size_t i = 0; i < dominantNormals->points.size(); i++)
	{
		colorMap->points[i].r = rand() % 256;
		colorMap->points[i].g = rand() % 256;
		colorMap->points[i].b = (i + 1) * 255 / m_NumberOfDominantRegions;
	}

	/*!< Labeling the points of the cloud according to the found dominant normals */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
	cloudRGB->points.resize(cloud->points.size());
	float minDiff, diff;
	int label;
	for (int i = 0; i < normals->points.size(); i++)
	{

		label = 0;
		minDiff = pow(normals->points[i].normal_x - dominantNormals->points[0].x, 2)
			+ pow(normals->points[i].normal_y - dominantNormals->points[0].y, 2)
			+ pow(normals->points[i].normal_z - dominantNormals->points[0].z, 2);
		/*!< handle inverse normals*/
		minDiff = minDiff > 2.0 ? sqrt(4.0 - minDiff) : sqrt(minDiff); /*!< the sqrt is not neccessery */

		for (int j = 1; j < dominantNormals->points.size(); j++)
		{
			diff = pow(normals->points[i].normal_x - dominantNormals->points[j].x, 2)
				+ pow(normals->points[i].normal_y - dominantNormals->points[j].y, 2)
				+ pow(normals->points[i].normal_z - dominantNormals->points[j].z, 2);
			/*!< handle inverse normals*/
			diff = diff > 2.0 ? sqrt(4.0 - diff) : sqrt(diff);	/*!< the sqrt is not neccessery */
			if (diff < minDiff)
			{
				minDiff = diff;
				label = j;
			}
		}

		cloudRGB->points[i].x = cloud->points[i].x;
		cloudRGB->points[i].y = cloud->points[i].y;
		cloudRGB->points[i].z = cloud->points[i].z;

		cloudRGB->points[i].r = colorMap->points[label].r;
		cloudRGB->points[i].g = colorMap->points[label].g;
		cloudRGB->points[i].b = colorMap->points[label].b;
	}

	return cloudRGB;
}

pcl::RegionGrowingRGB<pcl::PointXYZRGB> r3d::mtds::BirkysNormalSegmentation::regionGrowingRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB, unsigned short numberOfNeighbours, unsigned short pointColorThreshold, unsigned short minClusterSize, std::vector <pcl::PointIndices>* clusters)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(cloudRGB);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(numberOfNeighbours);
	reg.setPointColorThreshold(pointColorThreshold);
	reg.setMinClusterSize(minClusterSize);
	
	reg.extract(*clusters);

	return reg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr r3d::mtds::BirkysNormalSegmentation::segmentClusters(std::vector <pcl::PointIndices>* clusters)
{
	/*!< Loading needed data and define needed variables*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_dataSet.getXYZData().makeShared();
	pcl::PointCloud<pcl::Normal>::Ptr normals = m_dataSet.getNormalData().makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalHistogram;
	pcl::PointCloud<pcl::PointXYZ>::Ptr halfSphere;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

	/*!< Calculate normals if they are not avaible*/
	if (normals->empty())
	{
		normals = estimateNormals(cloud, m_OMPNumberOfCores, m_NENumberOfNeighbours);
	}

	/*!< Create "histogram" of normals from the calculated normals*/
	normalHistogram = createHistogramOfNormals(normals);

	/*!< Create a half sphere of potencial dominant directions*/
	halfSphere = createHalfSphere(m_PointsPerHalfSphere);

	/*!< searching for dominant normals*/
	m_dominantNormals = findDominantNormals(normalHistogram, m_NumberOfDominantRegions, m_DominantNormalsSearchRadius, halfSphere);

	/*!< Labeling the points of the cloud according to the found dominant normals*/
	cloudRGB = labelPointsAccordingToDominantNormals(cloud, normals, m_dominantNormals);

	/*!< Region growing */
	reg = regionGrowingRGB(cloudRGB, m_GRNumberOfNeighbours, m_PointColorThreshold, m_MinClusterSize, clusters);

	return reg.getColoredCloud();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::getDominantNormalsDirections()
{
	return m_dominantNormals;
}
