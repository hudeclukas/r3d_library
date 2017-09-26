#include "stdafx.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include <random>

#include "tinyxml.h"
#include "CommonUtil.h"
#include "DataStatistics.h"


r3d::data::DataStatistics::DataStatistics()
	: m_cloud(nullptr), 
	m_distanceThreshold(0.01),
	m_sigma(0.1),
	m_nrThreads(4),
	m_lineCheckoutStep(50),
	m_linearRepetitions(10),
	m_AngularCoefficientMax(3.0),
	m_planeIdentificationThreshold(0.5)
{
}

r3d::data::DataStatistics::~DataStatistics()
{
}

void r3d::data::DataStatistics::configure(TiXmlElement& config)
{
	setDistanceThreshold(atof(config.FirstChildElement("DistanceThreshold")->GetText()));
	setSigma(atof(config.FirstChildElement("Sigma")->GetText()));
	setNrThreads(atof(config.FirstChildElement("NumberOfThreads")->GetText()));
	setAngularCoefficientMax(atof(config.FirstChildElement("AngularCoefficientMax")->GetText()));
	setLineCheckoutStep(atof(config.FirstChildElement("LineCheckoutStep")->GetText()));
	setLinearRepetitions(atof(config.FirstChildElement("LinearRepetitions")->GetText()));
	setPlaneIdentificationThreshold(atof(config.FirstChildElement("PlaneIdentificationThreshold")->GetText()));
}


long double r3d::data::DataStatistics::computeRoughness()
{
	std::ofstream log;
	log.open("DataStatistics.log",std::ios_base::app);
	time_t time = std::time(nullptr);
	std::string dt = ctime(&time);
	log << "Data statistics started <at> " << dt << std::endl;

	std::vector<double> distances;

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelPlane((new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(m_cloud)));
	if (m_planeCoefs.size() != 4) //coef.isZero(0)
	{
		/*!< compute plane coefficients */
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelPlane);
		ransac.setDistanceThreshold(m_distanceThreshold);
		ransac.computeModel();
		//ransac.refineModel(m_sigma);
		ransac.getModelCoefficients(m_planeCoefs);

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Model coefficients computed: " << m_planeCoefs[0] << " " << m_planeCoefs[1] << " " << m_planeCoefs[2] << " " << m_planeCoefs[3] << " <at> " << dt << std::endl;
	}
	modelPlane->getDistancesToModel(m_planeCoefs, distances);
	

	std::vector<HASH> histogram = CommonUtil::computeDistancesHistogram(distances,0.0001);

	time = std::time(nullptr);
	dt = ctime(&time);
	long long endIteration = histogram.size() < 500 ? histogram.size() : 500;
	log << "Histogram created <at> " << dt << "\nfirst " << endIteration << "values\nDistance\tPoints " << std::endl;

#ifdef _DEBUG
	for (auto i = 0; i < endIteration; ++i)
	{
		log << histogram[i].distance << "\t\t" << histogram[i].points << std::endl;
	}
#endif

	/*! \brief find converging value to set borders for the plane points
	 *	gaussianBorder = index of the last added points block	
	 */
	size_t gaussianBorder = 0;
	size_t sum = 0;
	int repetitions = 0;
	double angularCoefficient = 0.0;

	sum += histogram[0].points;
	auto step = m_lineCheckoutStep;
	if (m_lineCheckoutStep > histogram.size())
	{
		step = histogram.size() / 10 - 0.5;
	}
	endIteration = histogram.size() - step;
	for (auto i = 1; i < endIteration; ++i)
	{
		sum += histogram[i].points;
		gaussianBorder = i;
		double Ax = histogram[i].distance, Bx = histogram[i + step % histogram.size()].distance;
		double Ay = histogram[i].points, By = histogram[i + step % histogram.size()].points;
		angularCoefficient = abs((By - Ay) / (Bx - Ax));
		if (angularCoefficient <= m_AngularCoefficientMax * PI / 180)
		{
			log << "A[" << Ax << "," << Ay << "], B[" << Bx << "," << By << "]" << std::endl;
			++repetitions;
			if (repetitions == m_linearRepetitions)
			{
				break;
			}
		} 
		else if (repetitions > 0)
		{
			log << "reset\n";
			repetitions = 0;
		}
	}
	long double dispersion = 0;
	for (auto i = 0; i <= gaussianBorder; ++i)
	{
		/*!< mean == 0, and square makes it positive, so no division by 2 is needed */
		dispersion += histogram[i].points * pow((histogram[i].distance), 2) / sum;
	}
	m_standardDeviation = sqrt(dispersion);
	m_maxPointPlaneDistance = histogram[gaussianBorder].distance;
	m_computed = true;
	
	time = std::time(nullptr);
	dt = ctime(&time);
	log << "standard deviation: " << m_standardDeviation
		<< " gaussian border / max distance: " << m_maxPointPlaneDistance << " with step: " << step
		<< "\nangle " << angularCoefficient << " vs max " << m_AngularCoefficientMax * PI / 180
		<< " <at> " << dt << std::endl;

	log.close();
	return m_standardDeviation;
}

double r3d::data::DataStatistics::segmentPointDispersion(pcl::PointCloud<pcl::PointXYZ>::Ptr segment, int iterations)
{
	double dispersion = -1;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	kdTree.setInputCloud(segment);
	size_t K;
	std::vector<double> meanDst;

	// start the randomizer for indices
	std::random_device rd;
	std::mt19937_64 rng(rd());

	if (iterations < 0)
	{
		iterations = 10;
	}
	if (segment->size() >= 25)
	{
		// start the randomizer for K nearest
		std::random_device rdK;
		std::mt19937_64 rngK(rdK());
		std::uniform_int_distribution<size_t> uniK(0, 20);

		for (auto i = 0; i < iterations; ++i)
		{
			// select random point from segment
			std::uniform_int_distribution<size_t> uni(0, segment->size() - 1);
			auto randomPointIndex = uni(rng);
			// select K
			K = uniK(rngK)+5;

			pcl::PointXYZ searchPoint = segment->points[randomPointIndex];
			std::vector<int> pointIndices(K);
			std::vector<float> pointDistances(K);
			kdTree.nearestKSearch(searchPoint, K, pointIndices, pointDistances);

			double dstSum = 0;
			for (auto j = 1; j < pointDistances.size(); ++j)
			{
				dstSum += pointDistances[j];
			}
			meanDst.push_back(dstSum / pointDistances.size());
		}

		double meanSum = 0;
		for (auto i = 0; i < iterations; ++i)
		{
			meanSum += meanDst[i];
		}
		dispersion = meanSum / iterations;
	}
	else
	{
		// select random point from segment
		K = segment->size();
		std::uniform_int_distribution<int> uni(0, K);
		auto randomPointIndex = uni(rng);
		pcl::PointXYZ searchPoint = segment->points[randomPointIndex];
		std::vector<int> pointIndices(K);
		std::vector<float> pointDistances(K);
		kdTree.nearestKSearch(searchPoint, K, pointIndices, pointDistances);

		double dstSum = 0;
		for (auto j = 1; j < pointDistances.size(); ++j)
		{
			dstSum += pointDistances[j];
		}
		dispersion = dstSum / pointDistances.size();
	}

	// change squared distance to Euclidean distance
	dispersion = std::sqrt(dispersion);
	// value correctness
	dispersion *= 1.03;
	return dispersion;
}

void r3d::data::DataStatistics::setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
	m_cloud = pointCloud;
	m_computed = false;
}

void r3d::data::DataStatistics::setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointIndices indices)
{
	if (m_cloud == nullptr)
	{
		m_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	}
	pcl::copyPointCloud(*pointCloud, *m_cloud);
	m_computed = false;
}

void r3d::data::DataStatistics::setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, pcl::PointIndices indices)
{
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(pointCloud);
	extract.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	// no need to set extractor negative
	// extract.setNegative(false); 
	pcl::PointCloud<pcl::PointXYZRGB> tmpCloud;
	extract.filter(*m_cloud);
	m_computed = false;
}

bool r3d::data::DataStatistics::isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr segment, Eigen::VectorXf coefficients, long double stDev)
{
	pcl::PointIndices::Ptr inlierIndeces = boost::make_shared<pcl::PointIndices>();
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model((new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(segment)));
	/*!< find inliers statisticaly according to the distance from plane */
	model->selectWithinDistance(coefficients, stDev, inlierIndeces->indices);

	/*!< extract inliers */
	pcl::PointCloud<pcl::PointXYZ> inlierPoints;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(segment);
	extract.setIndices(inlierIndeces);
	extract.setNegative(false);
	extract.filter(inlierPoints);

	/*!< extract outliers */
	pcl::PointCloud<pcl::PointXYZ> outlierPoints;
	extract.setNegative(true);
	extract.filter(outlierPoints);

	/*!< get ratio of inliers and outliers */
	float ratio = outlierPoints.points.size() / inlierPoints.points.size();
	double threshold = getPlaneIdentificationThreshold();

	if (ratio < threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}
