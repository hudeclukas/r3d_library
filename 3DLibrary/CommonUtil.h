#pragma once
#include <boost/thread/thread.hpp>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>


namespace r3d {
	struct HASH
	{
		HASH()
			: distance(0),
			  points(0)
		{
		}

		HASH(double distance, int points)
			: distance(distance),
			  points(points)
		{
		}

		bool operator==(const HASH &b) const
		{
			return this->points == b.points && this->distance == b.distance;
		}

		double distance;
		int points;
	};

	const long double PI = 4 * atan(1);

	class CommonUtil
	{
	public:
		CommonUtil();
		~CommonUtil();
		/*!
		* \brief computes histogram out of all distances in vector
		* use "cut" to group points in distance "cuts" /default is 0, but consider using small values as 0.0001 or 0.001
		*/
		static std::vector<r3d::HASH> computeDistancesHistogram(std::vector<double> distances, double cut = 0);

		static bool comparePointIndicesDesc(pcl::PointIndices& a, pcl::PointIndices& b);

		static bool comparePointIndicesAsc(pcl::PointIndices& a, pcl::PointIndices& b);

		static bool verifyPlaneCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &planeCloud, pcl::PointCloud<pcl::PointXYZ> &cornerPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr &planeConcaveHull, double tolerance = 2.0, double alpha = 0.05);

		/* \brief returns 3D angle between 3 points -> 2vectors AB and AC
		*/
		static double getAngle3D(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C);
		static void orderPointsInConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZ> &outputCloud, double alphaIn = 10.0);

		template<class PointT>
		static void planeCoefficients(pcl::PointCloud<PointT> cloud, pcl::PointIndices indices, Eigen::Vector4f& coefficients, double probability = 0.8, double threshold = 0.01);
	};

	template <class PointT>
	void CommonUtil::planeCoefficients(pcl::PointCloud<PointT> cloud, pcl::PointIndices indices, Eigen::Vector4f &coefficients, double probability, double threshold)
	{
		Eigen::VectorXf coef;
		pcl::SampleConsensusModelPlane<PointT> modelPlane(cloud.makeShared(), indices.indices);
		pcl::RandomSampleConsensus<PointT> ransac(boost::make_shared<pcl::SampleConsensusModelPlane<PointT>>(modelPlane));
		ransac.setDistanceThreshold(threshold);
		ransac.setProbability(probability);
		ransac.computeModel();
		ransac.getModelCoefficients(coef);

		coefficients = { coef[0], coef[1], coef[2], coef[3] };
	}
}
