#pragma once

#include "PlaneSegment.h"
#include "CommonUtil.h"
#include "PlanePointsExamination.h"
#include "Object.h"
#include "Method.h"

class TiXmlNode;

namespace r3d {
	namespace mtds {

		class PlanePointsExamination : public Method
		{
		public:
			PlanePointsExamination();
			PlanePointsExamination(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<pcl::PointIndices>& clusters, std::vector<Eigen::Vector4f>& coefficients, std::vector<long double> standardDeviation);

			~PlanePointsExamination();

			/*!
			* \brief Computes coefficients and separates inliers and outliers for every plane segment in dataset
			*/
			void computePlanePoints();

			std::vector<r3d::obj::Object*> findPotentialObjectsFromOutliers();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlaneInliers() const
			{
				return m_planeInliers;
			}

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlaneOutliers() const
			{
				return m_planeOutliers;
			}

			std::vector<Eigen::Vector4f> getClusterPlanesCoefficients() const
			{
				return m_segmentedClustersPlaneCoefficients;
			}

			std::vector<r3d::prims::PlaneSegment> getSegmentedClustersPlanes() const
			{
				return m_segmentedClustersPlanes;
			}

			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> getSegmentedOutliers() const
			{
				return m_segmentedOutliers;
			}

			void setConfiguration(TiXmlNode* param) override;

			std::vector<HASH> histogram;

		private:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeInliers;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeOutliers;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloudData;
			std::vector<pcl::PointIndices> m_segmentedCloudClusters;
			std::vector<long double> m_stdDeviation;
			std::vector<Eigen::Vector4f> m_segmentedClustersPlaneCoefficients;
			std::vector<r3d::prims::PlaneSegment> m_segmentedClustersPlanes;
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> m_segmentedOutliers;
			
			/* config attributes */
			double m_InliersDistanceParameter;
			double m_OutliersSearchRadiusParameter;
			double m_FilterMeanK;
			double m_FilterStDevMulTresh;
			int m_GRNumberOfNeighbours;
			double m_GRColorThreshold;
			int m_GRMinClusterSize;

		};
	}
}
