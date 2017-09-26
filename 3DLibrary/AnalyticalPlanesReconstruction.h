#pragma once

#include "ClosedSpace.h"
#include "DataStatistics.h"
#include "Method.h"

namespace r3d {
	namespace mtds {
		/*! \class GrowingRegionReconstruction GrowingRegionReconstruction.h "GrowingRegionReconstruction.h"
		*  \brief Composite class aggregating reconstruction methods and simple outliers segmentation
		*  Uses Birkis segmenation by growing region and color labeling
		*  Reconstructs planar objects using RANSAC analytical representation
		*  Segments outliers of these planar objects
		*  \author Lukas Hudec
		*/
		class AnalyticalPlanesReconstruction : public Method
		{

		public:
			AnalyticalPlanesReconstruction();
			AnalyticalPlanesReconstruction(const std::string name, const char * config);
			~AnalyticalPlanesReconstruction();

			bool reconstruct(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &coloredCloudI, std::vector <pcl::PointIndices> &clustersI, std::vector<r3d::obj::Wall> &planesO);
			/*! \brief Sets the configuration from XML source
			*   \warning Does not control existence of the config value - may fall
			*/
			void setConfiguration(TiXmlNode* param) override;

			std::vector<Eigen::Vector4f> getClusterCoefficients() const
			{
				return m_clusterCoefficients;
			}

			std::vector<long double> getStandardDeviations() const
			{
				return m_standardDeviations;
			}

			const pcl::PointCloud<pcl::PointXYZ>::Ptr& getSegmentedBorder() const 
			{
				return m_segmentedBorder;
			}

			r3d::data::DataStatistics m_dataStatistics;
		private:

			void setDefaultAttributes();
			/*! \brief Extracts edge clusters from inputcloud, erases PointIndices from clusters vector and copies edge points to edge outputcloud. Note: Output cloud must be initialised */
			static void extractEdgeClusters(const pcl::PointCloud<pcl::PointXYZRGB>& inputCloud, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZRGB>& planes, pcl::PointCloud<pcl::PointXYZRGB>& edges);
			/* \brief Computes planar coefficients for given cloud segment
			*  Uses RANSAC plane model to compute coefficients
			*  coloredCloud is input cloud, clusters are indices containing segment, isUsefulWall is output vector with information wheter given segment is actual plane, wall objects
			*/
			void computePlanarCoefficients(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &coloredCloudI, std::vector <pcl::PointIndices> &clustersI, std::vector<Eigen::Vector4f> &clusterCoefficientsO, std::vector<bool> &isUsefulPlaneO, std::vector<r3d::obj::Wall> &wallsO);
			/*! \brief Filters points from concave hull lieng on the same line, so the rest is only simple representation of the hull
			*/
			void filterHullPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr hull, int closestIntex, std::vector<int>& usefullPoints);
			/*! \bief Fits given points to the nearest points from the actual wall segment for more accurate represenation
			*   fills the filteredHull pointcloud with these points
			*   \warning points are not sorted 
			*/
			void fitHullToSegmentPoints(r3d::obj::Wall& wallI, pcl::PointCloud<pcl::PointXYZ>& cornerPoints, pcl::PointCloud<pcl::PointXYZ> filteredHull) const;
			/*! \bief Fits given points to the nearest points from the actual wall segment for more accurate represenation
			*   fills hull cloud and selects vector with these points' indices
			*   \warning points are not sorted
			*/
			void fitHullToSegmentPoints(r3d::obj::Wall& wallI, pcl::PointCloud<pcl::PointXYZ>& cornerPoints, pcl::PointCloud<pcl::PointXYZ>& hull, std::vector<int> &indices) const;
			void sortSegmentHull(pcl::PointCloud<pcl::PointXYZ>& hull);
			/*! /brief Refines corner points if the segment is not bordered correctly*/
			void refineCornerPoints(std::vector<r3d::obj::Wall>& wallsIO);
			/*! \brief Main method for reconstruction of the Growing Region segments */
			void reconstructWallPlanes(const std::vector<bool> &isUsefulPlaneI, std::vector<r3d::obj::Wall> &wallsIO);
			/*!
			* \brief Contains extracted borders(edges) between planes, that weren't connected to any plane by "growing region"
			*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr m_segmentedBorder;
			/*!
			* \brief Contains extracted planes selected/segmented by "growing region"
			*/
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmentedPlanes;

			/*!
			* \brief Point Cloud of examined clustered planes outliers
			* Usefull for further segmentation wall objects like switches, paintings, 
			*/
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeOutliers;
			/*!
			* \brief May contain extracted lines from segmented dataset by "growing region"
			*/
			//std::vector<r3d::prims::Line> m_segmentedLines;

			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_planeInliers;
			//std::vector<pcl::PointCloud<pcl::PointXYZRGB>> m_outliersSegments;

			std::vector<Eigen::Vector4f> m_clusterCoefficients;
			std::vector<long double> m_standardDeviations;

			/*!< config variables*/
			int m_KDTreeSearch;
			int m_MinClusterSize;
			int m_MaxClusterSize;
			int m_NumberOfNeighbours;
			int m_DispersionIterations;
			double m_SmoothnessThreshold;
			double m_CurvatureThreshold;
			double m_DistanceThreshold;
			double m_Probability;
			double m_RanSACLine;
			int m_NumberOfThreads;
			double m_ConcaveAlpha;
			double m_orderingAlpha;
			int m_CornerRadiusMultiplicator;
			int m_PlaneBorderThreshold;
			double m_filterAngle1;
			double m_filterAngle2;
		};
	}
}

