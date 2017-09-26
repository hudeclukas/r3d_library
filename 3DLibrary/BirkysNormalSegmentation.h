#pragma once

#ifndef __BIRKYSMETHODRECONSTRUCTION__
#define __BIRKYSMETHODRECONSTRUCTION__

#include "Method.h"
#include "ClosedSpace.h"
#include <pcl/segmentation/region_growing_rgb.h>

namespace r3d
{
	/*! \namespace mtds
	* contains all reconstuction methods
	*/
	namespace mtds
	{
		/*! \class BirkysMethodReconstruction BirkysMethodReconstruction.h "BirkysMethodReconstruction.h"
		*  \brief This is a derived class from class Reconstruction.
		*  Contains methods and variables for 3D reconstruction using normal estimation, normal histogram,
		*  point labeling/coloring, region growing.
		*
		*	\author Robert Birkus
		*/
		class BirkysNormalSegmentation :
			public Method
		{
		public:
			BirkysNormalSegmentation();
			~BirkysNormalSegmentation();

			void setData(pcl::PointCloud<pcl::PointXYZ> data) 
			{
				m_dataSet.setData(data);
			}
			void setData(pcl::PointCloud<pcl::RGB> data) 
			{
				m_dataSet.setData(data);
			}
			void setData(pcl::PointCloud<pcl::Normal> data) 
			{
				m_dataSet.setData(data);
			}

			unsigned short getM_OmpNumberOfCores() const
			{
				return m_OMPNumberOfCores;
			}

			void setM_OmpNumberOfCores(unsigned short m_omp_number_of_cores)
			{
				m_OMPNumberOfCores = m_omp_number_of_cores;
			}

			unsigned short getM_NeNumberOfNeighbours() const
			{
				return m_NENumberOfNeighbours;
			}

			void setM_NeNumberOfNeighbours(unsigned short m_ne_number_of_neighbours)
			{
				m_NENumberOfNeighbours = m_ne_number_of_neighbours;
			}

			unsigned short getM_NumberOfDominantRegions() const
			{
				return m_NumberOfDominantRegions;
			}

			void setM_NumberOfDominantRegions(unsigned short m_number_of_dominant_regions)
			{
				m_NumberOfDominantRegions = m_number_of_dominant_regions;
			}

			unsigned short getM_HistogramDeletionArea() const
			{
				return m_HistogramDeletionArea;
			}

			void setM_HistogramDeletionArea(unsigned short m_histogram_deletion_area)
			{
				m_HistogramDeletionArea = m_histogram_deletion_area;
			}

			unsigned short getM_GrNumberOfNeighbours() const
			{
				return m_GRNumberOfNeighbours;
			}

			void setM_GrNumberOfNeighbours(unsigned short m_gr_number_of_neighbours)
			{
				m_GRNumberOfNeighbours = m_gr_number_of_neighbours;
			}

			unsigned short getM_PointColorThreshold() const
			{
				return m_PointColorThreshold;
			}

			void setM_PointColorThreshold(unsigned short m_point_color_threshold)
			{
				m_PointColorThreshold = m_point_color_threshold;
			}

			unsigned short getM_MinClusterSize() const
			{
				return m_MinClusterSize;
			}

			void setM_MinClusterSize(unsigned short m_min_cluster_size)
			{
				m_MinClusterSize = m_min_cluster_size;
			}

			unsigned short getM_NumberOfPlains() const
			{
				return m_NumberOfPlains;
			}

			void setM_NumberOfPlains(unsigned short m_number_of_plains)
			{
				m_NumberOfPlains = m_number_of_plains;
			}

			unsigned short getM_PointsPerHalfSphere() const
			{
				return m_PointsPerHalfSphere;
			}

			void setM_PointsPerHalfSphere(unsigned short m_points_per_half_sphere)
			{
				m_PointsPerHalfSphere = m_points_per_half_sphere;
			}

			float getM_DistanceThreshold() const
			{
				return m_DistanceThreshold;
			}

			void setM_DistanceThreshold(float m_distance_threshold)
			{
				m_DistanceThreshold = m_distance_threshold;
			}

			float getM_DominantNormalsSearchRadius() const
			{
				return m_DominantNormalsSearchRadius;
			}

			void setM_DominantNormalsSearchRadius(float m_dominant_normals_search_radius)
			{
				m_DominantNormalsSearchRadius = m_dominant_normals_search_radius;
			}

			/*!
			*	\brief virtual function for setting up the member variables from the XML config file
			*	@param [in] param - XML parser class TiXmlNode
			*/
			void setConfiguration(TiXmlNode* param) override;

			/*!
			*	\brief function for normal estimation of a point cloud
			*	@param [in] cloudXYZ - XYZ point cloud pcl::PointCloud<pcl::PointXYZ>::Ptr
			*	@param [in] numberOfCores - number of cores for the parallel computation
			*	@param [in] numberOfNeighbours - number of neighbours considered during normal estimation computation
			*	@param [out] normals of the input point cloud pcl::PointCloud<pcl::Normal>::Ptr
			*/
			static pcl::PointCloud<pcl::Normal>::Ptr r3d::mtds::BirkysNormalSegmentation::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ, unsigned short numberOfCores, unsigned short numberOfNeighbours);

			/*!
			*	\brief function which creates a histogram of normals from the input normals
			*	@param [in] normals - input normals of point cloud
			*	@param [out] 3D histogram of normals
			*/
			static pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::createHistogramOfNormals(pcl::PointCloud<pcl::Normal>::Ptr normals);

			/*!
			*	\brief function which generates equidistributed points on the surface of a half sphere
			*	The output half sphere is designed for searching dominant normals
			*	@param [in] pointsPerHalfSphere - number of points on the half sphere
			*	@param [out] point cloud of the half sphere
			*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::createHalfSphere(unsigned short pointsPerHalfSphere);

			/*!
			*	\brief function which finds a given count of dominant normals in the input histogram of normals
			*	@param [in] normalHistogram - 3D histogram of normals
			*	@param [in] numberOfDominantNormals - the number of dominant normals to find
			*	@param [in] dominantNormalsSearchRadius - the search radius for neighbour searching
			*	@param [in] halfSphere - each point of the halfSphere is a potencial dominant normal
			*	@param [out] point cloud of dominant normals
			*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr r3d::mtds::BirkysNormalSegmentation::findDominantNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr normalHistogram, unsigned short numberOfDominantNormals, float dominantNormalsSearchRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr halfSphere);

			/*!
			*	\brief function which label each point of the cloud according to the "nearest" dominant normal
			*	@param [in] cloud - the input point cloud
			*	@param [in] normals - normals of each point in the input point cloud
			*	@param [in] dominantNormals - dominant normals of the input point cloud
			*	@param [out] colored point cloud
			*/
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr r3d::mtds::BirkysNormalSegmentation::labelPointsAccordingToDominantNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr dominantNormals);

			/*!
			*	\brief function which apply region growing algorithm to the colored input point cloud, region growing algorithm segments based on color
			*	@param [in] cloudRGB - colored point cloud
			*	@param [in] numberOfNeighbours - the number of searched neighbours (input paramater for region growing)
			*	@param [in] pointColorThreshold - color threshold (input parameter for region growing)
			*	@param [in] minClusterSize - minimal cluster size (input parameter for region growing), smaller clusters are merged to neighbouring clusters
			*	@param [out] clusters - segmented clusters of the point cloud
			*	@param [out] instance of pcl::RegionGrowingRGB class containing the segmented (colored) point cloud
			*/
			static pcl::RegionGrowingRGB<pcl::PointXYZRGB> r3d::mtds::BirkysNormalSegmentation::regionGrowingRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB, unsigned short numberOfNeighbours, unsigned short pointColorThreshold, unsigned short minClusterSize, std::vector <pcl::PointIndices>* clusters);

			/*!
			*	\brief function which runs the whole segmentation method in sequence
			*	@param [out] clusters - segmented clusters of the point cloud
			*	@param [out] segmented (colored) point cloud
			*/
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr r3d::mtds::BirkysNormalSegmentation::segmentClusters(std::vector <pcl::PointIndices>* clusters);

			pcl::PointCloud<pcl::PointXYZ>::Ptr getDominantNormalsDirections();

		private:
			space::ClosedSpace m_dataSet;
			/*!
			*	\brief Output XYZRGB point cloud needed for test visualisation
			*/
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmented_cloud;
			/*!
			*	\brief Coefficients from RANSAC algorithm needed for plain visualisation
			*/
			Eigen::VectorXf* m_coeffs;

			/*!
			*	\brief Set of dominant normals of the dataset, needed for visiulisation
			*/
			pcl::PointCloud<pcl::PointXYZ>::Ptr m_dominantNormals;

			/*! Configuration variables*/
			unsigned short m_OMPNumberOfCores;
			unsigned short m_NENumberOfNeighbours;
			unsigned short m_NumberOfDominantRegions;
			unsigned short m_HistogramDeletionArea;
			unsigned short m_GRNumberOfNeighbours;
			unsigned short m_PointColorThreshold;
			unsigned short m_MinClusterSize;
			unsigned short m_NumberOfPlains;
			unsigned short m_PointsPerHalfSphere;
			float m_DistanceThreshold;
			float m_DominantNormalsSearchRadius;
		};
	}
}

#endif __BIRKYSMETHODRECONSTRUCTION__