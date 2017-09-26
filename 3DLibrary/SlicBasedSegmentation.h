#pragma once

#ifndef __SLICBASEDSEGMENTATION__
#define __SLICBASEDSEGMENTATION__
#include "Method.h"

namespace r3d {
	namespace space{
		class ClosedSpace;
	}
	/*! \namespace mtds
	* contains all reconstuction methods
	*/
	namespace mtds {
		/*! \class SlicBasedSegmentation SlicBasedSegmentation.h "SlicBasedSegmentation.h"
		*  \brief This is a derived class from class Reconstruction.
		*  Contains methods and variables for 3D reconstruction based on SLIC clustering algorithm
		*  STILL UNDER CONSTRUCTION.
		*
		*	\author Robert Birkus
		*/
		class SlicBasedSegmentation :
			public Method
		{
		public:
			SlicBasedSegmentation();
			~SlicBasedSegmentation();

			/*!
			*  \brief static function which finds the boundary points of the given point cloud
			*/
			static std::vector<pcl::PointXYZ> estimateBoundaryBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
			/*!
			*	\brief virtual function for setting up the member variables from the XML config file
			*	@param [in] param - XML parser class TiXmlNode
			*/
			virtual void setConfiguration(TiXmlNode* param) override;
			/*!
			*	\brief TODO 
			*/
			static void calculateMeanAtributesOfClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudXYZ, pcl::PointCloud<pcl::Normal>::Ptr &cloudNormals, pcl::PointCloud<pcl::PointXYZ>::Ptr seedsXYZ, pcl::PointCloud<pcl::Normal>::Ptr seedsNormals, std::vector<pcl::PointIndices> &clusters);
			/*!
			*  \brief function responsible for the whole reconstruction calculations
			*/
			bool segmentClusters(space::ClosedSpace dataSetI, pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloudO, std::vector <pcl::PointIndices>* clustersO);
			
			/*!
			* \brief function for assigning every point of a cloud to its nearest seed according to SLIC distance
			* TODO finish doc
			*/
			static std::vector<pcl::PointIndices> assignPointsToNearestSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudXYZ,
				pcl::PointCloud<pcl::Normal>::Ptr &cloudNormals, pcl::PointCloud<pcl::PointXYZ>::Ptr &seedsXYZ, pcl::PointCloud<pcl::Normal>::Ptr &seedsNormals,
				float gridSize, float compactness);
		private:
			/*!
			*	\brief Output XYZRGB point cloud needed for test visualisation
			*/
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_segmented_cloud;

			/*! Configuration variables*/
			unsigned short m_OMPNumberOfCores;
			unsigned short m_NENumberOfNeighbours;
			unsigned short m_NumberOfIterations;
			float m_SuperpointCompactness;
			float m_GridSize;

		};
	}
}

#endif __SLICBASEDSEGMENTATION__
