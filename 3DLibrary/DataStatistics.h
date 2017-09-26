#include "tinyxml.h"
#pragma once

namespace r3d {
	namespace data {
		/*! \class DataStatistics DataStatistics.h "DataStatistics.h"
		*  \brief Statistics about the segmented pointclouds
		*  \author Lukas Hudec
		*/
		class DataStatistics
		{
		public:
			DataStatistics();
			~DataStatistics();

			/*! \brief Find whether the point cloud segment is representing a plane object according to RANSAC coefficients and standard deviation of the point cloud segment*/
			bool isPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr segment, Eigen::VectorXf coefficients, long double stDev);

			/*! \brief Sets the configuration from XML source
						*   \warning Does not control existence of the config value - may fall
						*/
			void configure(TiXmlElement& config);
			
			/*! \brief Computes roughnes of given cloud represented by standard deviation*/
			long double computeRoughness();

			/*! \brief Computes average mutual distance between points in given cloud represented*/
			static double segmentPointDispersion(pcl::PointCloud<pcl::PointXYZ>::Ptr segment, int iterations);

			void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

			void setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, pcl::PointIndices indices);
			void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointIndices indices);

			pcl::PointCloud<pcl::PointXYZ> getCloud() const
			{
				return *m_cloud;
			}

			void setDistanceThreshold(double distanceThreshold)
			{
				m_distanceThreshold = distanceThreshold;
			}

			void setSigma(double sigma)
			{
				m_sigma = sigma;
			}

			void setNrThreads(int nrThreads)
			{
				m_nrThreads = nrThreads;
			}

			void setLineCheckoutStep(int lineCheckoutStep)
			{
				m_lineCheckoutStep = lineCheckoutStep;
			}

			void setLinearRepetitions(int linearRepetitions)
			{
				m_linearRepetitions = linearRepetitions;
			}

			void setPlaneIdentificationThreshold(double planeIdentificationThreshold)
			{
				m_planeIdentificationThreshold = planeIdentificationThreshold;
			}

			void setAngularCoefficientMax(double angularCoefficientMax)
			{
				m_AngularCoefficientMax = angularCoefficientMax;
			}

			double getPlaneIdentificationThreshold() const
			{
				return	m_planeIdentificationThreshold;
			}


			long double getStandardDeviation() const
			{
				return m_standardDeviation;
			}

			long double getMaxPointPlaneDistance() const
			{
				return m_maxPointPlaneDistance;
			}

			bool isComputed() const
			{
				return m_computed;
			}

			void setPlaneCoefs(Eigen::VectorXf coefs)
			{
				m_planeCoefs = coefs;
			}

			/*! \brief returns computed plane coefficients used for standard deviation computing*/
			Eigen::VectorXf getPlaneCoefs() const
			{
				return m_planeCoefs;
			}
			
		private:
			Eigen::VectorXf m_planeCoefs;
			bool m_computed = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
			long double m_standardDeviation = 0.0;
			long double m_maxPointPlaneDistance = 0.0;

			double m_distanceThreshold = 0.01;
			double m_sigma = 0.1;
			int m_nrThreads = 4;
			int m_lineCheckoutStep = 30;
			int m_linearRepetitions = 2;
			double m_AngularCoefficientMax = 3.0;
			double m_planeIdentificationThreshold = 0.3;
		};
	}
}
