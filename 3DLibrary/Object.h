#ifndef __OBJECT__
#define __OBJECT__


#pragma once

namespace r3d {
	namespace obj {
		/*! 
		 *  \brief     Class of object.
 		 *  \details   This class is used to demonstrate a...
 		 *  \author    Lukas Hudec
 		 *  \author    Martin Jurik
 		 *  \version   1.0
 		 *  \date      2015
 		 *  \pre       First initialize the...
 		 *  \bug       Not all memory is freed when deleting an object of this class.
 		 *  \warning   Improper use can crash your application
 		 * \copyright License.
 		*/
		class Object
		{
		public:
			//! A constructor.
			/*!
			    A more elaborate description of the constructor.
			*/
			Object();
			Object(pcl::PointCloud<pcl::PointXYZ> objectData, std::string name);
			virtual ~Object();

			inline void setName(std::string name)
			{
				m_name = name;
			}
			inline std::string getName() const
			{
				return m_name;
			}
			void setObjectData(pcl::PointCloud<pcl::PointXYZ> objectData);
			void setObjectData(pcl::PointCloud<pcl::PointXYZRGB> objectData);

			pcl::PointCloud<pcl::PointXYZ>& getObjectData()
			{
				return m_originalData;
			}

			void setPointDispersion(double dispersion)
			{
				m_pointDispersion = dispersion;
			}

			double getPointDispersion() const
			{
				return m_pointDispersion;
			}

			void setStandardDeviation(long double standardDeviation)
			{
				m_standardDeviation = standardDeviation;
			}

			long double getStandardDeviation() const
			{
				return m_standardDeviation;
			}

			unsigned char getR() {
				return r;
			}

			unsigned char getG() {
				return g;
			}

			unsigned char getB() {
				return b;
			}

		private:
			unsigned char r, g, b;

		protected:

			std::string m_name;
			pcl::PointCloud<pcl::PointXYZ> m_originalData;
			/*! \var long double m_standardDeviation;
			* \brief Standard Deviation of data and sensor precision
			*/
			long double m_standardDeviation = 0.0;
			/*! \var double m_pointDispersion;
			* \brief Mean mutual distance between points in cloud
			*/
			double m_pointDispersion = 0.0;
		};
	}
}
#endif __OBJECT__