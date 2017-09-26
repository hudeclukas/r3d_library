#ifndef __RECTANGLE__
#define __RECTANGLE__

#pragma once
#include "Primitives.h"

namespace r3d {
	namespace prims {
		class Rectangle :
			public Primitives
		{
		public:
			Rectangle();
			Rectangle(r3d::prims::Primitives *prims);
			Rectangle(Eigen::Vector4f coefficients);
			Rectangle(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C, pcl::PointXYZ D);
			~Rectangle();


			/*!
			* \brief Adds a rectangle corner coordinate
			* \details As this is a rectangle, if there is already 4 corners, it pops the first and adds this to the end
			*/
			void addCoordinate(const pcl::PointXYZ point) override;
			void setCoordinates(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C, pcl::PointXYZ D);

			void setCoefficients(Eigen::Vector4f &coeff);
			Eigen::Vector4f &getCoefficients();

		private:
			Eigen::Vector4f m_planeCoefficients;
		};
	}
}
#endif __RECTANGLE__