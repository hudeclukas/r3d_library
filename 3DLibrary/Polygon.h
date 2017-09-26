#pragma once
#include "Primitives.h"

namespace r3d {
	namespace prims {
		class Polygon : 
			public Primitives
		{
		public:
			Polygon();
			Polygon(Eigen::Vector4f coefficients);
			Polygon(pcl::PointCloud<pcl::PointXYZ> coordinates);
			Polygon(r3d::prims::Primitives *prims);

			void addCoordinate(const pcl::PointXYZ point) override;
			void addCoordinates(const pcl::PointCloud<pcl::PointXYZ> coordinateCloud);

			void setCoefficients(const Eigen::Vector4f coeff);
			Eigen::Vector4f &getCoefficients();
			~Polygon();
		private:
			Eigen::Vector4f m_planeCoefficients;
		};


	}
}