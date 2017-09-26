#pragma once

namespace r3d {

	namespace prims {
	
		class Line
		{
		public:
			Line();
			Line(Eigen::VectorXf &coeff, pcl::PointCloud<pcl::PointXYZRGB> linePoints);
			~Line();

			void setCoefficients(Eigen::VectorXf &coeff);
			void setCoefficients(float pointX, float pointY, float pointZ, float directionX, float directionY, float directionZ);
			void setCloud(pcl::PointCloud<pcl::PointXYZRGB> linePoints);

			Eigen::VectorXf getCoefficients();
			pcl::PointCloud<pcl::PointXYZRGB> getPoints();

		private:
			Eigen::VectorXf m_lineCoefficients;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_linePoints;
		};
	
	}
}