#pragma once
namespace r3d {
	namespace prims {
		class PlaneSegment
		{
		public:
			PlaneSegment();
			~PlaneSegment();

			Eigen::VectorXf coefficients;
			pcl::PointCloud<pcl::PointXYZRGB> cloudData;
			std::vector<int> inliers;
			pcl::PointCloud<pcl::PointXYZRGB> inliersPoints;
			pcl::PointCloud<pcl::PointXYZRGB> outliers;

			bool computeOutliers();
		};
	}
}