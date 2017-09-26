#include "stdafx.h"
#include "CommonUtil.h"

#include <pcl/common/intersections.h>
#include <pcl/surface/concave_hull.h>

r3d::CommonUtil::CommonUtil()
{
}


r3d::CommonUtil::~CommonUtil()
{
}

/*!
* \brief computes histogram out of all distances in vector
* use "cut" to group points in distance "cuts" /default is 0, but consider using small values as 0.0001 or 0.001
*/
std::vector<r3d::HASH> r3d::CommonUtil::computeDistancesHistogram(std::vector<double> distances, double cut)
{
	std::sort(distances.begin(), distances.end());
	std::vector<HASH> histogram;
	
	auto base = cut == 0 ? distances[0] : cut;
	auto index = 0;
	histogram.push_back({ base, 0 });
	for each (double distance in distances)
	{
		if (distance <= base)
		{
			histogram[index].points++;
		}
		else
		{
			index++;
			base = cut == 0 ? distance : base + cut;
			histogram.push_back({ base, 1 });
		}
	}

	return histogram;
}

bool r3d::CommonUtil::comparePointIndicesDesc(pcl::PointIndices& a, pcl::PointIndices& b)
{
	return a.indices.size() > b.indices.size();
}

bool r3d::CommonUtil::comparePointIndicesAsc(pcl::PointIndices& a, pcl::PointIndices& b)
{
	return a.indices.size() < b.indices.size();
}

bool r3d::CommonUtil::verifyPlaneCornerPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &planeCloud, pcl::PointCloud<pcl::PointXYZ> &cornerPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr &planeConcaveHull, double tolerance, double alpha)
{
	pcl::ConvexHull<pcl::PointXYZ> convexHull;
	convexHull.setInputCloud(cornerPoints.makeShared());
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//convexHull.reconstruct (*cornerPoints);
	convexHull.reconstruct(*tmpCloud);
	//cornerPoints.clear();
	//pcl::copyPointCloud(*tmpCloud, cornerPoints);

	// make concave hull from planeCloud
	pcl::ConcaveHull<pcl::PointXYZ> concaveHull;
	concaveHull.setInputCloud(planeCloud);
	concaveHull.setAlpha(alpha);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud2(new pcl::PointCloud<pcl::PointXYZ>);
	concaveHull.reconstruct(*tmpCloud2);
	*planeConcaveHull = *tmpCloud2->makeShared();

	int outliers = 0;
	for (int ij = 0; ij < planeConcaveHull->width; ij++)
	{
		double minDistance = 20000;
		for (int i = 0; i < cornerPoints.width; i++)
		{
			pcl::PointXYZ checked_pointXYZ;
			Eigen::Vector4f checked_point;

			pcl::PointXYZ start_pointXYZ;
			Eigen::Vector4f start_point;
			pcl::PointXYZ end_pointXYZ;
			Eigen::Vector4f end_point;

			if (i == (cornerPoints.width - 1))
			{

				start_pointXYZ = cornerPoints.points[i];
				start_point = { start_pointXYZ.x, start_pointXYZ.y, start_pointXYZ.z, 0 };

				end_pointXYZ = cornerPoints.points[0];
				end_point = { end_pointXYZ.x, end_pointXYZ.y, end_pointXYZ.z, 0 };
			}
			else {
				checked_pointXYZ = planeConcaveHull->points[ij];
				checked_point = { checked_pointXYZ.x, checked_pointXYZ.y, checked_pointXYZ.z, 0 };

				start_pointXYZ = cornerPoints.points[i];
				start_point = { start_pointXYZ.x, start_pointXYZ.y, start_pointXYZ.z, 0 };

				end_pointXYZ = cornerPoints.points[i + 1];
				end_point = { end_pointXYZ.x, end_pointXYZ.y, end_pointXYZ.z, 0 };
			}

			// get the distance of points from concave hull
			double distance = std::sqrt(pcl::sqrPointToLineDistance(checked_point, start_point, end_point));
			if (distance < minDistance) minDistance = distance;
		}

		if (minDistance > tolerance) outliers++;
	}

	double ratio = (0.0 + outliers) / planeConcaveHull->width;

#ifdef _DEBUG
	std::cout << "outliers count= " << outliers << std::endl;
	std::cout << "points count= " << planeConcaveHull->width << std::endl;
	std::cout << "ratio= " << ratio << std::endl;
#endif
	
	if (ratio < 0.15) return true;
	else return false;
}

double r3d::CommonUtil::getAngle3D(pcl::PointXYZ A, pcl::PointXYZ B, pcl::PointXYZ C)
{
	Eigen::Vector4f v1(B.x - A.x, B.y - A.y, B.z - A.z, 0.0);
	Eigen::Vector4f v2(C.x - A.x, C.y - A.y, C.z - A.z, 0.0);

	return pcl::getAngle3D(v1, v2);
}

void r3d::CommonUtil::orderPointsInConcaveHull(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZ> &outputCloud, double alphaIn)
{
	if (inputCloud->size() > 3)
	{
		pcl::ConcaveHull<pcl::PointXYZ> concaveHull;
		concaveHull.setInputCloud(inputCloud);
		concaveHull.setAlpha(alphaIn);
		concaveHull.reconstruct(outputCloud);
	}
}
