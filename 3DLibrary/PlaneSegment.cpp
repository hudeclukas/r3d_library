#include "stdafx.h"
#include <pcl/filters/extract_indices.h>
#include "PlaneSegment.h"


r3d::prims::PlaneSegment::PlaneSegment()
{
}


r3d::prims::PlaneSegment::~PlaneSegment()
{
}

bool r3d::prims::PlaneSegment::computeOutliers()
{
	if (coefficients.size() < 4)
	{
		return false;
	}

	if (inliers.empty())
	{
		return false;
	}

	if (cloudData.empty())
	{
		return false;
	}

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	//prepare extractor
	extract.setInputCloud(cloudData.makeShared());
	pcl::PointIndices::Ptr indices = boost::make_shared<pcl::PointIndices>();
	indices->indices = inliers;
	extract.setIndices(indices);
	/*!< extract inliers = line points*/
	extract.setNegative(true);
	extract.filter(outliers);
	return true;
}
