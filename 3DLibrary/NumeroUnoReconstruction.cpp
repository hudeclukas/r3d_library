#include "stdafx.h"
#include "NumeroUnoReconstruction.h"


r3d::mtds::NumeroUnoReconstruction::NumeroUnoReconstruction()
: Method("NumeroUno - 1","NumeroUnoReconstruction")
{
}


r3d::mtds::NumeroUnoReconstruction::~NumeroUnoReconstruction()
{
}

void r3d::mtds::NumeroUnoReconstruction::visualise(pcl::visualization::PCLVisualizer* viewer)
{

}

void r3d::mtds::NumeroUnoReconstruction::setConfiguration(TiXmlNode* param) {
	if (param != NULL && param->FirstChild() != NULL) 
	{
		//settings here...
		//example:
		//m_OMPNumberOfCores = std::atoi(getParameter(param, "OMPNumberOfCores"));
	}
}

bool r3d::mtds::NumeroUnoReconstruction::calculate()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = m_dataSet.getXYZData().makeShared();
	pcl::PointCloud<pcl::PointXYZ>::Ptr z_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_slices(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	std::vector<float> projected_points;
	float mean_point_z;
	float total_distance_z;
	int max_iteration;
	float bandwidth;
	float position;
	float min;
	float max;
	pcl::PointXYZRGB slice_point;
	std::vector<float> slices;

	mean_point_z = calculate_mean_point(cloud);
	calculate_normals(cloud_normals, cloud, 6);
	calculate_projected_points(&projected_points, cloud_normals, cloud, z_cloud, 0.05);
	total_distance_z = calculate_total_distance_z(cloud, &min, &max);
	max_iteration = calculate_windows(total_distance_z, mean_point_z);
	bandwidth = (total_distance_z / max_iteration) / 2;
	position = min - bandwidth;
	slice_point.r = 255;
	slice_point.g = 0;
	slice_point.b = 0;
	slice_point.x = 0.0;
	slice_point.y = 0.0;

	//std::cout << "mean_point==" << mean_point_z << endl;
	//std::cout << "distance==" << total_distance_z << endl;
	//std::cout << "okien==" << max_iteration << endl;

	for (int i = 0; i < max_iteration; ++i)
	{
		float temp;
		if (temp = get_centroid(&position, bandwidth, projected_points))
		{
			if (temp != -1)
				slices.push_back(temp);
		}
	}
	std::sort(slices.begin(), slices.end());
	auto last = std::unique(slices.begin(), slices.end());
	slices.erase(last, slices.end());

	for (float i : slices)
	{
		//std::cout << "center==" << i << endl;
		slice_point.z = i;
		(*rgb_slices).push_back(slice_point);
	}

	m_outputCloud = rgb_slices;
	/*m_dataSet.setData(*rgb_slices.get());
	m_dataSet.setName("rgbData");*/

	return true;
}

int r3d::mtds::NumeroUnoReconstruction::calculate_windows(float dist, float m_point)
{
	int count = 0;
	float win = dist;
	while (win - (m_point*30.0) >= 0.0)
	{
		win -= (m_point*30.0);
		++count;
	}
	int num_win = (count > 1) ? (count) : (count + 1);
	return num_win;
}

float r3d::mtds::NumeroUnoReconstruction::calculate_mean_point(pcl::PointCloud<pcl::PointXYZ>::Ptr cld)
{
	int count = (*cld).size();
	float sum = 0.0;
	std::vector<float> z_values_distinct;
	for (int i = 0; i <count; ++i)
	{
		z_values_distinct.push_back((*cld).points.at(i).z);
	}
	std::sort(z_values_distinct.begin(), z_values_distinct.end());
	auto last = std::unique(z_values_distinct.begin(), z_values_distinct.end());
	z_values_distinct.erase(last, z_values_distinct.end());
	count = z_values_distinct.size();
	for (int i = 1; i <count; ++i)
	{
		sum += abs(z_values_distinct.at(i - 1) - z_values_distinct.at(i));
	}
	return (sum / count);
}

float r3d::mtds::NumeroUnoReconstruction::calculate_total_distance_z(pcl::PointCloud<pcl::PointXYZ>::Ptr cld, float* min, float* max)
{
	int count = (*cld).size();
	(*min) = (*cld).points.at(0).z;
	(*max) = (*min);
	for (int i = 0; i <count; ++i)
	{
		if ((*cld).points.at(i).z < (*min))
			(*min) = (*cld).points.at(i).z;
		if ((*cld).points.at(i).z >(*max))
			(*max) = (*cld).points.at(i).z;
	}
	float distance = (*max) - (*min);
	return distance;
}

int r3d::mtds::NumeroUnoReconstruction::meanshift(float* cntr, float b_width, std::vector<float> points, int* dir)
{
	if (points.empty() || (((*cntr) + b_width) < points.front()) || (((*cntr) - b_width) > points.back()))
		return NULL;
	std::vector<float> window;
	for (float i : points)
	{
		if (i >= (*cntr) - b_width && i <= (*cntr) + b_width)
			window.push_back(i);
	}
	if (window.empty())
	{
		return NULL;
	}
	else
	{
		float sum = 0.0;

		for (float i : window)
		{
			sum += (i - (*cntr));
		}

		sum = sum / window.size();

		if (-0.0000001 < sum && sum < 0.0000001)
		{
			return 1;
		}
		else
		{
			if (sum <= -0.0000001)
			{
				if ((*dir) == -1 || (*dir) == 0)
				{
					(*dir) = -1;
					(*cntr) += sum;
					return -1;
				}
				else
				{
					(*dir) = -1;
					sum = sum / 2;
					(*cntr) += sum;
					if (sum > -0.00001)
					{
						return 1;
					}
					else
					{
						return -1;
					}
				}
			}
			else
			{
				if (sum >= 0.0000001)
				{
					if ((*dir) == 1 || (*dir) == 0)
					{
						(*dir) = 1;
						(*cntr) += sum;
						return -1;
					}
					else
					{
						(*dir) = 1;
						sum = sum / 2;
						(*cntr) += sum;
						if (sum < 0.00001)
						{
							return 1;
						}
						else
						{
							return -1;
						}
					}
				}
			}
		}
	}
}


void r3d::mtds::NumeroUnoReconstruction::calculate_normals(pcl::PointCloud<pcl::Normal>::Ptr nrmls, pcl::PointCloud<pcl::PointXYZ>::Ptr cld, int k)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cld);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setKSearch(k);
	normal_estimator.compute(*nrmls);
	nrmls->at(0);
}

void r3d::mtds::NumeroUnoReconstruction::calculate_projected_points(std::vector<float>* proj_points, pcl::PointCloud<pcl::Normal>::Ptr nrmls, pcl::PointCloud<pcl::PointXYZ>::Ptr cld, pcl::PointCloud<pcl::PointXYZ>::Ptr z_cld, float err)
{
	int count = 0;
	for (pcl::Normal i : (*nrmls))
	{
		if ((i.normal_z > (1.0 - err)) || (i.normal_z < (err - 1.0)))
		{
			(*z_cld).push_back(pcl::PointXYZ(0.0, 0.0, (*cld).points.at(count).z));
			(*proj_points).push_back((*cld).points.at(count).z);
		}
		++count;
	}
	std::sort((*proj_points).begin(), (*proj_points).end());
}

float r3d::mtds::NumeroUnoReconstruction::get_centroid(float* pos, float b_width, std::vector<float> proj_points)
{
	int result;
	int direction = 0;
	(*pos) += (b_width * 2.0);
	float center = (*pos);

	result = meanshift(&center, b_width, proj_points, &direction);

	while (result == -1)
	{
		result = meanshift(&center, b_width, proj_points, &direction);
	}

	if (result != NULL)
	{
		return center;
	}
	else
	{
		return -1;
	}
}