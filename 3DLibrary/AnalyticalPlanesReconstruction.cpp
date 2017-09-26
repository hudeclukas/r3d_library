#include "stdafx.h"

#include <pcl/common/intersections.h>

#include "AnalyticalPlanesReconstruction.h"

#include "CommonUtil.h"
#include "DataStatistics.h"
#include "Wall.h"


r3d::mtds::AnalyticalPlanesReconstruction::AnalyticalPlanesReconstruction()
	: Method("Analytical reconstruction", "AnalyticalPlanesReconstruction")
{
	setDefaultAttributes();
}

r3d::mtds::AnalyticalPlanesReconstruction::AnalyticalPlanesReconstruction(const std::string name, const char * config)
	: Method(name, config)
{
	setDefaultAttributes();
}

r3d::mtds::AnalyticalPlanesReconstruction::~AnalyticalPlanesReconstruction()
{
}

void r3d::mtds::AnalyticalPlanesReconstruction::setConfiguration(TiXmlNode* param)
{
	/*!< returns parameter as char* */
	if (param != nullptr && param->FirstChild() != nullptr) {
		m_KDTreeSearch = std::atoi(param->FirstChildElement("KDTreeSearch")->GetText());
		m_MinClusterSize = std::atoi(param->FirstChildElement("MinClusterSize")->GetText());
		m_MaxClusterSize = std::atoi(param->FirstChildElement("MaxClusterSize")->GetText());
		m_NumberOfNeighbours = std::atoi(param->FirstChildElement("NumberOfNeighbours")->GetText());
		m_DispersionIterations = std::atoi(param->FirstChildElement("DispersionIterations")->GetText());
		m_SmoothnessThreshold = std::atof(param->FirstChildElement("SmoothnessThreshold")->GetText());
		m_CurvatureThreshold = std::atof(param->FirstChildElement("CurvatureThreshold")->GetText());
		m_DistanceThreshold = std::atof(param->FirstChildElement("DistanceThreshold")->GetText());
		m_Probability = std::atof(param->FirstChildElement("Probability")->GetText());
		m_RanSACLine = std::atof(param->FirstChildElement("RanSACLine")->GetText());
		m_NumberOfThreads = std::atoi(param->FirstChildElement("NumberOfThreads")->GetText());
		m_ConcaveAlpha = std::atof(param->FirstChildElement("ConcaveAlpha")->GetText());
		m_orderingAlpha = std::atof(param->FirstChildElement("OrderingAlpha")->GetText());
		m_CornerRadiusMultiplicator = std::atoi(param->FirstChildElement("CornerRadiusMultiplicator")->GetText());
		m_PlaneBorderThreshold = std::atoi(param->FirstChildElement("PlaneBorderThreshold")->GetText());
		m_filterAngle1 = std::atof(param->FirstChildElement("FilterAngle1")->GetText());
		m_filterAngle2 = std::atof(param->FirstChildElement("FilterAngle2")->GetText());

		m_dataStatistics.configure(*param->FirstChildElement("DataPreprocessing"));
	}

}



bool r3d::mtds::AnalyticalPlanesReconstruction::reconstruct(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& coloredCloudI, std::vector<pcl::PointIndices>& clustersI, std::vector<r3d::obj::Wall>& planesO)
{
	std::ofstream log;
	time_t time = std::time(nullptr);
	char * dt = ctime(&time);
	std::string file;
	std::string name("AlRecon_");
	std::string type(".log");
	file = name + std::to_string(time) + type;

	log.open(file);

	log << "Analytical reconstruction - started <at> " << dt << endl;

	if (clustersI.size() > 0)
	{
		std::vector<bool> isUsefulPlane;

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Analytical reconstruction - RANSAC coefficients start <at> " << dt << endl;

		/*!< compute plane coefficients for all clusters and flag valid plane clusters */
		computePlanarCoefficients(coloredCloudI, clustersI, m_clusterCoefficients, isUsefulPlane, planesO);

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Analytical reconstruction - RANSAC coefficients done <at> " << dt << endl;

		for (auto i = 0; i < planesO.size(); ++i)
		{
			planesO[i].setPointDispersion(m_dataStatistics.segmentPointDispersion(planesO[i].getObjectData().makeShared(), m_DispersionIterations));
#ifdef _DEBUG
			log << "Analytical reconstruction - Statistics - wall pointCount: " << planesO[i].getObjectData().size() << std::endl;
			log << "Analytical reconstruction - Statistics - wall pointDispersion: " << planesO[i].getPointDispersion() << std::endl;
#endif
			m_standardDeviations.push_back(planesO[i].getStandardDeviation());
#ifdef _DEBUG
			log << "Analytical reconstruction - Statistics - wall sd: " << planesO[i].getStandardDeviation() << std::endl;
			log << "Analytical reconstruction - UsefulPlane " << i << " is " << isUsefulPlane[i] << std::endl;
#endif
	}

		log << "Analytical reconstruction - reconstruct plane corner points start <at> " << dt << endl;

		/*!< reconstruct planes */
		reconstructWallPlanes(isUsefulPlane, planesO);

		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Analytical reconstruction - planes reconstructed <at> " << dt << endl;
}
	else
	{
		time = std::time(nullptr);
		dt = ctime(&time);
		log << "Analytical reconstruction - clusters are empty <at> " << dt << endl;
		return false;
	}

	log.close();
	return true;
}

void r3d::mtds::AnalyticalPlanesReconstruction::setDefaultAttributes()
{
	m_KDTreeSearch = 50;
	m_MinClusterSize = 200;
	m_MaxClusterSize = 1000000;
	m_NumberOfNeighbours = 10;
	m_DispersionIterations = 100;
	m_SmoothnessThreshold = 3.0;
	m_CurvatureThreshold = 0.3;
	m_DistanceThreshold = 0.01;
	m_Probability = 0.8;
	m_RanSACLine = 0.05;
	m_NumberOfThreads = 4;
	m_ConcaveAlpha = 0.1;
	m_orderingAlpha = 1;
	m_CornerRadiusMultiplicator = 10;
	m_PlaneBorderThreshold = 50;
	m_filterAngle1 = 3;
	m_filterAngle2 = 7;

	m_segmentedBorder = nullptr;
}

/*! extracts edge clusters from inputcloud, erases PointIndices from clusters vector and copies edge points to edge outputcloud. Note: Output cloud must be initialised */
void r3d::mtds::AnalyticalPlanesReconstruction::extractEdgeClusters(
	const pcl::PointCloud<pcl::PointXYZRGB>& inputCloud,
	std::vector<pcl::PointIndices>& clusters,
	pcl::PointCloud<pcl::PointXYZRGB>& planes,
	pcl::PointCloud<pcl::PointXYZRGB>& edges)
{
	pcl::PointIndices::Ptr edgeIndices(new pcl::PointIndices);

	auto cluster = clusters.begin();

	/*!< fill edgeIndices from clusters containing indices of edges (red ones) */
	while (cluster != clusters.end())
	{
		edgeIndices->indices.insert(edgeIndices->indices.end(), cluster->indices.begin(), cluster->indices.end());
		++cluster;
	}
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(inputCloud.makeShared());
	extract.setIndices(edgeIndices);
	extract.setNegative(true);
	extract.filter(edges);
	extract.setNegative(false);
	extract.filter(planes);
}

void r3d::mtds::AnalyticalPlanesReconstruction::computePlanarCoefficients(
	const pcl::PointCloud <pcl::PointXYZRGB>::Ptr &coloredCloudI,
	std::vector <pcl::PointIndices> &clustersI,
	std::vector<Eigen::Vector4f> &clusterCoefficientsO,
	std::vector<bool> &isUsefulPlaneO,
	std::vector<r3d::obj::Wall> &wallsO)
{
	std::sort(clustersI.begin(), clustersI.end(), CommonUtil::comparePointIndicesDesc);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	for (auto i = 0; i < clustersI.size(); ++i)
	{
		
		/*!< set-up wall */
		Eigen::Vector4f coeffs;
		CommonUtil::planeCoefficients<pcl::PointXYZRGB>(*coloredCloudI, clustersI[i], coeffs, m_Probability, m_DistanceThreshold);
		//Eigen::VectorXf coef = { coeffs[0], coeffs[1], coeffs[2], coeffs[3] };

		obj::Wall wall;
		wall.setShape(r3d::prims::Polygon(coeffs));
		wallsO.push_back(wall);
		clusterCoefficientsO.push_back(coeffs);

		extract.setInputCloud(coloredCloudI);
		extract.setIndices(boost::make_shared<pcl::PointIndices>(clustersI[i]));
		// no need to set extractor negative
		// extract.setNegative(false); 
		pcl::PointCloud<pcl::PointXYZRGB> tmpCloud;
		extract.filter(tmpCloud);
		wallsO[i].setObjectData(tmpCloud);

		m_dataStatistics.setCloud(wallsO[i].getObjectData().makeShared());
		m_dataStatistics.setPlaneCoefs(coeffs);
		m_dataStatistics.computeRoughness();
		auto sDeviation = m_dataStatistics.getStandardDeviation();
		wallsO[i].setStandardDeviation(sDeviation);

		/*!< check if the cluster is valid plane */
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(tmpCloud, *xyzCloud);
		bool isPlane = m_dataStatistics.isPlane(xyzCloud, coeffs, wallsO[0].getStandardDeviation());
		isUsefulPlaneO.push_back(isPlane);
	}
}

void r3d::mtds::AnalyticalPlanesReconstruction::filterHullPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr hull, int closestIndex, std::vector<int>& usefulPoints)
{
	int id0 = closestIndex;
	int id1 = closestIndex + 1;
	int id2 = closestIndex + 2;
	long double vf1 = m_filterAngle1 * PI / 180;
	long double vf2 = PI - m_filterAngle2 * PI / 180;

	for (id2; id2 < hull->size() + closestIndex + 2; ++id2)
	{
		auto angle12 = r3d::CommonUtil::getAngle3D(
			(*hull)[id0 % hull->size()],
			(*hull)[id1 % hull->size()],
			(*hull)[id2 % hull->size()]);
		auto angle02 = r3d::CommonUtil::getAngle3D(
			(*hull)[id1 % hull->size()],
			(*hull)[id2 % hull->size()],
			(*hull)[id0 % hull->size()]);

		if (vf1 < angle12 || angle02 < vf2)
		{
			usefulPoints.push_back(id0 % hull->size());
			id0 = id1;
		}
		id1 = id2;
	}

	// check first and last point from usefulPointsuse
	if (usefulPoints.size() > 2)
	{
		id0 = usefulPoints.size() - 1;
		id1 = 0;
		id2 = 1;

		auto angle12 = r3d::CommonUtil::getAngle3D(
			(*hull)[usefulPoints[id0]],
			(*hull)[usefulPoints[id1]],
			(*hull)[usefulPoints[id2]]);
		auto angle02 = r3d::CommonUtil::getAngle3D(
			(*hull)[usefulPoints[id1]],
			(*hull)[usefulPoints[id2]],
			(*hull)[usefulPoints[id0]]);

		if (vf1 >= angle12 && angle02 >= vf2)
		{
			usefulPoints.erase(usefulPoints.begin());
		}
	}
}

void r3d::mtds::AnalyticalPlanesReconstruction::fitHullToSegmentPoints(r3d::obj::Wall& wallI, pcl::PointCloud<pcl::PointXYZ>& cornerPoints, pcl::PointCloud<pcl::PointXYZ> filteredHull) const
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTreeCornerPoints;
	kdTreeCornerPoints.setInputCloud(cornerPoints.makeShared());
	double radius = m_CornerRadiusMultiplicator * wallI.getPointDispersion();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTreeSegment;

	kdTreeSegment.setInputCloud(wallI.getObjectData().makeShared());
	for (auto idx = 0; idx < filteredHull.size(); ++idx)
	{
		pcl::PointXYZ pt = filteredHull[idx];

		/*!< check if there is a corner point around hull corner point */
		std::vector<int> indices(1);
		std::vector<float> sqrDst(1);
		if (cornerPoints.size() > 0)
		{
			kdTreeCornerPoints.radiusSearch(pt, radius, indices, sqrDst, 1);
		}
		kdTreeSegment.nearestKSearch(pt, 1, indices, sqrDst);
		pcl::PointXYZ newCorner = wallI.getObjectData()[indices[0]];
		cornerPoints.push_back(newCorner);
		kdTreeCornerPoints.setInputCloud(cornerPoints.makeShared());
	}
}

void r3d::mtds::AnalyticalPlanesReconstruction::fitHullToSegmentPoints(r3d::obj::Wall& wallI, pcl::PointCloud<pcl::PointXYZ>& cornerPoints, pcl::PointCloud<pcl::PointXYZ>& hull, std::vector<int>& indices) const
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTreeSegment;
	kdTreeSegment.setInputCloud(wallI.getObjectData().makeShared());
	for (auto idx = 0; idx < indices.size(); ++idx)
	{
		pcl::PointXYZ pt = hull[indices[idx]];

		/*!< check if there is a corner point around hull corner point */
		std::vector<int> kdIndices(1);
		std::vector<float> sqrDst(1);
		kdTreeSegment.nearestKSearch(pt, 1, kdIndices, sqrDst);
		pcl::PointXYZ newCorner = wallI.getObjectData()[kdIndices[0]];
		cornerPoints.push_back(newCorner);
	}
}

void r3d::mtds::AnalyticalPlanesReconstruction::sortSegmentHull(pcl::PointCloud<pcl::PointXYZ>& hull)
{
	auto tmpHull = hull;
	hull.clear();

	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;

	pcl::PointXYZ point = *(tmpHull.end() - 1);
	tmpHull.points.pop_back();
	hull.push_back(point);

	std::vector<int> index(1);
	std::vector<float> distance(1);
	while (!tmpHull.empty())
	{
		kdTree.setInputCloud(tmpHull.makeShared());
		kdTree.nearestKSearch(point, 1, index, distance);
		point = tmpHull[index[0]];
		hull.push_back(point);
		tmpHull.erase(tmpHull.begin() + index[0]);
	}
	std::ofstream log;
	log.open("hullorder.log");
	for (auto i = 0; i < hull.size(); ++i)
	{
		log << hull[i] << std::endl;
	}
	log.close();
}

void r3d::mtds::AnalyticalPlanesReconstruction::refineCornerPoints(std::vector<r3d::obj::Wall>& wallsIO)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	m_segmentedBorder = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//pcl::ConcaveHull<pcl::PointXYZ> ccHull;
	//ccHull.setAlpha(m_ConcaveAlpha);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (auto i = 0; i < wallsIO.size(); ++i)
	{
		auto hull = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		auto &cornerPoints = wallsIO[i].getCornerPoints();
		r3d::CommonUtil::orderPointsInConcaveHull(cornerPoints.makeShared(), cornerPoints, m_orderingAlpha);
		if (!CommonUtil::verifyPlaneCornerPoints(wallsIO[i].getObjectData().makeShared(), cornerPoints, hull, m_PlaneBorderThreshold * wallsIO[i].getPointDispersion(), m_ConcaveAlpha))
		{
			*m_segmentedBorder += *hull;

			// sorting points in hull
			sortSegmentHull(*hull);

			/*!< filter useless points from hull and complete the coordinates */
			std::vector<int> closest(1);
			if (cornerPoints.size() > 0)
			{
				kdTree.setInputCloud(hull);

				pcl::PointXYZ searchPoint = cornerPoints[0];
				std::vector<float> sqrDstToClosest(1);
				kdTree.nearestKSearch(searchPoint, 1, closest, sqrDstToClosest);
			}
			else
			{
				closest[0] = 0;
			}

			std::vector<int> usefullPoints;
			filterHullPoints(hull, closest[0], usefullPoints);

			fitHullToSegmentPoints(wallsIO[i], cornerPoints, *hull, usefullPoints);
			std::ofstream log;
			log.open("cornerpoints.log");
			for (auto j = 0; j < cornerPoints.size(); ++j)
			{
				log << cornerPoints[j] << std::endl;
			}
			log.close();
		}
	}
}

void r3d::mtds::AnalyticalPlanesReconstruction::reconstructWallPlanes(
	const std::vector<bool> &isUsefulPlaneI,
	std::vector<r3d::obj::Wall> &wallsIO)
{
	std::ofstream log;
	time_t time = std::time(nullptr);
	char * dt = ctime(&time);
	log.open("reconstruction.log");
	log << "Walls size: " << wallsIO.size()
		<< "\nCombinations started <at> " << dt << std::endl;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
	for (auto i = 0; i < wallsIO.size(); i++)
	{
		for (auto j = i + 1; j < wallsIO.size(); j++)
		{
			for (auto k = j + 1; k < wallsIO.size(); k++)
			{
				std::vector<Eigen::Vector4f*> selCoeffs = { &wallsIO[i].getShape().getCoefficients(), &wallsIO[j].getShape().getCoefficients(), &wallsIO[k].getShape().getCoefficients() };

				/*!< if all planes are valid */
				if (isUsefulPlaneI[i] && isUsefulPlaneI[j] && isUsefulPlaneI[k])
				{
					Eigen::Vector3f intersectionPoint;

					/*!< if planes have intersection */
					if (pcl::threePlanesIntersection(*selCoeffs[0], *selCoeffs[1], *selCoeffs[2], intersectionPoint))
					{
#ifdef _DEBUG
						log << "Plane 0 coefficients\n" << selCoeffs[0][0] << std::endl;
						log << "Plane 1 coefficients\n  " << selCoeffs[1][0] << std::endl;
						log << "Plane 2 coefficients\n  " << selCoeffs[2][0] << std::endl;
						log << "Planes intersection X=" << intersectionPoint[0] << " Y=" << intersectionPoint[1] << " Z=" << intersectionPoint[2] << std::endl;
#endif
						std::vector<bool> isNearIntersection = { true, true, true };
						pcl::PointXYZ searchPoint;
						searchPoint.x = intersectionPoint[0];
						searchPoint.y = intersectionPoint[1];
						searchPoint.z = intersectionPoint[2];
						std::vector<int> wallIndexes = { i, j, k };
						for (auto idx = 0; idx < wallIndexes.size(); ++idx)
						{
							std::vector<int> pointIdxRadiusSearch;
							std::vector<float> pointRadiusSquaredDistance;
							kdTree.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(wallsIO[wallIndexes[idx]].getObjectData()));
							if (kdTree.radiusSearch(searchPoint, m_CornerRadiusMultiplicator * wallsIO[wallIndexes[idx]].getPointDispersion(), pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
							{
#ifdef _DEBUG
								log << "KN search " << i << " " << j << " " << k 
									<< " distance: " << m_CornerRadiusMultiplicator * wallsIO[wallIndexes[idx]].getPointDispersion() 
									<< " nearest points " << pointIdxRadiusSearch.size()
									<< std::endl;
#endif
						}
							else
							{
								isNearIntersection[idx] = false;
							}
#ifdef _DEBUG
							if (isNearIntersection[idx])
							{
								log << "idx " << idx << " isNearIntersection: " << isNearIntersection[idx] << std::endl;
							}
#endif
				}
						/*!< if all of them have at least 1 concave hull point around the intersection point */
						if (isNearIntersection[0] && isNearIntersection[1] && isNearIntersection[2])
						{
							wallsIO[i].addCornerPoint(intersectionPoint);
							wallsIO[j].addCornerPoint(intersectionPoint);
							wallsIO[k].addCornerPoint(intersectionPoint);
						}
			}
		}
	}
}
	}

	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Combinations ended, Refine corners <at> " << dt << std::endl;

	refineCornerPoints(wallsIO);

	time = std::time(nullptr);
	dt = ctime(&time);
	log << "Reconstruction end <at>" << dt << std::endl;
	log.close();
}
//m_segmentedPlanes = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
//extractEdgeClusters(*colored_cloud, clusters, *m_segmentedPlanes, *edges);
//m_segmentedBorder = edges->makeShared();

//log << "Growing Region - lines segmentation" << endl;

//// Uncomment if you want to start lines segmentation
////**************************************************************
//pcl::PointCloud<pcl::PointXYZRGB> tmpFilteredLine;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpFilteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::ExtractIndices<pcl::PointXYZRGB> extract;

//int nrPoints = (int) m_segmentedBorder->points.size();
//while (m_segmentedBorder->points.size() > 0.02 * nrPoints)
//{
//	pcl::SampleConsensusModelLine<pcl::PointXYZRGB>::Ptr model
//		(new pcl::SampleConsensusModelLine<pcl::PointXYZRGB>(m_segmentedBorder));
//	pcl::PointIndices::Ptr ransacIndices(new pcl::PointIndices);
//	Eigen::VectorXf coefficients;
//	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model, m_RanSACLine);
//	ransac.computeModel();
//	
//	ransac.getInliers(ransacIndices->indices);
//	ransac.getModelCoefficients(coefficients);

//	//prepare extractor
//	extract.setInputCloud(m_segmentedBorder);
//	extract.setIndices(ransacIndices);
//	//extract inliers = line points
//	extract.setNegative(false);
//	extract.filter(tmpFilteredLine);
//	m_segmentedLines.push_back(r3d::prims::Line(coefficients,tmpFilteredLine));
//	
//	//extract outliers of the line points
//	extract.setNegative(true);
//	extract.filter(*tmpFilteredCloud);

//	//swap for next round of line segmentation
//	m_segmentedBorder.swap(tmpFilteredCloud);
//}
////**************************************************************

//log << "Growing Region - lines segmented" << endl;


// used for finding corner points... unsuccessfull
//auto distance = m_CornerRadiusMultiplicator * wallsIO[i].getPointDispersion();
//auto originalSize = hull->size();
//auto lineModel = boost::make_shared<pcl::SampleConsensusModelLine<pcl::PointXYZ>>(pcl::SampleConsensusModelLine<pcl::PointXYZ>(wallsIO[i].getCornerPoints().makeShared()));
//std::vector<pcl::PointCloud<pcl::PointXYZ>> lineCloudsStack;
//std::vector<Eigen::VectorXf> lineCoefficientsStack;
//while (hull->size() > 0.1 * originalSize && hull->size() > 3)
//{
//#ifdef _DEBUG
//	log << "hull->size() vs original Size: " << hull->size() << " " << originalSize << std::endl;
//#endif
//	// compute RANSAC model for the first line set
//	auto hullModel = boost::make_shared<pcl::SampleConsensusModelLine<pcl::PointXYZ>>(pcl::SampleConsensusModelLine<pcl::PointXYZ>(hull));
//	pcl::PointIndices::Ptr lineInliers(new pcl::PointIndices());
//	std::vector<int> lineCoordinates;
//	Eigen::VectorXf coefficients;
//	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(hullModel, m_RanSACLine);
//	ransac.computeModel();
//	// get line inliers	and coefficients
//	ransac.getInliers(lineInliers->indices);
//	ransac.getModelCoefficients(coefficients);
//	// find line coordinates
//	lineModel->selectWithinDistance(coefficients, distance, lineCoordinates);
//#ifdef _DEBUG
//	log << "Coordinates found: " << lineCoordinates.size() << std::endl;
//#endif
//	if (lineCoordinates.size() < 2)
//	{
//		// if line does not have ending coordinates, it points can be deleted
//		lineCloudsStack.push_back(pcl::PointCloud<pcl::PointXYZ>(*hullTmp, lineInliers->indices));
//		lineCoefficientsStack.push_back(coefficients);
//	}
//	// extract found points
//	extract.setInputCloud(hull);
//	extract.setIndices(lineInliers);
//	pcl::PointCloud<pcl::PointXYZ> tmpFilteredHull;
//	extract.setNegative(true);
//	extract.filter(tmpFilteredHull);
//	tmpFilteredHull.swap(*hull);
//	lineInliers.reset();
//}
//
//// check intersections of all lines combinations 2/N 
//for (auto j = 0; j < lineCoefficientsStack.size(); ++j)
//{
//	for (auto k = j + 1; k < lineCoefficientsStack.size(); ++k)
//	{
//		auto square_dst = 5 * std::pow(wallsIO[j].getStandardDeviation() + wallsIO[k].getStandardDeviation(), 2);
//		Eigen::Vector4f intersectionPoint;
//		if (pcl::lineWithLineIntersection(lineCoefficientsStack[j], lineCoefficientsStack[k], intersectionPoint, square_dst))
//		{
//#ifdef _DEBUG
//			log << "intersection " << j << ", " << k << ":\n" << intersectionPoint << std::endl;
//#endif
//			std::vector<int> pointIdxRadiusSearch;
//			std::vector<float> pointRadiusSquaredDistance;
//			kdTree.setInputCloud(lineCloudsStack[j].makeShared());
//			pcl::PointXYZ searchPoint;
//			searchPoint.x = intersectionPoint[0];
//			searchPoint.y = intersectionPoint[1];
//			searchPoint.z = intersectionPoint[2];
//			if (kdTree.radiusSearch(searchPoint, distance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//			{
//				kdTree.setInputCloud(lineCloudsStack[k].makeShared());
//				if (kdTree.radiusSearch(searchPoint, distance, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//				{
//					wallsIO[i].addCornerPoint(searchPoint);
//				}
//			}
//		}
//	}
//}