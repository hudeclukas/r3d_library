#ifndef __CLOSEDSPACE__
#define __CLOSEDSPACE__

#include <pcl/visualization/pcl_visualizer.h>
#include "Wall.h"

#pragma once

/*! \namespace r3d
    \brief Our team dedicated namespace R3D.

    Contains other subnamespaces to distinct method types in our reconstruction project
*/
namespace r3d {
	/*! \namespace space

	Contains space and data storing and administrating classes
	*/
	namespace space {
		/*! \class ClosedSpace ClosedSpace.h "ClosedSpace.h"
		*  \brief This class contains all stored data, reconstructed primitives and objects segmented in scene.
		*
		*   \author Lukas Hudec
		*/
		class ClosedSpace
		{
		public:
			ClosedSpace();
			/*! \brief Constructor that sets source and space/area name
			*  @param [in] source files names that contains point cloud data
			*  @param [in] name scene name, in case to hash this "data buffer"
			*/
			ClosedSpace(std::vector<std::string> source, std::string name);
			
			virtual ~ClosedSpace();
			/*! \fn setName
			*/
			void setName(std::string name)
			{
				m_name = name;
			}
			std::string getName() const
			{
				return m_name;
			}
			void setData(pcl::PointCloud<pcl::PointXYZ> &data)
			{
				pcl::copyPointCloud(data,m_inputXYZ);
			}
			void setData(pcl::PointCloud<pcl::RGB> &data)
			{
				pcl::copyPointCloud(data, m_inputRGB);
			}
			void setData(pcl::PointCloud<pcl::Normal> &data)
			{
				pcl::copyPointCloud(data, m_inputNormal);
			}
			void clear()
			{
				if (!m_inputXYZ.empty())
				{
					m_inputXYZ.clear();
				}
				if (!m_inputRGB.empty())
				{
					m_inputRGB.clear();
				}
				if (!m_inputNormal.empty())
				{
					m_inputNormal.clear();
				}
				if (!m_reducedDataXYZRGB.empty())
				{
					m_reducedDataXYZRGB.clear();
				}
				m_objects.clear();
				m_walls.clear();
			}
			pcl::PointCloud<pcl::PointXYZ>& getXYZData()
			{
				return m_inputXYZ;
			}
			pcl::PointCloud<pcl::RGB>& getRGBData()
			{
				return m_inputRGB;
			}
			pcl::PointCloud<pcl::Normal>& getNormalData()
			{
				return m_inputNormal;
			}

			void setSource(std::vector<std::string> source)
			{
				m_source = source;
			}
			std::vector<std::string> getSource() const
			{
				return m_source;
			}

			/*! \fn pcl::PointCloud<PointT> addObject(r3d::obj::Object &object)
			* \brief Adds new segmentated Object into object stack
			* counts the reduced data and adds them to reduced PointCloud
			* @param [in] object vector of string file names
			*/
			void addObject(r3d::obj::Object &object);
			std::vector<r3d::obj::Object*> getObjects() { return m_objects; }
			
			/*! \fn int addWall(r3d::obj::Wall &wall)
			* \brief Adds new segmentated/reconstructed Wall object into wall stack
			* @param [in] wall vector of string file names
			* @return int number of segmentated walls in stack
			*/
			int addWall(r3d::obj::Wall &wall);
			std::vector<r3d::obj::Wall>& getWalls() { return m_walls; }

			/*! \fn void visualise(pcl::visualization::PCLVisualizer* viewer)
			* \brief Loads reconstructed shape data into pcl/vtk viewer in a way they should be represented and viewed
			* @param [in] viewer pcl/vtk viewer that will visualise data
			* @return bool true - if loading proceeded correctly, false otherwise
			*/
			void visualise(pcl::visualization::PCLVisualizer* viewer);
			void visualiseOriginalData(pcl::visualization::PCLVisualizer* viewer) const;

		private:

			/*! \var pcl::PointCloud<pcl::PointXYZ> *m_inputXYZ
			* \brief original XYZ data point cloud that should be used to segmentate in all functions
			* pointCloud containing original data loaded from external file
			*/
			pcl::PointCloud<pcl::PointXYZ> m_inputXYZ;
			/*! \var pcl::PointCloud<pcl::RGB> *m_inputRGB
			* \brief original RGB data point cloud that should be used to segmentate in all functions
			* pointCloud containing original data loaded from external file
			*/
			pcl::PointCloud<pcl::RGB> m_inputRGB;
			/*! \var pcl::PointCloud<pcl::Normal> *m_inputNormal
			* \brief original Normal data point cloud that should be used to segmentate in all functions
			* pointCloud containing original data loaded from external file
			*/
			pcl::PointCloud<pcl::Normal> m_inputNormal;
			
			/*! \var pcl::PointCloud<pcl::PointXYZRGB> *m_reducedDataXYZRGB
			* \brief reduced data remaining after reconstruction of original m_data
			* pointCloud containing remaining data loaded from external file - empty on constructor called
			*/
			pcl::PointCloud<pcl::PointXYZRGB> m_reducedDataXYZRGB;

			std::string m_name;
			std::vector<std::string> m_source;

			/*! \var std::vector<r3d::obj::Object> m_objects
			* \brief segmentated objects from scene
			* stack of Object-s
			*/
			std::vector<r3d::obj::Object*> m_objects;
			/*! \var std::vector<r3d::obj::Object> m_walls
			* \brief segmentated Walls from scene
			* stack of Wall-s (Object)
			*/
			std::vector<r3d::obj::Wall> m_walls;
		};
	}
}

#endif __CLOSEDSPACE__