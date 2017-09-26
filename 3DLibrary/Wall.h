#ifndef __WALL__
#define __WALL__

#pragma once
#include "Object.h"
#include "Polygon.h"

namespace r3d {
	namespace obj {
		/*!
		*   \brief default shape for simple Wall object
		*   @author Lukas Hudec
		*/
		class Wall :
			public Object
		{
		public:
			Wall();
			/*! \brief Constructor for wall object, 
			*  \brief points does not have to be oriented by this order, 
			*  \brief but have to follow the same pattern
			*  @param coordinates Wall coordinates in a pointcloud
			*/
			Wall(pcl::PointCloud<pcl::PointXYZ> coordinates);
			~Wall();

			/*! \brief Only one shape per wall allowed
			*/
			void setShape(r3d::prims::Polygon shape);
			r3d::prims::Polygon& getShape()
			{
				return m_shape;
			}

			void addCornerPoint(const pcl::PointXYZ corner);
			void addCornerPoint(const Eigen::Vector3f corner);
			/*! \brief Sets cloud with corner points of the Wall and returns the number of set corners in cloud
			*/
			pcl::PointCloud<pcl::PointXYZ> &getCornerPoints();
			int coordinatesCount();
			bool isValidWall() const;

			void setIsValid(bool isValid);


		private:
			r3d::prims::Polygon m_shape;
			bool m_isValid;
		};
	}
}
#endif __WALL__