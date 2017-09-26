#ifndef __PRIMITIVES__
#define __PRIMITIVES__

#pragma once

namespace r3d {
	namespace prims {
		class Primitives
		{
		public:
			Primitives();
			virtual ~Primitives();

			virtual void addCoordinate(const pcl::PointXYZ point) = 0;
			inline pcl::PointCloud<pcl::PointXYZ> &getCoordinates()
			{
				return m_coordinates;
			}
			inline bool empty() const
			{
				return m_coordinates.size() == 0;
			}
		protected:
			pcl::PointCloud<pcl::PointXYZ> m_coordinates;
		};
	}
}
#endif __PRIMITIVES__