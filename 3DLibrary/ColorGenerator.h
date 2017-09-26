#pragma once

#ifndef _COLORGENERATOR_
#define _COLORGENERATOR_

namespace r3d {
	class ColorGenerator {
	public:
		static ColorGenerator& getInstance() {
			static ColorGenerator instance;
			return instance;
		}

		unsigned char* getNextColor();

		static void paintPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

	private:
		ColorGenerator()
		{
			srand(std::time(nullptr));
		};

		int m_offset = 71; /* 89 */
		int m_color = 0;
		unsigned char r = 255, g = 255, b = 255;

	};
}

#endif _COLORGENERATOR_