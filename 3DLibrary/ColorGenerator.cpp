#include "stdafx.h"
#include "ColorGenerator.h"


void r3d::ColorGenerator::paintPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
	pcl::RGB randomColor;
	randomColor.g = rand() % 256;
	randomColor.b = rand() % 256;
	for each (pcl::PointXYZRGB outlier in cloud)
	{
		outlier.r = 255.0;
		outlier.g = randomColor.g;
		outlier.b = randomColor.b;
	}
}

unsigned char* r3d::ColorGenerator::getNextColor() {
	auto resultColors = new unsigned char[3];
		
	if (m_color > 1020) {
		m_color -= 1020;
	}

	if (m_color >= 0 && m_color <= 510) {
		if (m_color <= 255) {
			r = 255;
			g = m_color;
			b = 0;
		}
		else if (m_color > 255) {
			r = 255 - (m_color - 255);
			g = 255;
			b = 0;
		}
	}
	else if (m_color > 510 && m_color <= 1020) {
		if (m_color <= 765) {
			r = 0;
			g = 255;
			b = 255 - (765 - m_color);
		}
		else if (m_color > 765) {
			r = 0;
			g = 255 - (m_color - 765);
			b = 255;
		}
	}

	resultColors[0] = r;
	resultColors[1] = g;
	resultColors[2] = b;

	m_color += m_offset;
	return resultColors;
}
