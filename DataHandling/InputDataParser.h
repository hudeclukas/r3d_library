#ifndef __INPUTDATAPARSER__
#define __INPUTDATAPARSER__

#pragma once
namespace r3d {
	namespace parser {
		/*! \brief Data parser, whatever data type from ASCII to pointcloud XYZ coordinate system
		*   \author Lukas Hudec
		*/
		class InputDataParser
		{
		public:
			InputDataParser();
			~InputDataParser();
			/*! /brief loads and parses CSV file containing XYZ data in columns 1=X, 2=Y, 3=Z (zero based)
			*   @param [in] source file name to be loaded and parsed
			*/
			static pcl::PointCloud<pcl::PointXYZ>* parseXYZCsv(const std::string source);
		};
	}
}

#endif __INPUTDATAPARSER__