#pragma once

#include <tinyxml.h>


namespace r3d {
	/*! \mainpage 3DRecon
	*
	* \section intro_sec Introduction
	*
	* The main goal of this project is to design and implement methods for 3D Scene Reconstruction,
	* automatic generation of simple semantic 3D data description obtained from stereo reconstruction
	* and the Hierarchical 3D Stitching of surface patches.
	* The outcome of the implementation will be a working prototype for 3D data reconstruction.
	*
	* \section manual_sec Manual
	*
	* \subsection step1 Step 1: Include dataset with pointcloud
	* \subsection step2 Step 2: Include dataset with pointcloud
	* \subsection step2(optional) Step 2 (Optional): View original data with our visualizer
	* \subsection step3 Step 3: Choose one of our methods and click on compute
	* \subsection step4 Step 4: Now you can visualize method output
	*
	*
	* Dependencies:
	* PCL
	* OpenCV
	* MFC
	* GoogleTest
	* sgCoreSDK
	* TinyXML
	*
	* Webpage: http://team05-15.studenti.fiit.stuba.sk/
	*/
	namespace mtds {
		class Method
		{
		public:
			/*! \class Method Method.h "Method.h"
				*  \brief This is an abstract parent class to all working methods.
				*  m_methodName should be set to dedicated method name
				*
				*   \author Lukas Hudec
				* /
			Method();
			/*!
			*  Constructor with parameter of Reconstruction name
			*  @param [in] name reconstruction method name
			*  @param [in] config method name - xml element in configuration file
			*/
			Method(std::string name, const char * config);

			virtual ~Method();
			
			const char * _config_name;
			const std::string _methodName;

			virtual void setConfiguration(TiXmlNode* param) = 0;

		};
	}
}
