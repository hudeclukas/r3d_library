#ifndef __DATAREADER__
#define __DATAREADER__

#pragma once
#include "InputDataParser.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>

#include <pcl/common/file_io.h>

/*! \namespace r3d
    \brief A namespace.

    A more detailed namespace description.
*/
namespace r3d {
	/*! \namespace io
	* \brief contains functions to work with various data types 
	* input functions loads and converts them to pcl::PointCloud
	* output functions (doesn't exist right now)
	*/
	namespace io {

		/*! \brief Loads data coordinates from certain type of CSV file.
		*  @param [in] fileName file name to load
		*  @return pointcloud of loaded points from file
		*/
		template <typename PointT>
		pcl::PointCloud<PointT> *loadCsvFile(const std::string fileName);

		template <typename PointT>
		/*! \brief Loads mesh from VTK file and converts it to pcl::PointCloud
		*  @param fileName Name of file to read data from 
		*  @return pointcloud of loaded points from file
		*/
		pcl::PointCloud<PointT> *loadVTKMeshFile(const std::string fileName)
		{
			pcl::PolygonMesh mesh;
			if (pcl::io::loadPolygonFileVTK(fileName, mesh) == -1)
			{
				PCL_ERROR("Error loading file %s", fileName);
				return NULL;
			}
			else
			{
				pcl::PointCloud<PointT> *output = new pcl::PointCloud<PointT>();
				pcl::fromPCLPointCloud2(mesh.cloud, *output);
				return output;
			}
		}

		/*! \brief Loads points from PCD file
		*  @param fileName Name of file to read data from
		*  @return pointcloud of loaded points from file
		*/
		template <typename PointT>
		pcl::PointCloud<PointT> *loadPcdFile(const std::string fileName)
		{
			pcl::PointCloud<PointT> *output = new pcl::PointCloud<PointT>; /*!< Detailed description after the member */
			if (pcl::io::loadPCDFile<PointT>(fileName, *output) == -1)
			{
				PCL_ERROR("Error loading file %s", fileName);
				return NULL;
			}
			else
			{
				return output;
			}
		}

		template <typename PointT>
		/*! \brief Loads points from PLY file
		*  @param fileName Name of file to read data from
		*  @return pointcloud of loaded points from file
		*/
		pcl::PointCloud<PointT> *loadPlyFile(const std::string fileName)
		{
			pcl::PointCloud<PointT> *output = new pcl::PointCloud<PointT>;
			if (pcl::io::loadPLYFile<PointT>(fileName, *output) == -1)
			{
				PCL_ERROR("Error loading file %s", fileName);
				return NULL;
			}
			else
			{
				return output;
			}
		}

		template <typename PointT>
		/*! \brief Loads points from OBJ file
		*  @param fileName Name of file to read data from
		*  @return pointcloud of loaded points from file
		*/
		pcl::PointCloud<PointT> *loadObjFile(const std::string fileName)
		{
			pcl::PointCloud<PointT> *output = new pcl::PointCloud<PointT>;
			if (pcl::io::loadOBJFile<PointT>(fileName, *output) == -1)
			{
				PCL_ERROR("Error loading file %s", fileName);
				return NULL;
			}
			else
			{
				return output;
			}
		}

		template <typename PointT>
		/*! An template type. 
		    *  PointT template that contains mainly pcl::PointXYZ and pcl::PointXYZRGB data
		    *  @param files an std::vector<std::string> argument of all files to be reader/loaded.
		    *  @sa loadObjFile(), loadPlyFile(), loadPcdFile(), loadVTKMeshFile() and loadCsvFile()
		    *  @return cloudData
		    */
		pcl::PointCloud<PointT> *loadFiles(const std::vector<std::string> files)
		{
		            //! cloudData pointer.
         			/*! temporary storage for data that is filled from all selected files, at the end returned */
			pcl::PointCloud<PointT> *cloudData = new pcl::PointCloud<PointT>; /*! Point Cloud */

			for (auto file : files)
			{
				std::cout << file << std::endl;
				std::string type = pcl::getFileExtension(file);
				if (boost::iequals(type, "csv"))
				{
					//pcl::PointCloud<PointT>* tmpCloud = m_dataParser.parseXYZCsv(file);
					if (pcl::isSamePointType<PointT, pcl::PointXYZ>() || pcl::isSamePointType<PointT, pcl::PointXYZRGBNormal>())
					{
						*cloudData += *r3d::io::loadCsvFile<PointT>(file);
					}
				}
				else if (boost::iequals(type, "ply"))
				{
					*cloudData += *r3d::io::loadPlyFile<PointT>(file);
				}
				else if (boost::iequals(type, "obj"))
				{
					*cloudData += *r3d::io::loadObjFile<PointT>(file);
				}
				else if (boost::iequals(type, "pcd"))
				{
					*cloudData += *r3d::io::loadPcdFile<PointT>(file);
				}
				else if (boost::iequals(type, "vtk"))
				{
					*cloudData += *r3d::io::loadVTKMeshFile<PointT>(file);
				}
				else
				{
					return nullptr;
				}
			}

			return cloudData;
		}
		
	}
}


#endif __DATAREADER__