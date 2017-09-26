// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently

#pragma once

#include "targetver.h"

#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>

#include <pcl/common/common_headers.h>
#include <pcl/common/file_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/thread/thread.hpp>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>








// TODO: reference additional headers our program requires here
