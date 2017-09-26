#include "stdafx.h"
#include "InputDataParser.h"


r3d::parser::InputDataParser::InputDataParser()
{
}


r3d::parser::InputDataParser::~InputDataParser()
{
}

pcl::PointCloud<pcl::PointXYZ>* r3d::parser::InputDataParser::parseXYZCsv(const std::string source)
{
	std::ifstream input(source.c_str());
	if (!input.is_open())
	{
		PCL_ERROR("Error loading file %s \n", source.c_str());
		return NULL;
	}
	/*!< find separator */
	std::string line;
	getline(input, line);
	unsigned __int64 pos = line.find_first_of(":,;/");
	std::string separator = line.substr(pos, 1);
	if (separator.empty())
	{
		PCL_ERROR("No suitable separator found\n");
		return NULL;
	}
	pcl::PointCloud<pcl::PointXYZ> *output(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<std::string> coordinatesVector;
	while (input.eof() != 1)
	{
		getline(input, line); /*!< Always get the line -> so while advancesr */
		//cout << line << endl;

		if (line.size() == 0) /*!< void lines could be accientaly present, in case continue */
		{
			continue;
		}

		typedef boost::tokenizer<boost::char_separator<char>> tokenizer; /*!< Tokenize the line */
		boost::char_separator<char> sep(separator.c_str());
		tokenizer tokens(line, sep);

		/*!< Assign tokens to a string vector */
		coordinatesVector.clear();
		coordinatesVector.assign(tokens.begin(), tokens.end());

		pcl::PointXYZ point;
		point.x = atof(coordinatesVector[1].c_str());
		point.y = atof(coordinatesVector[2].c_str());
		point.z = atof(coordinatesVector[3].c_str());

		output->push_back(point); /*!< Add the point to the cloud */
	}
	PCL_DEBUG("File %s loaded", source.c_str());
	input.close();
	return output;
}

