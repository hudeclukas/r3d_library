#pragma once
#include "tinyxml.h"

class PGrabber
{
public:
	PGrabber();
	~PGrabber();

	const static char* getParameter(TiXmlNode* param, const char* name);
};

