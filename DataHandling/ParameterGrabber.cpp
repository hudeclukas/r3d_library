#include "stdafx.h"
#include "ParameterGrabber.h"


PGrabber::PGrabber()
{
}


PGrabber::~PGrabber()
{
}


const char* PGrabber::getParameter(TiXmlNode* param, const char* name)
{
	TiXmlElement* element = param->FirstChildElement(name);
	if (element == nullptr)
	{
		throw "Wrong XML config";
	}
	else
	{
		return element->GetText();
	}
}