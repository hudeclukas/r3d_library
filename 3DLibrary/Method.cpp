#include "stdafx.h"
#include "Method.h"


r3d::mtds::Method::Method(std::string name, const char * config)
	: _methodName(name)
{
	_config_name = config;
}

r3d::mtds::Method::~Method()
{
}

