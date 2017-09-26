#include "stdafx.h"
#include "MethodWidget.h"


r3d::MethodWidget::MethodWidget()
	: m_configNode(nullptr)
{
}


r3d::MethodWidget::~MethodWidget()
{
}

std::string r3d::MethodWidget::getWidgetName() const
{
	return m_widgetName;
}

r3d::space::ClosedSpace& r3d::MethodWidget::getReconstructedDataset()
{
	return m_dataSet;
}

void r3d::MethodWidget::setConfiguration(TiXmlNode* param)
{
	m_configNode = param;
}