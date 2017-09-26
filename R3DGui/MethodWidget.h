#pragma once
#include <ClosedSpace.h>

class TiXmlNode;

namespace r3d {
	class MethodWidget
	{
	public:
		MethodWidget();
		virtual ~MethodWidget();

		virtual std::string getWidgetName() const;

		void setDataName(std::string name)
		{
			m_dataSet.setName(name);
		}

		void setData(pcl::PointCloud<pcl::PointXYZ> &data)
		{
			m_dataSet.setData(data);
			m_hasNewDataset = true;
		}
		void setData(pcl::PointCloud<pcl::RGB> &data)
		{
			m_dataSet.setData(data);
			m_hasNewDataset = true;
		}
		void setData(pcl::PointCloud<pcl::Normal> &data)
		{
			m_dataSet.setData(data);
			m_hasNewDataset = true;
		}

		r3d::space::ClosedSpace& getDataSet()
		{
			return m_dataSet;
		}

		/*! \fn virtual r3d::space::ClosedSpace<pcl::PointXYZ> &getReconstructedDataset()
		*  \brief Virtual in case that some reconstruction method needed to override and change the outcome
		*  \return r3d::space::ClosedSpace<pcl::PointXYZ> dataset with all segmentated structures and primitives
		*/
		virtual r3d::space::ClosedSpace &getReconstructedDataset();

		/*! \brief virtual function to be implemented by child classes and reconstruction methods
		*/
		virtual bool calculate() = 0;

		/*! \brief virtual function to be implemented by child classes and reconstruction methods
		*   as debug/test visualisation
		*/
		virtual void visualise(pcl::visualization::PCLVisualizer* viewer) = 0;
		
		virtual void setConfiguration(TiXmlNode* param);

	protected:

		TiXmlNode* m_configNode;

		bool m_hasNewDataset = false;
		space::ClosedSpace m_dataSet;
		std::string m_widgetName;
	};
}
