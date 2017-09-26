#ifndef __GUIDLG__
#define __GUIDLG__

#include "tinyxml.h"
#include "MethodWidget.h"

#pragma once


// CguiDlg dialog
class CguiDlg : public CDialogEx
{
// Construction
public:
	CguiDlg(CWnd* pParent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GUI_DIALOG };
#endif

protected:
	 void DoDataExchange(CDataExchange* pDX) override;	// DDX/DDV support


// Implementation
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton4();
	pcl::visualization::PCLVisualizer* viewer;
	TiXmlNode *methodConfig;
private:

	int iRet;
	CStringList csFullPathname, csOnlyName;
	//std::string* cesty_suborov;
	std::vector<std::string> filePaths;
	std::string xmlPath;
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>* m_originalData;
	pcl::visualization::PCLVisualizerInteractorStyle * style;

	// Reconstruction methods
	/*! \var std::vector<r3d::mtds::Reconstruction *> m_reconstructionMethods
	*   \brief vector containing all reconstruction methods to be shown and used
	*   \brief best to fill in constructor
	*/
	std::vector<r3d::MethodWidget *> m_reconstructionMethods;
	r3d::MethodWidget* m_selectedMethod;

	bool bXYZ = false, bRGB = false, bNormal = false;

public:
//	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnBnClickedButton2();
//	afx_msg void OnBnClickedzobraz();
//	afx_msg void OnBnClickedstop();
	afx_msg void OnBnClickedfile();
	afx_msg void OnBnClickedN();
//	afx_msg void OnBnClickedbt();
//	afx_msg void OnBnClickedstop();
	afx_msg void OnBnClickedbt();
	afx_msg void OnBnClickedbutton();
	afx_msg void OnBnClickedcalculationbutton();
	afx_msg void OnBnClickedcalculation();
	afx_msg void OnBnClickedvisualizationbutton();
	afx_msg void OnBnClickedxmlfile();
//	afx_msg void OnBnClickedButtonSave();
	afx_msg void OnBnClickedSvNorm();
	afx_msg void OnBnClickedXm();
	afx_msg void OnBnClickedXmlFile();
	afx_msg void OnBnClickedOpenConf();
//	afx_msg void OnBnClickedButton();
	afx_msg void OnClickedRmButton();
	afx_msg void OnBnClickedRM();
	afx_msg void OnBnClickedButton();
	afx_msg void OnBnClickedBt();
};

#endif __GUIDLG__