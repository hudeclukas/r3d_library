// guiDlg.cpp : gui implementation file
//

#include "stdafx.h"
//#include "tinyxml.h"

//#include "AnalyticalPlanesReconstruction.h"
#include "ReconstructionWidget.h"
//#include "BirkysNormalSegmentation.h"
#include "SegmentationWidget.h"
//#include "SlicBasedSegmentation.h"
#include "SLICSegmentationWidget.h"
#include "OutliersWidget.h"
//#include "NumeroUnoReconstruction.h"
//#include "CloudRotationTest.h"

#include "CustomInteractor.h"

#include "DataReader.h"

#include "gui.h"
#include "guiDlg.h"

#include "afxdialogex.h"
#include <string>
#include <vector>
#include "Exporter.h"
#include "NoiseRemover.h"
#include <pcl/io/pcd_io.h>

#ifdef _DEBUG
//#define new DEBUG_NEW
#endif

// CAboutDlg dialog used for App About
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

protected:
	void DoDataExchange(CDataExchange* pDX) override;    // DDX/DDV support

	// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CguiDlg dialog
CguiDlg::CguiDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_GUI_DIALOG, pParent), 
	viewer(nullptr), 
	methodConfig(nullptr), 
	iRet(0), 
	m_originalData(nullptr), 
	style(nullptr), 
	m_selectedMethod(nullptr)
{
	// pushback new reconstruction method
	m_reconstructionMethods.push_back(new r3d::ReconstructionWidget());
	m_reconstructionMethods.push_back(new r3d::SegmentationWidget());
	m_reconstructionMethods.push_back(new r3d::SLICSegmentationWidget());
	m_reconstructionMethods.push_back(new r3d::OutliersWidget());
	//m_reconstructionMethods.push_back(new r3d::mtds::BirkysNormalSegmentation());
	//m_reconstructionMethods.push_back(new r3d::mtds::NumeroUnoReconstruction());
	//m_reconstructionMethods.push_back(new r3d::mtds::FurukawaReconstruction());
	//m_reconstructionMethods.push_back(new r3d::mtds::CloudRotationTest());
	//m_reconstructionMethods.push_back(new r3d::mtds::SlicBasedSegmentation());

	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CguiDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CguiDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDOK, &CguiDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON4, &CguiDlg::OnBnClickedButton4)
	ON_CBN_SELCHANGE(IDC_COMBO1, &CguiDlg::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(IDC_BUTTON2, &CguiDlg::OnBnClickedButton2)
	ON_BN_CLICKED(open_file, &CguiDlg::OnBnClickedfile)
	ON_BN_CLICKED(vypocet_N, &CguiDlg::OnBnClickedN)
	ON_BN_CLICKED(stop_bt, &CguiDlg::OnBnClickedbt)
	ON_BN_CLICKED(calculation, &CguiDlg::OnBnClickedcalculation)
	ON_BN_CLICKED(visualizationbutton, &CguiDlg::OnBnClickedvisualizationbutton)
	ON_BN_CLICKED(BUTTON_SV_NORM, &CguiDlg::OnBnClickedSvNorm)
	ON_BN_CLICKED(BTN_OPEN_CONF, &CguiDlg::OnBnClickedOpenConf)
	ON_BN_CLICKED(noise_bt, &CguiDlg::OnBnClickedRM)
	ON_BN_CLICKED(EXP_BT, &CguiDlg::OnBnClickedBt)
END_MESSAGE_MAP()


// CguiDlg message handlers

BOOL CguiDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon


	for (size_t i = 0; i < m_reconstructionMethods.size(); i++)
	{
		static_cast<CComboBox*>(GetDlgItem(IDC_COMBO1))->AddString(CString(m_reconstructionMethods.at(i)->getWidgetName().c_str()));
	}
	// Combo boxes initialization
	static_cast<CComboBox*>(GetDlgItem(IDC_COMBO1))->SetCurSel(0);
	static_cast<CComboBox*>(GetDlgItem(IDC_COMBO1))->SetDroppedWidth(100);
	// VTK warnings turned off
	vtkObject::GlobalWarningDisplayOff();
	xmlPath = "";
	
	//default xml config file inicialization
	TiXmlDocument doc("conf.xml");
	doc.LoadFile();
	TiXmlElement* rootElement = doc.RootElement();
	rootElement = rootElement->FirstChildElement("method");
	methodConfig = rootElement->Clone();

	viewer = nullptr;

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CguiDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CguiDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CguiDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CguiDlg::OnBnClickedOk()
{
	// loading files using multi select dialog
	CFileDialog l_SampleDlg(true, nullptr, nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_ALLOWMULTISELECT, nullptr, nullptr, 0, true);
	iRet = l_SampleDlg.DoModal();
	CString l_strFileName, error_msg;
	std::string str = "no file chosen!";
	error_msg.Format(_T("%S"), str.c_str());
	l_strFileName = l_SampleDlg.GetPathName();

	if (iRet == IDOK) {
		POSITION pos = l_SampleDlg.GetStartPosition();
		while (pos)	{
			CString PathnameStr = l_SampleDlg.GetNextPathName(pos);
			//store absolute file paths
			csFullPathname.AddTail(PathnameStr);
			filePaths.push_back(std::string(CW2A(PathnameStr.GetString())));
		}

		// tu by sa malo nejak vyriesit nacitavanie tych roznych typov a rozdelenie do jednotlivych instancii cloudov
		//m_originalData = r3d::io::loadFiles<pcl::PointXYZ>(filePaths);

		CString l_infoMsg;
		l_infoMsg.Format(_T("%S"), "Files loaded");
		MessageBox(l_infoMsg);

	}
	else
		MessageBox(error_msg);
}


void CguiDlg::OnBnClickedCheck1()
{

}


void CguiDlg::OnCbnSelchangeCombo1()
{

}


void CguiDlg::OnBnClickedButton2()
{
	
}

void CguiDlg::OnBnClickedButton4() {}


void CguiDlg::OnBnClickedfile()
{
	// loading files using multi select dialog
	CFileDialog sampleDlg(true, nullptr, nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT | OFN_ALLOWMULTISELECT, nullptr, nullptr, 0, true);
	//sampleDlg.m_ofn.lpstrFilter = (L"pcd files (*.pcd)\0*.pcd\0ply files (*.ply)\0*.ply\0obj files (*.obj)\0*.obj\0vtk files (*.vtk)\0*.vtk\0csv files (*.csv)\0*.csv\0");
	sampleDlg.m_ofn.lpstrFilter = (L"supported files (*.pcd;*.csv;*.ply;*.obj;*.vtk)\0*.pcd;*.csv;*.ply;*.obj;*.vtk\0");
	iRet = sampleDlg.DoModal();
	CString strFileName, error_msg;
	std::string str = "no file chosen!";
	error_msg.Format(_T("%S"), str.c_str());
	strFileName = sampleDlg.GetPathName();

	if (iRet == IDOK) {
		// reset vector with path to selected files
		filePaths.clear();
		POSITION pos = sampleDlg.GetStartPosition();
		while (pos)	{
			CString PathnameStr = sampleDlg.GetNextPathName(pos);
			//store absolute file paths
			csFullPathname.AddTail(PathnameStr);
			filePaths.push_back(std::string((CW2A(PathnameStr.GetString()))));
		}

		if (m_originalData != nullptr)
		{
			m_originalData->clear();
		}
		// load point cloud files 
		m_originalData = r3d::io::loadFiles<pcl::PointXYZRGBNormal>(filePaths);

		// compute roughness of the loaded dataset
		pcl::PointCloud<pcl::PointXYZ> tmpCloud;
		pcl::copyPointCloud(*m_originalData, tmpCloud);

		CString l_infoMsg;
		l_infoMsg.Format(_T("%S"), "Files loaded");
		MessageBox(l_infoMsg);

		static_cast<CButton*>(GetDlgItem(calculation))->EnableWindow(true);
		static_cast<CButton*>(GetDlgItem(IDC_BUTTON_SAVE))->EnableWindow(true);
		static_cast<CButton*>(GetDlgItem(visualizationbutton))->EnableWindow(true);
		static_cast<CButton*>(GetDlgItem(noise_bt))->EnableWindow(true);
	}
	else
		MessageBox(error_msg);
}


void CguiDlg::OnBnClickedN()	/*!< \todo Must delete this, becuase MFC does not do it for us -.- */
{

}


void CguiDlg::OnBnClickedbt()
{
	if (viewer != nullptr && !viewer->wasStopped())
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->close();
	}
}


void CguiDlg::OnBnClickedcalculation()
{
	int selected = 0;
	CString item;
	std::string label;

	if (csFullPathname.GetSize() > 0) {
		selected = static_cast<CComboBox*>(GetDlgItem(IDC_COMBO1))->GetCurSel();
		static_cast<CComboBox*>(GetDlgItem(IDC_COMBO1))->GetLBText(selected, item);
		label = (CW2A(item.GetString()));

		m_selectedMethod = nullptr;
		for (size_t i = 0; i < m_reconstructionMethods.size(); i++)
		{
			if (m_reconstructionMethods.at(i)->getWidgetName().compare(label) == 0)
			{
				m_selectedMethod = m_reconstructionMethods.at(i);
				break;
			}
		}
		m_selectedMethod->getDataSet().clear();
		pcl::PointCloud<pcl::PointXYZ> workDataXYZ;
		pcl::PointCloud<pcl::RGB> workDataRGB;
		pcl::PointCloud<pcl::Normal> workDataNormal;
		CString msg;
		if (m_selectedMethod!= nullptr) {
			try 
			{
				m_selectedMethod->setConfiguration(methodConfig);
			}
			catch (const char* m) 
			{
				msg.Format(_T("%S"), m);
				MessageBox(msg);
			}
			

			// get file header information
			for (auto file : filePaths)
			{
				pcl::PCLPointCloud2* header = new pcl::PCLPointCloud2;
				std::string type = pcl::getFileExtension(file); 
				if (boost::iequals(type, "ply"))
				{
					pcl::PLYReader plyReader;
					Eigen::Vector4f origin;
					Eigen::Quaternionf orientation;
					int ply_version;
					int data_type;
					unsigned int data_idx;
					plyReader.readHeader(filePaths.at(0), *header, origin, orientation, ply_version, data_type, data_idx, 0);
				}
				else if (boost::iequals(type, "obj"))
				{
					pcl::OBJReader objReader;
					Eigen::Vector4f origin;
					Eigen::Quaternionf orientation;
					int ply_version;
					int data_type;
					unsigned int data_idx;
					objReader.readHeader(file, *header, origin, orientation, ply_version, data_type, data_idx, 0);
				}
				else if (boost::iequals(type, "pcd"))
				{
					pcl::PCDReader pcdReader;
					pcdReader.readHeader(file, *header);
				}
				else
				{
					/*!< \todo For other file types load every data */
					bXYZ = true;
					bNormal = true;
					bRGB = true;
				}

				// find out which fields are contained in the file
				for (int i = 0; i < header->fields.size(); i++)
				{
					if (header->fields[i].name.compare("x") == 0)
					{
						bXYZ = true;
						continue;
					}
					else if (header->fields[i].name.compare("normal_x") == 0)
					{
						bNormal = true;
						continue;
					}
					else if (header->fields[i].name.compare("rgb") == 0) 
					{
						bRGB = true;
						continue;
					}
				}	
			}

			// set data into closedSpace
			if (bXYZ)
			{
				pcl::copyPointCloud(*m_originalData, workDataXYZ);
				m_selectedMethod->setData(workDataXYZ);
			}

			if (bRGB)
			{
				pcl::copyPointCloud(*m_originalData, workDataRGB);
				m_selectedMethod->setData(workDataRGB);
			}

			if (bNormal)
			{
				pcl::copyPointCloud(*m_originalData, workDataNormal);
				m_selectedMethod->setData(workDataNormal);
			}

			if (!bXYZ && !bRGB && !bNormal)
			{
				// no header or non point cloud file is loaded
				pcl::copyPointCloud(*m_originalData, workDataXYZ);
				m_selectedMethod->setData(workDataXYZ);
				pcl::copyPointCloud(*m_originalData, workDataRGB);
				m_selectedMethod->setData(workDataRGB);
				pcl::copyPointCloud(*m_originalData, workDataNormal);
				m_selectedMethod->setData(workDataNormal);
			}
			
			
			bool success = m_selectedMethod->calculate();
			CString l_infoMsg;
			l_infoMsg.Format(_T("%S"), "Reconstruction finished successfully");
			if (success)
			{
				MessageBox(l_infoMsg);
				static_cast<CButton*>(GetDlgItem(IDC_RADIO2))->EnableWindow(true);
				static_cast<CButton*>(GetDlgItem(IDC_RADIO3))->EnableWindow(true);
				static_cast<CButton*>(GetDlgItem(IDC_RADIO4))->EnableWindow(true);
			}
			else
			{
				l_infoMsg.Format(_T("%S"), "Reconstrucion failed");
				MessageBox(l_infoMsg);
			}
		}
	}
	else {}
}


void CguiDlg::OnBnClickedvisualizationbutton()
{
	int argc = 0;
	char* argv[] = { "one", "two", "thre" };
	style = new r3d::visualization::CustomInteractor();
	viewer = new pcl::visualization::PCLVisualizer(argc, argv, "Test Window", style, true);

	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	std::vector<pcl::visualization::Camera> cameras;
	viewer->getCameras(cameras);
	pcl::visualization::Camera cam = cameras[0];
	cam.pos[0] = 0;
	cam.pos[1] = 0;
	cam.pos[2] = -1;
	cam.focal[0] = 0;
	cam.focal[1] = 0;
	cam.focal[2] = -0.75;
	viewer->setCameraParameters(cam, 0);
	viewer->setSize(800, 600);
	// implementation for each radio button
	if (static_cast<CButton*>(GetDlgItem(IDC_RADIO1))->GetCheck()) {	//original data
		//m_selectedMethod->getReconstructedDataset().visualiseOriginalData(viewer);
		// or we could visualise these original data from gui cloud that is actually original data
		pcl::PointCloud<pcl::PointXYZ>::Ptr dataXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*m_originalData, *dataXYZ);
		viewer->addPointCloud<pcl::PointXYZ>(dataXYZ, "originalXYZ");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "originalXYZ");
	}
	if (static_cast<CButton*>(GetDlgItem(IDC_RADIO2))->GetCheck()) {
		m_selectedMethod->visualise(viewer);
	}
	if (static_cast<CButton*>(GetDlgItem(IDC_RADIO3))->GetCheck()) {
		viewer->setBackgroundColor(1.0, 1.0, 1.0);
		auto walls = m_selectedMethod->getReconstructedDataset().getWalls();

		float r, g, b;
		for (auto i = 0; i < walls.size(); i++)
		{
			auto &wall_coordinates = walls[i].getCornerPoints();
			r = (1.0 / 255.0) * float(walls[i].getR());
			g = (1.0 / 255.0) * float(walls[i].getG());
			b = (1.0 / 255.0) * float(walls[i].getB());
			viewer->addPolygon<pcl::PointXYZ>(wall_coordinates.makeShared(), r, g, b, "shape" + std::to_string(i));
			
		}
		viewer->setRepresentationToSurfaceForAllActors();

	}
	if (static_cast<CButton*>(GetDlgItem(IDC_RADIO4))->GetCheck()) {
		m_selectedMethod->visualise(viewer);
		viewer->setRepresentationToSurfaceForAllActors();
	}
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}


void CguiDlg::OnBnClickedSvNorm()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr pcXYZNormal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZ(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*m_originalData, *pcXYZ);
	pcl::PointCloud<pcl::Normal>::Ptr pcNormals(new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(4);
	ne.setInputCloud(pcXYZ);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	ne.setKSearch(100);
	ne.compute(*pcNormals);

	pcl::copyPointCloud(*pcNormals, *m_originalData);
	bNormal = true;

	pcl::copyPointCloud(*pcXYZ, *pcXYZNormal);
	pcl::copyPointCloud(*pcNormals, *pcXYZNormal);

	pcl::io::savePCDFileASCII("PointCloudXYZNormal.pcd", *pcXYZNormal);

	CString l_infoMsg;
	l_infoMsg.Format(_T("%S"), "Normals saved to file.");
	MessageBox(l_infoMsg);
}


void CguiDlg::OnBnClickedXm()
{
	
}



void CguiDlg::OnBnClickedOpenConf()
{
	CFileDialog sampleDlg(true, nullptr, nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, nullptr, nullptr, 0, true);
	sampleDlg.m_ofn.lpstrFilter = (L"xml files (*.xml)\0*.xml\0");
	iRet = sampleDlg.DoModal();
	CString strFileName, error_msg,msg;
	std::string str = "the last choosen or default XML file in use!";
	error_msg.Format(_T("%S"), str.c_str());
	strFileName = sampleDlg.GetPathName();

	if (iRet == IDOK) {
		xmlPath = std::string((CW2A(strFileName.GetString())));
		char* filename = static_cast<char *>(malloc(xmlPath.size() + 1));
		memcpy(filename, xmlPath.c_str(), xmlPath.size() + 1);

		TiXmlDocument doc(filename);
		doc.LoadFile();
		TiXmlElement* rootElement = doc.RootElement();
		rootElement = rootElement->FirstChildElement("method");
		methodConfig = rootElement->Clone();
		
		for each (r3d::MethodWidget *method in m_reconstructionMethods)
		{
			try 
			{
				method->setConfiguration(methodConfig);
			}
			catch (const char* m) 
			{
				msg.Format(_T("%S"), m);
				MessageBox(msg);
			}
		}

		CString infoMsg;
		infoMsg.Format(_T("%S"), "xml file loaded");
		MessageBox(infoMsg);
	}
	else
		MessageBox(error_msg);
}

void CguiDlg::OnBnClickedRM()
{
	CString strFileName, msg;
	r3d::remover::NoiseRemover noiseRemover;
	try 
	{
		noiseRemover.setConfiguration(methodConfig->FirstChildElement("NoiseRemover"));
	}
	catch (const char* m) 
	{
		msg.Format(_T("%S"), m);
		MessageBox(msg);
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
	cloud_filtered = noiseRemover.removeNoise(m_originalData);
	pcl::copyPointCloud(*cloud_filtered, *m_originalData);
	
	LPCTSTR fileSuffix = L".pcd";
	CFileDialog sampleDlg(false, fileSuffix, nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, nullptr, nullptr, 0, true);
	sampleDlg.m_ofn.lpstrFilter = (L"pcd files (*.pcd)\0*.pcd\0");
	iRet = sampleDlg.DoModal();
	
	strFileName = sampleDlg.GetPathName();

	if (iRet == IDOK) {
		CT2CA str(strFileName);
		std::string strName(str);
		char* fName = new char[strName.length() + 1];
		strcpy(fName, strName.c_str());		
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZ>(fName, *cloud_filtered, false);
		delete[] fName;
		msg.Format(_T("%S"), "file saved");
		MessageBox(msg);
	}
	else {
		msg.Format(_T("%S"), "file not saved");
		MessageBox(msg);
	}
}



void CguiDlg::OnBnClickedBt()
{
	/*	export test
	both cornerPoints.pcd and cornerPoints2.pcd contains four points*/
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("0.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
	}


	std::vector<r3d::obj::Wall> walls;
	r3d::obj::Wall wall1(*cloud1);
	walls.push_back(wall1);
	//walls.push_back(wall2);
	r3d::exporter::Exporter e;
	e.addWalls(walls);
	e.writeDXF("test.dxf");
	*/
	
	CString strFileName, msg;

	r3d::exporter::Exporter e;
	if (m_selectedMethod != nullptr) {
		e.addWalls(m_selectedMethod->getDataSet().getWalls());
		//e.addWalls(walls);
		LPCTSTR pripona = L".dxf";
		CFileDialog sampleDlg(false, pripona, nullptr, OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, nullptr, nullptr, 0, true);
		sampleDlg.m_ofn.lpstrFilter = (L"dxf files (*.dxf)\0*.dxf\0");
		iRet = sampleDlg.DoModal();		
		strFileName = sampleDlg.GetPathName();

		if (iRet == IDOK) {
			CT2CA str(strFileName);
			std::string strName(str);
			char* fName = new char[strName.length() + 1];
			strcpy(fName, strName.c_str());

			if (e.writeDXF(fName)) {
				//	export successful
				msg.Format(_T("%S"), "export successful");
				MessageBox(msg);
			}
			else {
				//	export failed
				msg.Format(_T("%S"), "export failed");
				MessageBox(msg);
			}
			delete[] fName;

		}
		else {
			msg.Format(_T("%S"), "export failed");
			MessageBox(msg);
		}
	}
	else {
		msg.Format(_T("%S"), "null pointer error");
		MessageBox(msg);
	}
}

