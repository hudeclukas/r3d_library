set(R3D_R3DGui "R3DGui")

add_executable(${R3D_R3DGui}
	gui.h
	gui.cpp
	guiDlg.h
	guiDlg.cpp
	)
	
target_include_directories(${R3D_R3DGui} PUBLIC
	${R3DLibraryDir}
	${DataHandlingDir}
	${ExporterDir}
	${R3DGuiDir}
	${VisualizationDir}
	)

target_link_libraries(${R3D_R3DGui} LINK_PRIVATE
	R3D_DataHandling
	R3D_3DLibrary
	R3D_Exporter
	R3D_Visualization
	)