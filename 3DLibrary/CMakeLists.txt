set(R3D_3DLibrary "3DLibrary")

add_library(${R3D_3DLibrary} SHARED
	Method.h
	Method.cpp
	)
	
target_include_directories(${R3D_3DLibrary} PUBLIC
	${R3D_DataHandling}
	)

target_link_libraries(${R3D_3DLibrary} LINK_PRIVATE
	R3D_DataHandling
	)