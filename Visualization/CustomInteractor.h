#pragma once
#include <pcl/visualization/interactor_style.h>
//#include <vtkInteractorStyleTerrain.h>

//, public vtkInteractorStyleTerrain
namespace r3d {
	namespace visualization {
		class CustomInteractor : public pcl::visualization::PCLVisualizerInteractorStyle
		{
		public:
			CustomInteractor();
			/*!
			*  IO event handling methods
			*/
			void CustomInteractor::OnMouseMove();
			void CustomInteractor::OnLeftButtonDown();
			void CustomInteractor::OnLeftButtonUp();

			void CustomInteractor::OnChar();
			void CustomInteractor::OnKeyPress();
			void CustomInteractor::OnKeyRelease();

			/*!
			 *  Camera movement methods - reused implementation from VTK's vtkInteractorStyleTerrain
			 */
			void CustomInteractor::Rotate();
			void CustomInteractor::KeyboardPanLeft();
			void CustomInteractor::KeyboardPanRight();

			void CustomInteractor::MoveForwards();
			void CustomInteractor::MoveBackwards();
			void CustomInteractor::Init();
		};
	}
}
