#include <iostream>
#include <iomanip>
#include "CustomInteractor.h"
#include "vtkInteractorStyleTerrain.h"
#include "vtkRenderWindowInteractor.h"

#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCallbackCommand.h"
#include "vtkExtractEdges.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkPolyData.h"
#include "vtkPolyDataMapper.h"

#include "vtkSphereSource.h"


r3d::visualization::CustomInteractor::CustomInteractor()
{
	std::cout << "calling custominteractor constructor.\n";
	/*
	double fp[3], pos[3];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);
	
	pos[0] = -1;
	pos[1] = -1;
	pos[2] = -1;
	camera->SetPosition(pos);

	fp[0] = 0;
	fp[1] = 0;
	fp[2] = -0.95;
	camera->SetFocalPoint(fp);
	*/
	//this->Init();
}

void r3d::visualization::CustomInteractor::OnLeftButtonDown()
{
	switch (this->State)
	{
	case VTKIS_ROTATE:
		this->EndRotate();
		if (this->Interactor)
		{
			this->ReleaseFocus();
		}
		break;
	default:
		this->FindPokedRenderer(this->Interactor->GetEventPosition()[0],
			this->Interactor->GetEventPosition()[1]);
		if (this->CurrentRenderer == NULL)
		{
			return;
		}

		this->GrabFocus(this->EventCallbackCommand);
		this->StartRotate();
		break;
	}
	
}
void r3d::visualization::CustomInteractor::OnLeftButtonUp()
{
	
	switch (this->State)
	{
	case VTKIS_ROTATE:
		this->EndRotate();
		if (this->Interactor)
		{
			this->ReleaseFocus();
		}
		break;
	}
	
}

void r3d::visualization::CustomInteractor::OnKeyRelease()
{
vtkRenderWindowInteractor *rwi = this->Interactor;

switch (rwi->GetKeyCode())
{

case 'a':
switch (this->State)
{
case VTKIS_PAN:
this->EndPan();
if (this->Interactor)
{
this->ReleaseFocus();
}
break;
}
break;

default:
this->Superclass::OnKeyRelease();
break;
}
}

void r3d::visualization::CustomInteractor::OnChar()
{

}

void r3d::visualization::CustomInteractor::OnKeyPress()
{
	vtkRenderWindowInteractor *rwi = this->Interactor;

	int x = this->Interactor->GetEventPosition()[0];
	int y = this->Interactor->GetEventPosition()[1];

	double pos[3], fp[3];
	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();

	//this->StartPan();
	switch (rwi->GetKeyCode())
	{
	case 'a':
		this->KeyboardPanLeft();
		this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
		//this->Pan();
		break;
	case 'd':
		this->KeyboardPanRight();
		this->InvokeEvent(vtkCommand::InteractionEvent, NULL);
		break;
	case 'w':
		this->MoveForwards();
		break;
	case 's':
		this->MoveBackwards();
		break;
	case 'r':
		this->Init();
		break;
	/*
	case 'x':
		this->Rot();
		break;
	case 'y':
		this->Pos();
		break;
	*/
	default:
		this->Superclass::OnChar();
		break;
	}
}

void r3d::visualization::CustomInteractor::MoveForwards()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	// Get the vector of motion

	double fp[3], fpNew[3], fpNew2[3], pos[3];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);
	/*
	pos[0] = -1;
	pos[1] = -1;
	pos[2] = -1;
	*/
	fpNew[0] = fp[0] + (fp[0] - pos[0]);
	fpNew[1] = fp[1] + (fp[1] - pos[1]);
	fpNew[2] = fp[2] + (fp[2] - pos[2]);
	/*
	camera->SetPosition(fp);
	camera->SetFocalPoint(fpNew);
	*/
	fpNew2[0] = fpNew[0] + (fpNew[0] - fp[0]);
	fpNew2[1] = fpNew[1] + (fpNew[1] - fp[1]);
	fpNew2[2] = fpNew[2] + (fpNew[2] - fp[2]);

	camera->SetPosition(fpNew);
	camera->SetFocalPoint(fpNew2);

	if (rwi->GetLightFollowCamera())
	{
		this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
	}

	rwi->Render();
}


void r3d::visualization::CustomInteractor::MoveBackwards()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;
	double fp[3], posNew[3], pos[3];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);
	
	posNew[0] = pos[0] - (fp[0] - pos[0]);
	posNew[1] = pos[1] - (fp[1] - pos[1]);
	posNew[2] = pos[2] - (fp[2] - pos[2]);

	camera->SetPosition(posNew);
	camera->SetFocalPoint(pos);

	if (rwi->GetLightFollowCamera())
	{
		this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
	}

	rwi->Render();
}

void r3d::visualization::CustomInteractor::KeyboardPanLeft()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	/*!< Get the vector of motion */
	double fp[3], focalPoint[3], pos[3], v[3], p1[4], p2[4];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);

	this->ComputeWorldToDisplay(fp[0], fp[1], fp[2],
		focalPoint);

	this->ComputeDisplayToWorld(rwi->GetEventPosition()[0]+100,
		rwi->GetEventPosition()[1],
		focalPoint[2],
		p1);

	this->ComputeDisplayToWorld(rwi->GetLastEventPosition()[0],
		rwi->GetLastEventPosition()[1],
		focalPoint[2],
		p2);
	
	for (int i = 0; i<3; i++)
	{
		v[i] = p2[i] - p1[i];
		v[1] = 0;
			pos[i] += v[i];
			fp[i] += v[i];
	}

	camera->SetPosition(pos);
	camera->SetFocalPoint(fp);

	if (rwi->GetLightFollowCamera())
	{
		this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
	}


	rwi->Render();
}

void r3d::visualization::CustomInteractor::KeyboardPanRight()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	// Get the vector of motion

	double fp[3], focalPoint[3], pos[3], v[3], p1[4], p2[4];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);


	this->ComputeWorldToDisplay(fp[0], fp[1], fp[2],
		focalPoint);

	this->ComputeDisplayToWorld(rwi->GetEventPosition()[0] -100,
		rwi->GetEventPosition()[1],
		focalPoint[2],
		p1);

	this->ComputeDisplayToWorld(rwi->GetLastEventPosition()[0],
		rwi->GetLastEventPosition()[1],
		focalPoint[2],
		p2);
	
	for (int i = 0; i<3; i++)
	{
		v[i] = p2[i] - p1[i];
		v[1] = 0;
		//v[i] = 0.01;
		//if (i != 1) {
		pos[i] += v[i];
		fp[i] += v[i];
		//}
	}

	camera->SetPosition(pos);
	camera->SetFocalPoint(fp);

	if (rwi->GetLightFollowCamera())
	{
		this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
	}

	rwi->Render();	
}


void r3d::visualization::CustomInteractor::Init()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	double fp[3], pos[3];

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->GetPosition(pos);
	camera->GetFocalPoint(fp);

	pos[0] = 0;
	pos[1] = 0;
	pos[2] = -1;
	camera->SetPosition(pos);

	fp[0] = 0;
	fp[1] = 0;
	fp[2] = -0.75;
	camera->SetFocalPoint(fp);

	if (rwi->GetLightFollowCamera())
	{
		this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
	}

	rwi->Render();
}
void r3d::visualization::CustomInteractor::Rotate()
{
	if (this->CurrentRenderer == NULL)
	{
		return;
	}

	vtkRenderWindowInteractor *rwi = this->Interactor;

	int dx = -(rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0]);
	int dy = -(rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1]);

	int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

	double a = dx / static_cast<double>(size[0]) * 180.0;
	double e = dy / static_cast<double>(size[1]) * 180.0;

	if (rwi->GetShiftKey())
	{
		if (abs(dx) >= abs(dy))
		{
			e = 0.0;
		}
		else
		{
			a = 0.0;
		}
	}

	vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
	camera->Azimuth(a);

	double dop[3], vup[3];

	camera->GetDirectionOfProjection(dop);
	vtkMath::Normalize(dop);
	camera->GetViewUp(vup);
	vtkMath::Normalize(vup);

	double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(dop, vup)));
	if ((angle + e) > 179.0 ||
		(angle + e) < 1.0)
	{
		e = 0.0;
	}

	camera->Elevation(e);

	if (this->AutoAdjustCameraClippingRange)
	{
		this->CurrentRenderer->ResetCameraClippingRange();
	}

	rwi->Render();
}

void r3d::visualization::CustomInteractor::OnMouseMove()
{

	int x = this->Interactor->GetEventPosition()[0];
	int y = this->Interactor->GetEventPosition()[1];

	switch (this->State)
	{
	case VTKIS_ROTATE:

		this->FindPokedRenderer(x, y);
		this->Rotate();
		this->InvokeEvent(vtkCommand::InteractionEvent, NULL);

		break;
	case VTKIS_PAN:

		this->FindPokedRenderer(x, y);
		this->Pan();
		this->InvokeEvent(vtkCommand::InteractionEvent, NULL);

		break;

	case VTKIS_DOLLY:

		this->FindPokedRenderer(x, y);
		this->Dolly();
		this->InvokeEvent(vtkCommand::InteractionEvent, NULL);

		break;
	}


}
