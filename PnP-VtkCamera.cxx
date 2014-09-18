#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkWorldPointPicker.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkCamera.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv\cv.h>
#include <opencv2\core\core.hpp>
#include <iostream>
#include <vtkMatrix4x4.h>

using namespace cv;
using namespace std;

std::vector<Point3f> objectPoints;
std::vector<Point2f> imagePoints;
vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkMatrix4x4> projectorview = vtkSmartPointer<vtkMatrix4x4>::New();
vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();


// Define interaction style
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static MouseInteractorStyle* New();
    vtkTypeMacro(MouseInteractorStyle, vtkInteractorStyleTrackballCamera);
    
    virtual void OnLeftButtonDown() 
    {
 
      //std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0] << " " << this->Interactor->GetEventPosition()[1] << std::endl;
	  imagePoints.push_back(cv::Point2f(this->Interactor->GetEventPosition()[0], this->Interactor->GetEventPosition()[1]));
      this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0], 
                         this->Interactor->GetEventPosition()[1], 
                         0,  // always zero.
                         this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
      double picked[3];
      this->Interactor->GetPicker()->GetPickPosition(picked);
      //std::cout << "Picked value: " << picked[0] << " " << picked[1] << " " << picked[2] << std::endl;
	  objectPoints.push_back(cv::Point3f(picked[0], picked[1], picked[2]));
	  int pointSize;
	  pointSize = objectPoints.size();
	  //renderer->SetActiveCamera(camera);
	  //camera->SetPosition(0,0,-5);
	  //camera->SetFocalPoint(0,0,0);
	  double cameraPos[3];
	  cv::Mat cameraPosPnP(3,1,CV_64FC1);
	  //camera->GetPosition(cameraPos);
	  //std::cout << "Camera Position is " << cameraPos[0] << "," << cameraPos[1]  <<"," << cameraPos[2] << endl;
	  if (pointSize >= 4)
	  {
		  double cr[2];
	      renderer->GetActiveCamera()->GetClippingRange(cr);
	      int windowsize[2];
		  renderWindow->GetInteractor()->GetSize(windowsize[0], windowsize[1]);
	      projectorview = renderer->GetActiveCamera()->GetProjectionTransformMatrix(windowsize[0]/windowsize[1],cr[0],cr[0]);	 
	      double Fy = projectorview->Element[1][1]*windowsize[1]/2.0;
		  //cout << "Fy is" << Fy << endl;
	      double uc = (double)windowsize[0]/2.0,vc = (double)windowsize[1]/2.0;
		  //cout << "uc is " << uc << "and vc is" << vc << endl;
		  //cout << "Number of correspondences is " << imagePoints.size() << endl;
		  double __k[9] = {Fy, 0, uc, 0, Fy, vc, 0, 0, 1};
		  cv::Mat intrinsicMatrix(3,3,CV_64FC1,__k);
          double __d[4] = {0,0,0,0};
          cv::Mat distCoeffs(1,4,CV_64F,__d);
		  cv::Mat rvec(1,3,CV_64FC1);
		  cv::Mat tvec(1,3,CV_64FC1);
		  cv::solvePnP(objectPoints, imagePoints, intrinsicMatrix, distCoeffs, rvec, tvec, false, CV_EPNP);
		  cv::Mat rot(3,3,CV_64FC1);
		  cv::Rodrigues(rvec, rot);
		  cv::Mat rotT(3,3,CV_64FC1);
		  rotT = rot.t();
		  renderer->SetActiveCamera(camera);
		  cameraPosPnP = rotT*tvec;
		  //cameraPosPnP = -1*cameraPosPnP;
		  renderer->GetActiveCamera()->GetPosition(cameraPos);
		  double focalP[3];
		  camera->SetPosition(0,0,-10);
          camera->SetFocalPoint(0,0,0);
		  renderer->GetActiveCamera()->GetFocalPoint(focalP);
		  std::cout << "**********************************************" << endl;
		  std::cout << "Focal Point is " << focalP[0] << "," << focalP[1]  <<"," << focalP[2] << endl;
	      std::cout << "Camera Position is " << cameraPos[0] << "," << cameraPos[1]  <<"," << cameraPos[2] << endl;
		  std::cout << "PnP Camera Position is" << cameraPosPnP.at<double>(Point(0,0))  << "," << cameraPosPnP.at<double>(Point(0,1))  <<"," << cameraPosPnP.at<double>(Point(0,2)) << endl;
		  cout<<"OpenCV Output" << endl;
	      cout << rot.at<double>(Point(0,0)) << " " << rot.at<double>(Point(0,1)) << " " << rot.at<double>(Point(0,2)) << " " << tvec.at<double>(Point(0,0)) << endl;
          cout << rot.at<double>(Point(1,0)) << " " << rot.at<double>(Point(1,1)) << " " << rot.at<double>(Point(1,2)) << " " << tvec.at<double>(Point(0,1)) << endl;
          cout << rot.at<double>(Point(2,0)) << " " << rot.at<double>(Point(2,1)) << " " << rot.at<double>(Point(2,2)) << " " << tvec.at<double>(Point(0,2)) << endl;
		  std::cout << "**********************************************" << endl;
      // Forward events
         vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	  }
	  
    }
 
};
vtkStandardNewMacro(MouseInteractorStyle);
 
int main(int, char *[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetRadius(2);
  sphereSource->Update();
  
 
  vtkSmartPointer<vtkWorldPointPicker> worldPointPicker = vtkSmartPointer<vtkWorldPointPicker>::New();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =  vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> actor =  vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  // Create a renderer, render window, and interactor
  
  renderer->SetActiveCamera(camera);
  camera->SetPosition(0, 0, -10);
  camera->SetFocalPoint(0,0,0);
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetPicker(worldPointPicker);
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  vtkSmartPointer<MouseInteractorStyle> style =  vtkSmartPointer<MouseInteractorStyle>::New();
  renderWindowInteractor->SetInteractorStyle( style );
 
  // Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
 
  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
 
  return EXIT_SUCCESS;
}