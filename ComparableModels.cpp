#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdlib.h>
#include <assert.h>

#include <ModelFile/ModelFile.h>

#include <vnl/vnl_double_3.h>

#include <vgl/vgl_box_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_distance.h>

#include<vtkSmartPointer.h>
#include<vtkXMLPolyDataWriter.h>
#include<vtkPolyData.h>
#include<vtkCellData.h>
#include<vtkPoints.h>
#include<vtkCellArray.h>
#include<vtkUnsignedIntArray.h>
#include<vtkLine.h>

#include <Geometry/Color.h>
#include <Geometry/Transformation.h>
#include <Geometry/Ray.h>
#include <Geometry/Geometry.h>
#include <Geometry/Helpers.h>

#include <Tools/Tools.h>

#include <KDTree/KDTree.h>

#include "ComparableModels.h"

#include <omp.h>

#include <boost/progress.hpp>

//constructor

/*
ComparableModels::ComparableModels(const ModelFile &WorldInput, const ModelFile &ModelInput, const LidarScanner &ScannerIn)
{
	World = WorldInput;
	Model = ModelInput;
	Scanner = ScannerIn;

	//CreateUsingOpenGL();
	CreateUsingOctree();
}
*/

ComparableModels::ComparableModels(const ModelFile &WorldInput, const ModelFile &ModelInput)
{
	World = WorldInput;
	Model = ModelInput;
	Valid_ = false;
	ShowProgress_ = true;
}


void ComparableModels::Init()
{
	if(!Model.Octree_.Built_)
		Model.BuildOctree();

	this->Scanner = LidarScanner(true);
	vgl_point_3d<double> scannerloc;
	bool IsScan = World.getScannerLocation(scannerloc);
	if(!IsScan)
	{
		std::cout << "Error: The scene, " << World.getName() << ", does not have ScannerLocation annotation!" << std::endl;
	}
	Scanner.ScanParams.setLocation(scannerloc);

	std::clog << "Scanner Location: " << Scanner.ScanParams.getLocation() << std::endl;
	
	//CreateUsingOpenGL();
	CreateUsingOctree();
	
//	Model.Octree_.Delete();
	Valid_ = true;
	
}


//////////////// Accessors /////////////
unsigned int ComparableModels::GoodPoints() const
{
	return Tools::CountOccurances(PointTypes, GOOD);
}

unsigned int ComparableModels::BadPoints() const
{
	return Tools::CountOccurances(PointTypes, BAD);
}

unsigned int ComparableModels::UninformativePoints() const
{
	return Tools::CountOccurances(PointTypes, UNINFORMATIVE);
}
/*
unsigned int ComparableModels::MissPoints() const
{
	return Tools::CountOccurances(PointTypes, MISS);
}
*/

/*
void ComparableModels::CreateOpenGL()
{
	std::clog << "main" << std::endl;

	//SDL_Surface *screen = InitSDL(Width, Height, "Test Window");
	InitSDL(Width, Height, "Test Window");

	ModelFile Model;

	for(unsigned int i = 0; i < 1000; i++)
	{
		display(Model);
	}
	
	SDL_Quit();
}
*/

/*
void display(const ModelFile &Model)
{
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable(GL_DEPTH_TEST);

	Model.LookAt(vgl_point_3d<double>(0.0, 0.0, 0.0));

	Model.DrawPoints();
	
	SDL_GL_SwapBuffers();
}
*/

void ComparableModels::CreateUsingOctree()
{
	/* 
	Determines the matching points, but nothing about their relative distance.
	*/
	
	/*
	Outputs:
	UsedWorldPoints, MatchingModelPoints: The points in these two files are correspondences
	*/
	
	assert(World.NumPoints() >= 4); //there must be enough points to determine a frustrum
	
	Model.setPointIndices();
	World.setPointIndices();

	ModelNumber = 0;

	//cull the world points by seeing which world points are in the visual hull (visual frustrum) of the model
	vgl_box_3d<double> ModelBox = Model.GetBoundingBox();
	std::vector<vgl_point_3d<double> > WorldOPCoords = GetOPCoords(World.getPoints());
	
	std::vector<unsigned int> WorldIndicesInsideFrustrum = geom::IndicesInsideFrustrum(ModelBox, WorldOPCoords, Scanner.ScanParams.getLocation());
	
	this->MissPoints = 0;
	unsigned int ValidModel = 0;
	
	unsigned int NumFrustrumPoints = WorldIndicesInsideFrustrum.size();
	std::clog << "There are " << NumFrustrumPoints << " points inside the viewing frustrum." << std::endl;
	
	boost::progress_display* show_progress = NULL;
	if(ShowProgress_)
	{
		//std::clog << "Comparable models showing progress." << std::endl;
		//std::clog << "Setup progressbar with " << NumFrustrumPoints << " points." << std::endl;
		show_progress = new boost::progress_display(NumFrustrumPoints);
	}
	else
	{
		//std::clog << "Comparable models not showing progress." << std::endl;
	}
	//#pragma omp parallel for shared(ModelPoints, WorldPoints, InvalidModel, ValidModel) private(WorldPoint, dir, LP, bIntersect, ModelPoint)
	std::vector<unsigned int> TempUsedWorldPoints(NumFrustrumPoints);
	std::vector<OrientedPoint> TempMatchingModelPoints(NumFrustrumPoints);
	std::vector<bool> Used(NumFrustrumPoints, false);
#pragma omp parallel for
	for(unsigned int i = 0; i < NumFrustrumPoints; i++)
	{
		if(ShowProgress_)
			++(*show_progress);//this should be atomic
		
		vgl_point_3d<double> WorldPoint;
		vgl_vector_3d<double> dir;
		LidarPoint LP;
		bool bIntersect;
		
		WorldPoint = World.getCoord(WorldIndicesInsideFrustrum[i]);
		
		dir = WorldPoint - Scanner.ScanParams.getLocation();
		
		bIntersect = Scanner.AcquirePoint(Model, dir, LP);
				
		if(!bIntersect)
		{
			this->MissPoints++; //this should be atomic
			continue;
		}
		else
		{
			if(!LP.getValid())
			{
				std::cout << "The intersection is not valid!" << std::endl;
			}
			if(LP.getCoord() == Scanner.ScanParams.getLocation())
			{
				std::cout << "Intersection is at the scanner?!" << std::endl;
			}
			ValidModel++; //this should be atomic
			TempUsedWorldPoints[i] = WorldIndicesInsideFrustrum[i]; //save world point index
			TempMatchingModelPoints[i] = OrientedPoint(LP.getCoord()); //save model point (could be a point not originally on the model if triangles were intersected)
			Used[i] = true;
		}
	}
	
	std::clog << "Miss points: " << MissPoints << " ValidModel: " << ValidModel << std::endl;
	
	MatchingModelPoints.clear();
	UsedWorldPoints.clear();

	//go through TempMatchingModelPoints and TempUsedWorldPoints and put valid points into the non-temp versions

	unsigned int usedcount = 0;
	for(unsigned int i = 0; i < NumFrustrumPoints; i++)
	{
		if(Used[i] == true)
		{
			usedcount++;
			MatchingModelPoints.push_back(TempMatchingModelPoints[i]);
			UsedWorldPoints.push_back(TempUsedWorldPoints[i]);
		}
	}
	
	//std::clog << "Matching model points: " << MatchingModelPoints.size() << " UsedWorldPoints: " << UsedWorldPoints.size() << std::endl;
	//std::clog << "usedcount: " << usedcount << " ValidModel: " << ValidModel << std::endl;
	std::clog << "There are " << usedcount << " points that match for consistency evaluation." << std::endl;
	this->NumPoints = ValidModel;
}


void ComparableModels::WriteMatchingPoints(const std::string &WorldFilename, const std::string &ModelFilename)
{
	if(UsedWorldPoints.size() == 0)
	{
		std::cout << "There are no comparable points!" << std::endl;
		exit(-1);
	}
	
	//world
	std::vector<OrientedPoint> WorldPoints;
	for(unsigned int i = 0; i < UsedWorldPoints.size(); i++)
	{
		WorldPoints.push_back(World.Points_[UsedWorldPoints[i]]);
	}
	ModelFile WorldOutput;
	WorldOutput.setPoints(WorldPoints);
	WorldOutput.Init();
	WorldOutput.Write(WorldFilename);

	//model
	ModelFile ModelOutput;
	ModelOutput.setPoints(MatchingModelPoints);
	ModelOutput.Init();
	ModelOutput.Write(ModelFilename);

}

void ComparableModels::WriteConnectingLines(const std::string &LineFilename, const double mismatch)
{
	vgl_point_3d<double> ScannerLocation;
	World.getScannerLocation(ScannerLocation);
	
	vtkSmartPointer<vtkPolyData> polydata = vtkPolyData::New();

	vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();

	std::vector<bool> LineType;
	for(unsigned int i = 0; i < UsedWorldPoints.size(); i++)
	{
		vgl_point_3d<double> P0 = World.Points_[UsedWorldPoints[i]].getCoord();
		Points->InsertNextPoint(P0.x(), P0.y(), P0.z());
		
		vgl_point_3d<double> P1 = MatchingModelPoints[i].getCoord();
		Points->InsertNextPoint(P1.x(), P1.y(), P1.z());
		
		double dist_world = vgl_distance(P0, ScannerLocation);
		double dist_model = vgl_distance(P1, ScannerLocation);
		
		if(dist_world > (dist_model + mismatch)) //bad
			LineType.push_back(0); //false = bad
		else
			LineType.push_back(1); //true = good
	}

	polydata->SetPoints(Points);

	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

	unsigned int NumLines = UsedWorldPoints.size();
	for(unsigned int i = 0; i < NumLines; i++)
	{
		vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
		line->GetPointIds()->SetId(0,2*i);
		line->GetPointIds()->SetId(1,2*i + 1);
		lines->InsertNextCell(line);
	}
		
	polydata->SetLines(lines);
	
	//Color lines
	vtkSmartPointer<vtkUnsignedCharArray> LineColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	LineColors->SetNumberOfComponents(3);
	LineColors->SetName("LineColors");
	
	vtkSmartPointer<vtkUnsignedIntArray> LineConsistencies = vtkSmartPointer<vtkUnsignedIntArray>::New();
	LineConsistencies->SetNumberOfComponents(1);
	LineConsistencies->SetName("LineConsistencies");
		
	//create empty unsigned char arrays
	unsigned char r[3];
	unsigned char g[3];
	
	//fill the char arrays
	CharArray(Colors::Red(), r);
	CharArray(Colors::Green(), g);
	for(unsigned int i = 0; i < NumLines; i++)
	{
		if(LineType[i])
		{
			LineColors->InsertNextTupleValue(g);
			LineConsistencies->InsertNextValue(1);
		}
		else
		{
			LineColors->InsertNextTupleValue(r);
			LineConsistencies->InsertNextValue(0);
		}		
	}
	
	polydata->GetCellData()->AddArray(LineColors);
	polydata->GetCellData()->AddArray(LineConsistencies);

	vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	writer->SetFileName(LineFilename.c_str());
	writer->SetInput(polydata);
	writer->Write();	
}

std::ostream& operator<<(std::ostream& output, const ComparableModels &Comparable)
{
	output << "Comparable Models: " << endl << "-------------" << endl;
	output << "Total Points: " << Comparable.TotalPoints() << endl;
	output << "Good Points: " << Comparable.GoodPoints() << endl;
	output << "Bad Points: " << Comparable.BadPoints() << endl;
	output << "Uninformative Points: " << Comparable.UninformativePoints() << endl;
	output << endl;
	return output;
}

ModelFile ComparableModels::ColorOriginalModel(const double allowance)
{
	if(!Valid_)
	{
		std::cout << "Cannot run ColorOriginalModel() before creating the comparable models.\n";
		exit(-1);
	}
	
	std::vector<Color<unsigned char> > ModelColors(Model.NumPoints(), Colors::Grey((unsigned char)122));
	
	if(!ModelTree.isValid())
	{
		ModelTree.CreateTree(Model.getCoords());
	}
	
	for(unsigned int i = 0; i < MatchingModelPoints.size(); i++)
	{
		vgl_point_3d<double> TestPoint = MatchingModelPoints[i].getCoord();
		
		//unsigned int ind = ModelTree.ClosestPointIndex(TestPoint);
		std::vector<unsigned int> indices = ModelTree.IndicesWithinRadius(.05, TestPoint);
		for(unsigned int counter = 0; counter < indices.size(); counter++)
		{
			unsigned int ind = indices[counter];
			if((ModelColors[ind] == Colors::Red()) || (ModelColors[ind] == Colors::Green()))
			{
				//this point has already been assigned a color
				continue;
			}
			//std::cout << "The point closest to " << TestPoint << " is " << ind << " " << Model.getCoord(ind) << std::endl;
			
			double s = (GetMatchingWorldCoord(i) - Scanner.ScanParams.getLocation()).length();
			double m = (GetMatchingModelPoint(i) - Scanner.ScanParams.getLocation()).length();
			
			if(s > (m + allowance))
			{
				ModelColors[ind] = Colors::Red();
			}
			else
			{
				ModelColors[ind] = Colors::Green();
			}
		}	
		
		//std::cout << "Colored point " << ind << ModelColors[ind] << endl;
	}
	
	ModelFile ColoredModel = Model;
	ColoredModel.setColors(ModelColors);
	
	return ColoredModel;
}