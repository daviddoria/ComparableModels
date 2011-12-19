#include <iostream>
#include "ComparableModels.h"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

void TestMatchingPoints(const ModelFile &World, const ModelFile &Model, const std::vector<double> &ScannerPos);
ModelFile MakeSimpleWorld();
		
		using std::endl;
		
int main( int argc, char* argv[] )
{
	
	std::string WorldFilename, ModelFilename;
	
	std::vector<double> ScannerPos;
	
	//parse arguments
	po::options_description desc("Allowed options");
	desc.add_options()
			("help", "Help message.")
			("model", po::value<string>(&ModelFilename), "Model file")
			("scan", po::value<string>(&WorldFilename), "World file")
			;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm); //assign the variables (if they were specified)
	
	ModelFile World;
	
	ModelFile Model;
	
	//if no files have been provided, run a simple synthetic test
	if(!vm.count("model") || !vm.count("scan"))
	{
		std::cout << "Generating synthetic scene..." << std::endl;
		std::vector<vgl_point_3d<double> > Points;
		Points.push_back(vgl_point_3d<double> (0.0, 0.0, 0.0));
		Points.push_back(vgl_point_3d<double> (10.0, 0.0, 0.0));
		Points.push_back(vgl_point_3d<double> (0.0, 10.0, 0.0));
		std:vector<std::vector<unsigned int> > VertexLists;
		std::vector<unsigned int> VertexList;
		VertexList.push_back(0);
		VertexList.push_back(1);
		VertexList.push_back(2);
		
		VertexLists.push_back(VertexList);
		
		Model.setCoords(Points);
		Model.setVertexLists(VertexLists);
		std::vector<vgl_point_3d<double> > ScenePoints;
		ScenePoints.push_back(vgl_point_3d<double> (1.0, 1.0, 1.0));
		ScenePoints.push_back(vgl_point_3d<double> (1.0, 2.0, 1.0));
		ScenePoints.push_back(vgl_point_3d<double> (2.0, 1.0, 1.0));
		ScenePoints.push_back(vgl_point_3d<double> (3.0, 1.5, 2.5));
		
		World.setCoords(ScenePoints);
		World.setScannerLocation(vgl_point_3d<double>(1.0, 1.0, -1.0));
	
	}
	else
	{
		World.Read(WorldFilename);
		Model.Read(ModelFilename);
	}
	
		
	TestMatchingPoints(World, Model, ScannerPos);
	return 0;
}


void TestMatchingPoints(const ModelFile &World, const ModelFile &Model, const std::vector<double> &ScannerPos)
{
	ComparableModels Comparable(World, Model);
	Comparable.Init();
	
	Comparable.Model.Write("Model.vtp");
	Comparable.World.Write("World.vtp");
	Comparable.WriteMatchingPoints("WorldComparable.vtp", "ModelComparable.vtp");
	double mismatch = 0.4;
	Comparable.WriteConnectingLines("Lines.vtp", mismatch);
}

ModelFile MakeSimpleWorld(void)
{
	//need 4 points to determine a frustrum
	vector<OrientedPoint> WorldPoints;
	double s = .02;
	WorldPoints.push_back(OrientedPoint(vgl_point_3d<double> (s, s, 0.0)));
	WorldPoints.push_back(OrientedPoint(vgl_point_3d<double> (s, -s, 0.0)));
	WorldPoints.push_back(OrientedPoint(vgl_point_3d<double> (-s, s, 0.0)));
	WorldPoints.push_back(OrientedPoint(vgl_point_3d<double> (-s, -s, 0.0)));
	
	ModelFile World;
	World.setPoints(WorldPoints);
	World.Init();
	//World.Write("TestWorld.vtp");
	
	return World;
}