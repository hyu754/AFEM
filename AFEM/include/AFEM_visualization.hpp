
#include "AFEM_simulation.hpp"


#ifndef AFEM_VISUALIZATION_H
#define AFEM_VISUALIZATION_H
namespace AFEM
{
	class Visualization;
}


class AFEM::Visualization
{
public:
	Visualization(AFEM::Simulation *input_sim){
		simulation_class = input_sim;
	}
	/*
	~Visualization();*/


	void Visualization::run_visualization();

private:

	AFEM::Simulation *simulation_class;

};

//Visualization::Visualization()
//{
//}
//
//Visualization::~Visualization()
//{
//}


#endif  //AFEM_VISUALIZATION_H