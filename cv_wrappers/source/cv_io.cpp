
#include "cv_io.h"

//Function will write points to file, it is able to write 2d and 3d points
//Input:	file_name - file to save to
//			pnts - vector of points
int write_2d_points_to_file(std::string filename, imagePointVector pnts){
	std::ofstream output_file(filename);
	for (auto iter = pnts.begin(); iter != pnts.end(); ++iter){

		output_file << iter->x << " " << iter->y << std::endl;
	}

	output_file.close();

	return 1;
}
int write_d_points_to_file(std::string filename, std::vector<int> id, imagePointVector pnts){
	std::ofstream output_file(filename);
	for (auto iter = pnts.begin(); iter != pnts.end(); ++iter){

		output_file << iter->x << " " << iter->y << std::endl;
	}

	output_file.close();
	return 1;
}

//Function will write points to file, it is able to write 3d points
//Input:	file_name - file to save to
//			pnts - vector of points
int write_3d_points_to_file(std::string filename, worldPointVector pnts){

	std::ofstream output_file(filename);
	for (auto iter = pnts.begin(); iter != pnts.end(); ++iter){
		output_file << iter->x << " " << iter->y << " " << iter->z << std::endl;
	}

	output_file.close();
	return 1;
}

int write_3d_points_to_file(std::string filename, float time, int counter, worldPointVector pnts){
	std::string counter_string;

	if (counter < 10){
		counter_string = "000"+std::to_string(counter);
	}
	else if (counter < 100){
		counter_string ="00"+ std::to_string(counter) ;
	}
	else if (counter < 1000){
		counter_string = "0"+std::to_string(counter);

	}
	std::ofstream output_file("../matlab/AFEM_OUTPUT/" + filename + counter_string + ".txt");
	output_file << time << std::endl;
	for (auto iter = pnts.begin(); iter != pnts.end(); ++iter){
		output_file << iter->x << " " << iter->y << " " << iter->z << std::endl;
	}

	output_file.close();
	return 1;
}
int write_3d_points_to_file(std::string filename, std::vector<int> id, worldPointVector pnts){
	if (id.size() != pnts.size()){
		std::cout << "Error: vector of ids and pnts must be same size . " << std::endl;
		return -1;
	}

	std::ofstream output_file(filename);

	int _c = 0;
	for (auto iter = pnts.begin(); iter != pnts.end(); ++iter){
		output_file<< std::to_string(_c) << iter->x << " " << iter->y << " " << iter->z << std::endl;
		_c++;
	}

	output_file.close();
	return 1;
}


