#include <motion_follower.hpp>

double euclidean_distance (std::shared_ptr<std::vector<double>> v1, std::shared_ptr<std::vector<double>> v2){
	double temp = 0;
	for (short int i=0; i<v1->size(); i++){
		temp += pow((v1->at(i) - v2->at(i)), 2);
	}
	return sqrt(temp);
}