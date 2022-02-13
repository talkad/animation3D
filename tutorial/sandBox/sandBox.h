#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	void move_snake();
	void calc_all_weights();
	Eigen::VectorXd create_weight_vec(double w1, double w1_ind, double w2, double w2_ind);
	void next_vertices_position();
	void add_weights();


	double calc_related_distance(int i);
	Eigen::Vector3d target_pose;
	int scale;
	int joints_num;
	std::vector<Eigen::Vector3d> skinnedSkeleton;
	std::vector<Movable> Joints;
	unsigned char direction = ' ';
	unsigned char previous_direction = 'r';

	typedef
		std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
		RotationList;
	// W - weights matrix
	// BE - Edges between joints
	// C - joints positions
	// P - parents
	// M - weights per vertex per joint matrix
	// U - new vertices position after skinning

	Eigen::MatrixXd V, W, C, U, M;
	Eigen::MatrixXi F, BE;
	Eigen::VectorXi P;
	RotationList vQ;
	std::vector<Eigen::Vector3d> vT;
	//std::vector<RotationList > poses; // rotation of joints
	//int score = 0;

private:	
	void Animate();
};

