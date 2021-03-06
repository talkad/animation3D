#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>

class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d& rot);
	void MyScale(Eigen::Vector3d amt);

	void SetCenterOfRotation(Eigen::Vector3d amt);

	void MyTranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);

	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);

	void MyRotate(const Eigen::Quaterniond rot);

	Eigen::Matrix3d GetRotation() const { return Tout.rotation().matrix(); }

	Eigen::Vector3d GetTranslation() const { return (Tout * Tin).matrix().block(0, 3, 3, 1); }

	Eigen::Vector3d Movable::GetCenterOfRotation();

	void SetTranslation(Eigen::Vector3d position);

	void LookAt(Eigen::Vector3d forward);

	virtual ~Movable() {}

	Eigen::Affine3d Tout, Tin;


private:
};