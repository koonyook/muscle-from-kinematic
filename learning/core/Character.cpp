#include "Character.h"
#include "BVH.h"
#include "DARTHelper.h"
#include "Muscle.h"
#include <tinyxml.h>
using namespace dart;
using namespace dart::dynamics;
using namespace MASS;
Character::
Character()
	:mSkeleton(nullptr),mBVH(nullptr),w_mTc(Eigen::Isometry3d::Identity())
{

}

void
Character::
LoadSkeleton(const std::string& path,bool create_obj)
{
	mSkeleton = BuildFromFile(path,create_obj);
	std::map<std::string,std::string> bvh_map;
	TiXmlDocument doc;
	doc.LoadFile(path);
	TiXmlElement *skel_elem = doc.FirstChildElement("Skeleton");

	for(TiXmlElement* node = skel_elem->FirstChildElement("Node");node != nullptr;node = node->NextSiblingElement("Node"))
	{
		if(node->Attribute("endeffector")!=nullptr)
		{
			std::string ee =node->Attribute("endeffector");
			if(ee == "True")
			{
				mEndEffectors.push_back(mSkeleton->getBodyNode(std::string(node->Attribute("name"))));
			}
		}
		TiXmlElement* joint_elem = node->FirstChildElement("Joint");
		if(joint_elem->Attribute("bvh")!=nullptr)
		{
			bvh_map.insert(std::make_pair(node->Attribute("name"),joint_elem->Attribute("bvh")));
		}
	}
	
	mBVH = new BVH(mSkeleton,bvh_map);

	templateRootHeight=mSkeleton->getBodyNode(0)->getParentJoint()->getJointProperties().mT_ParentBodyToJoint.translation()[1];

}
void
Character::
LoadMuscles(const std::string& path)
{
	TiXmlDocument doc;
	if(!doc.LoadFile(path)){
		std::cout << "Can't open muscle file : " << path << std::endl;
		return;
	}

	TiXmlElement *muscledoc = doc.FirstChildElement("Muscle");
	for(TiXmlElement* unit = muscledoc->FirstChildElement("Unit");unit!=nullptr;unit = unit->NextSiblingElement("Unit"))
	{
		std::string name = unit->Attribute("name");
		double f0 = std::stod(unit->Attribute("f0"));
		double lm = std::stod(unit->Attribute("lm"));
		double lt = std::stod(unit->Attribute("lt"));
		double pa = std::stod(unit->Attribute("pen_angle"));
		double lmax = std::stod(unit->Attribute("lmax"));
		mMuscles.push_back(new Muscle(name,f0,lm,lt,pa,lmax));
		int num_waypoints = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
			num_waypoints++;
		int i = 0;
		for(TiXmlElement* waypoint = unit->FirstChildElement("Waypoint");waypoint!=nullptr;waypoint = waypoint->NextSiblingElement("Waypoint"))	
		{
			std::string body = waypoint->Attribute("body");
			Eigen::Vector3d glob_pos = string_to_vector3d(waypoint->Attribute("p"));
			if(i==0||i==num_waypoints-1)
			// if(true)
				mMuscles.back()->AddAnchor(mSkeleton->getBodyNode(body),glob_pos);
			else
				mMuscles.back()->AddAnchor(mSkeleton,mSkeleton->getBodyNode(body),glob_pos,2);

			i++;
		}
	}
	

}
void
Character::
LoadBVH(const std::string& path,bool cyclic)
{
	if(mBVH ==nullptr){
		std::cout<<"Initialize BVH class first"<<std::endl;
		return;
	}
	mBVH->Parse(path,cyclic);
}
void
Character::
Reset()
{
	w_mTc.setIdentity();
	w_mTc.translation()[1] -= templateRootHeight;
}
void
Character::
SetPDParameters(double kp, double kv)
{
	int dof = mSkeleton->getNumDofs();
	mKp = Eigen::VectorXd::Constant(dof,kp);	
	mKv = Eigen::VectorXd::Constant(dof,kv);	
}
Eigen::VectorXd
Character::
GetSPDForces(const Eigen::VectorXd& p_desired)
{
	Eigen::VectorXd q = mSkeleton->getPositions();
	Eigen::VectorXd dq = mSkeleton->getVelocities();
	double dt = mSkeleton->getTimeStep();
	// Eigen::MatrixXd M_inv = mSkeleton->getInvMassMatrix();
	Eigen::MatrixXd M_inv = (mSkeleton->getMassMatrix() + Eigen::MatrixXd(dt*mKv.asDiagonal())).inverse();

	Eigen::VectorXd qdqdt = q + dq*dt;	//projected pose in a stupid linear way.

	Eigen::VectorXd p_diff = -mKp.cwiseProduct(mSkeleton->getPositionDifferences(qdqdt,p_desired));
	Eigen::VectorXd v_diff = -mKv.cwiseProduct(dq);
	Eigen::VectorXd ddq = M_inv*(-mSkeleton->getCoriolisAndGravityForces()+p_diff+v_diff+mSkeleton->getConstraintForces());	//getConstraintForces for joint limit constraint

	Eigen::VectorXd tau = p_diff + v_diff - dt*mKv.cwiseProduct(ddq);

	tau.head<6>().setZero();	//remove force/torque related to root translation and orientation
								//because none of those 6 DOF can be directly controlled by muscle.
								//the agent has to touch environment (like ground) to have such effect.
	return tau;
}

Eigen::Isometry3d
ProjectToGround(Eigen::Isometry3d a)
{
	Eigen::Isometry3d b=a;
	b.translation()[1]=0;
	//Y axis must point upward
	b.linear()(0,1)=0;
	b.linear()(1,1)=1;
	b.linear()(2,1)=0;
	//Z axis must point parallel to the ground
	auto zAxis=b.linear().col(2);
	zAxis[1]=0;
	b.linear().col(2)=zAxis.normalized();
	//X axis from cross product
	b.linear().col(0)=b.linear().col(1).cross(b.linear().col(2));

	return b;
}

Eigen::VectorXd
Character::
GetTargetPositions(double t,double dt)
{
	Eigen::VectorXd p = mBVH->GetMotion(t);	//angle of every joint directly from BVH file (rotVec, CHECKED)
	Eigen::Isometry3d mTc_Tcurrent = dart::dynamics::FreeJoint::convertToTransform(p.head<6>());	//current root 6D relative to the origin of mocap (CHECKED)
	Eigen::Isometry3d w_Tcurrent = w_mTc * mTc_Tcurrent;	//mTc is reference point on the ground
	Eigen::Vector6d p_head = dart::dynamics::FreeJoint::convertToPositions(w_Tcurrent);
	p.head<6>() = p_head;
	//generalized coordinate of the root is rotVec & translation.
	if(mBVH->IsCyclic())
	{
		double t_mod = std::fmod(t, mBVH->GetMaxTime());
		t_mod = t_mod/mBVH->GetMaxTime();	// 0 < t_mod < 1

		double r = 0.95;
		if(t_mod>r)	//near the end (last 5% of the circle)
		{
			//adjust the height so that the continuity at the wraping point is preserved
			double ratio = (t_mod-r)/(1.0-r);	//ratio=0 at r, ramp up to ratio=1 at 1
			double delta = mBVH->Get_w_T0().translation()[1] - mBVH->Get_w_T1().translation()[1];
			delta *= ratio;
			p[4] += delta;	//adjust root height
		}
		

		double tdt_mod = std::fmod(t+dt, mBVH->GetMaxTime());
		if(tdt_mod<dt)
		{	
			//stack on current state of the character (from the simulation)
			Eigen::VectorXd q = mSkeleton->getPositions();
			Eigen::Isometry3d w_Tstate = dart::dynamics::FreeJoint::convertToTransform(q.head<6>());
			Eigen::Isometry3d w_TstateProjected = ProjectToGround(w_Tstate);
			Eigen::Isometry3d refSpot_T0Projected = ProjectToGround(mBVH->Get_w_T0());
			Eigen::Isometry3d w_refSpot = w_TstateProjected * refSpot_T0Projected.inverse();
			w_mTc = ProjectToGround( w_refSpot );
			
			w_mTc.translation()[1] -= templateRootHeight;
		}	
	}
	

	return p;
}

Eigen::Isometry3d 
Character::
GetStem()
{
	Eigen::Isometry3d a=w_mTc;
	a.translation()[1]+=templateRootHeight;
	return a;
}

std::pair<Eigen::VectorXd,Eigen::VectorXd>
Character::
GetTargetPosAndVel(double t,double dt)
{
	Eigen::VectorXd p = this->GetTargetPositions(t,dt);		//in generalized coordinate (joint angle)
	Eigen::Isometry3d tmp = w_mTc;
	Eigen::VectorXd p1 = this->GetTargetPositions(t+dt,dt);	//this will trigger the shift in w_mTc one frame before we need
	w_mTc=tmp;	//bring back because the base cannot shift twice

	return std::make_pair(p,mSkeleton->getPositionDifferences(p1, p)/dt);
}