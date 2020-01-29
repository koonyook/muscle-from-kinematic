#include "Runner.h"
#include "Environment.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
#include <iostream>
#include <fstream>
using namespace MASS;
using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;

Runner::
Runner(Environment* env)
	:mEnv(env),mSimulating(false),mMuscleNNLoaded(false)
{
	mNNLoaded = false;

	mm = p::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = p::import("sys");
	
	p::str module_dir = (std::string(MASS_ROOT_DIR)+"/python").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);
	p::exec("import torch",mns);
	p::exec("import torch.nn as nn",mns);
	p::exec("import torch.optim as optim",mns);
	p::exec("import torch.nn.functional as F",mns);
	p::exec("import torchvision.transforms as T",mns);
	p::exec("import numpy as np",mns);
	p::exec("from Model import *",mns);
}
Runner::
Runner(Environment* env,const std::string& nn_path)
	:Runner(env)
{
	mNNLoaded = true;

	boost::python::str str = ("num_state = "+std::to_string(mEnv->GetNumState())).c_str();
	p::exec(str,mns);
	str = ("num_action = "+std::to_string(mEnv->GetNumAction())).c_str();
	p::exec(str,mns);

	nn_module = p::eval("SimulationNN(num_state,num_action)",mns);

	p::object load = nn_module.attr("load");
	load(nn_path);
}
Runner::
Runner(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path)
	:Runner(env,nn_path)
{
	mMuscleNNLoaded = true;
	boost::python::str str = ("num_total_muscle_related_dofs = "+std::to_string(mEnv->GetNumTotalRelatedDofs())).c_str();
	p::exec(str,mns);
	str = ("num_actions = "+std::to_string(mEnv->GetNumAction())).c_str();
	p::exec(str,mns);
	str = ("num_muscles = "+std::to_string(mEnv->GetCharacter()->GetMuscles().size())).c_str();
	p::exec(str,mns);

	muscle_nn_module = p::eval("MuscleNN(num_total_muscle_related_dofs,num_actions,num_muscles)",mns);

	p::object load = muscle_nn_module.attr("load");
	load(muscle_nn_path);
}

void Runner::extractMuscleGroupMean(double timeLength,std::vector<int> muscleIndexSeeds)
{
	//seeds are just selected muscles, they will be mapped to muscleGroupIndex
	//assume that they do not repeat the same group twice
	std::vector<int> targetGroupIndexList;
	if(muscleIndexSeeds.size()>0)
 	{
		for(int i : muscleIndexSeeds)
		{
			targetGroupIndexList.push_back(mEnv->groupIndexFromMuscleIndex[i]);
		}
	}
	else	//if there is no seed, just export all the group
	{
		for(int i=0;i<mEnv->muscleGroupN;i++)
		{
			targetGroupIndexList.push_back(i);
		}
 	}

	Reset(false);
	int totalControlStep=int(timeLength*mEnv->GetControlHz());

	//std::list<Eigen::VectorXd> aList;
	//std::vector<Muscle*> muscles=mEnv->GetCharacter()->GetMuscles();
	//int nMuscle = muscles.size();
	

	int sim_per_control = mEnv->GetSimulationHz()/mEnv->GetControlHz();
	int sim_per_activation = 2;

	int nRow = totalControlStep*sim_per_control/sim_per_activation;
	int nCol = targetGroupIndexList.size();	

	double act[nRow][nCol];
	int rowCount=0;
	for(int frame=0;frame<totalControlStep;frame++)
	{
		Eigen::VectorXd action;
		if(mNNLoaded)
			action = GetActionFromNN();
		else
			action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
		mEnv->SetAction(action);

		if(mEnv->GetUseMuscle())
		{
			
			for(int i=0;i<sim_per_control;i+=sim_per_activation)	//10 round of muscle update
			{
				Eigen::VectorXd mt = mEnv->GetMuscleTorques();
				Eigen::VectorXd activation=GetActivationFromNN(mt);
				Eigen::VectorXd groupActivation = mEnv->convertActivationToGroupActivation(activation);
				//std::cout << activation(0) << "\n====\n";	//can compile
				//std::cout << activation.rows() << "\n====\n";
				//aList.push_back(activation);
				//ans[rowCount][0]=frame;
				for(int j=0;j<targetGroupIndexList.size();j++)
					act[rowCount][j]=groupActivation(targetGroupIndexList[j]);
				rowCount++;

				mEnv->SetActivationLevels(activation);
				for(int j=0;j<sim_per_activation;j++)
					mEnv->Step();
			}	
		}
		else
		{
			for(int i=0;i<sim_per_control;i++)
				mEnv->Step();	
		}
		double wTime=mEnv->GetWorldTime();
		std::cout << "Time: " << wTime << "\tCtrl Frame No: " << int( std::round(wTime*mEnv->GetControlHz())) << "\n";	//TESTED
	}

	//print aList to a csv file
	std::ofstream myfile;
  	myfile.open ("groupActivation.csv");
  	myfile << "frame No., ";
  
	//first row is frameNo & muscle name
	for(int groupIndex : targetGroupIndexList)
	{
		myfile << mEnv->groupNameFromGroupIndex[groupIndex] +", ";
	}	
	myfile << "\n";
	//data directly from ans[][]
	for(int i=0;i<nRow;i++)
	{
		myfile << i/(sim_per_control/sim_per_activation) <<", ";
		for(int j=0;j<targetGroupIndexList.size();j++)
		{
			myfile << act[i][j] <<", ";
		}
		myfile << "\n";
	}
	myfile.close();
}

void
Runner::
extractActivation(double timeLength)
{
	Reset(false);
	int totalControlStep=int(timeLength*mEnv->GetControlHz());

	//std::list<Eigen::VectorXd> aList;
	std::vector<Muscle*> muscles=mEnv->GetCharacter()->GetMuscles();
	int nMuscle = muscles.size();
	

	int sim_per_control = mEnv->GetSimulationHz()/mEnv->GetControlHz();
	int sim_per_activation = 2;

	int nRow = totalControlStep*sim_per_control/sim_per_activation;
	int nCol = nMuscle;	

	double act[nRow][nCol];
	int rowCount=0;
	for(int frame=0;frame<totalControlStep;frame++)
	{
		Eigen::VectorXd action;
		if(mNNLoaded)
			action = GetActionFromNN();
		else
			action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
		mEnv->SetAction(action);

		if(mEnv->GetUseMuscle())
		{
			
			for(int i=0;i<sim_per_control;i+=sim_per_activation)	//10 round of muscle update
			{
				Eigen::VectorXd mt = mEnv->GetMuscleTorques();
				Eigen::VectorXd activation=GetActivationFromNN(mt);
				//std::cout << activation(0) << "\n====\n";	//can compile
				//std::cout << activation.rows() << "\n====\n";
				//aList.push_back(activation);
				//ans[rowCount][0]=frame;
				for(int j=0;j<nMuscle;j++)
					act[rowCount][j]=activation(j);
				rowCount++;

				mEnv->SetActivationLevels(activation);
				for(int j=0;j<sim_per_activation;j++)
					mEnv->Step();
			}	
		}
		else
		{
			for(int i=0;i<sim_per_control;i++)
				mEnv->Step();	
		}
		double wTime=mEnv->GetWorldTime();
		std::cout << "Time: " << wTime << "\tCtrl Frame No: " << int( std::round(wTime*mEnv->GetControlHz())) << "\n";	//TESTED
	}

	//print aList to a csv file
	std::ofstream myfile;
  	myfile.open ("activation.csv");
  	myfile << "frame No., ";
  
	//first row is frameNo & muscle name
	for(int j=0;j<nMuscle;j++)
	{
		myfile << muscles[j]->name +", ";
	}	
	myfile << "\n";
	//data directly from ans[][]
	for(int i=0;i<nRow;i++)
	{
		myfile << i/(sim_per_control/sim_per_activation) <<", ";
		for(int j=0;j<nMuscle;j++)
		{
			myfile << act[i][j] <<", ";
		}
		myfile << "\n";
	}
	myfile.close();

}

void
Runner::
Step()
{	
	int sim_per_control = mEnv->GetSimulationHz()/mEnv->GetControlHz();
	Eigen::VectorXd action;
	if(mNNLoaded)
		action = GetActionFromNN();
	else
		action = Eigen::VectorXd::Zero(mEnv->GetNumAction());
	mEnv->SetAction(action);

	if(mEnv->GetUseMuscle())
	{
		int sim_per_activation = 2;
		for(int i=0;i<sim_per_control;i+=sim_per_activation){
			Eigen::VectorXd mt = mEnv->GetMuscleTorques();
			mEnv->SetActivationLevels(GetActivationFromNN(mt));
			for(int j=0;j<sim_per_activation;j++)
				mEnv->Step();
		}	
	}
	else
	{
		for(int i=0;i<sim_per_control;i++)
			mEnv->Step();	
	}
	double wTime=mEnv->GetWorldTime();
	std::cout << "Time: " << wTime << "\tFrame No: " << int( std::round(wTime*mEnv->GetControlHz())) << "\n";	//TESTED
}
void
Runner::
Reset(bool RSI)
{
	mEnv->Reset(RSI);
}

np::ndarray toNumPyArray(const Eigen::VectorXd& vec)
{
	int n = vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0;i<n;i++)
	{
		dest[i] = vec[i];
	}

	return array;
}


Eigen::VectorXd
Runner::
GetActionFromNN()
{
	p::object get_action;
	get_action= nn_module.attr("get_action");
	Eigen::VectorXd state = mEnv->GetState();
	p::tuple shape = p::make_tuple(state.rows());
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray state_np = np::empty(shape,dtype);
	
	float* dest = reinterpret_cast<float*>(state_np.get_data());
	for(int i =0;i<state.rows();i++)
		dest[i] = state[i];
	
	p::object temp = get_action(state_np);
	np::ndarray action_np = np::from_object(temp);

	float* srcs = reinterpret_cast<float*>(action_np.get_data());

	Eigen::VectorXd action(mEnv->GetNumAction());
	for(int i=0;i<action.rows();i++)
		action[i] = srcs[i];

	return action;
}

Eigen::VectorXd
Runner::
GetActivationFromNN(const Eigen::VectorXd& mt)
{
	if(!mMuscleNNLoaded)
	{
		mEnv->GetDesiredTorques();
		return Eigen::VectorXd::Zero(mEnv->GetCharacter()->GetMuscles().size());
	}
	p::object get_activation = muscle_nn_module.attr("get_activation");
	Eigen::VectorXd dt = mEnv->GetDesiredTorques();
	np::ndarray mt_np = toNumPyArray(mt);
	np::ndarray dt_np = toNumPyArray(dt);

	p::object temp = get_activation(mt_np,dt_np);
	np::ndarray activation_np = np::from_object(temp);

	Eigen::VectorXd activation(mEnv->GetCharacter()->GetMuscles().size());
	float* srcs = reinterpret_cast<float*>(activation_np.get_data());
	for(int i=0;i<activation.rows();i++)
		activation[i] = srcs[i];

	return activation;
}
