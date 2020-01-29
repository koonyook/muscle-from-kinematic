#ifndef __MASS_WINDOW_H__
#define __MASS_WINDOW_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

namespace p = boost::python;
namespace np = boost::python::numpy;

namespace MASS
{
class Environment;
class Muscle;
class Runner
{
public:
	Runner(Environment* env);
	Runner(Environment* env,const std::string& nn_path);
	Runner(Environment* env,const std::string& nn_path,const std::string& muscle_nn_path);

	void extractActivation(double timeLength);
	void extractMuscleGroupMean(double timeLength,std::vector<int> muscleIndexSeeds);

private:

	void Step();
	void Reset(bool RSI=true);

	Eigen::VectorXd GetActionFromNN();
	Eigen::VectorXd GetActivationFromNN(const Eigen::VectorXd& mt);

	p::object mm,mns,sys_module,nn_module,muscle_nn_module;


	Environment* mEnv;
	bool mSimulating;
	bool mNNLoaded;
	bool mMuscleNNLoaded;
};
};


#endif