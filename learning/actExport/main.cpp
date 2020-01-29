#include "Runner.h"
#include "Environment.h"
#include "DARTHelper.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
#include <ctype.h>
namespace p = boost::python;
namespace np = boost::python::numpy;
int main(int argc,char** argv)
{
	MASS::Environment* env = new MASS::Environment();

	if(argc==1)
	{
		std::cout<<"Provide Metadata.txt"<<std::endl;
		return 0;
	}
	env->Initialize(std::string(argv[1]),true);
	
	//grouping must be done with the same prefix, the number might skip unexpectedly for example L_Transversus_Abdominis has none, 2, and 4
	std::vector<int> v;	//selected muscleIndex seed
	//v.push_back(236);
	//v.push_back(88);
	//v.push_back(208);
	//v.push_back(196);
	//v.push_back(26);
	//v.push_back(202);
	//v.push_back(182);
	//v.push_back(254);
	//v.push_back(246);
	//v.push_back(250);
	//v.push_back(260);
	//v.push_back(170);
	//v.push_back(102);
	//v.push_back(92);
	//v.push_back(102);
	//v.push_back(272);

	//adding the right side
	int m=v.size();
	for(int i=0;i<m;i++)
	{
		v.push_back(v[i]+1);
	}

	Py_Initialize();
	np::initialize();
	//glutInit(&argc, argv);
	MASS::Runner* runner;
	if(argc == 2)
	{
		runner = new MASS::Runner(env);
	}
	else
	{
		if(env->GetUseMuscle())
		{
			if(argc!=4){
				std::cout<<"Please provide two networks"<<std::endl;
				return 0;
			}
			runner = new MASS::Runner(env,argv[2],argv[3]);	
		}
		else
		{
			if(argc!=3)
			{
				std::cout<<"Please provide the network"<<std::endl;
				return 0;
			}
			runner = new MASS::Runner(env,argv[2]);
		}
	}
	
	double extractionTimeLength=env->GetCharacter()->mBVH->GetMaxTime()-1/env->GetControlHz();
	runner->extractMuscleGroupMean(extractionTimeLength,v);

}
