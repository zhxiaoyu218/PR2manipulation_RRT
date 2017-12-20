#include <vector>
#include <cmath>
#include <iostream>

using namespace std;
using namespace OpenRAVE;

typedef std::vector<double> Configuration;

class RRTNode
{
  Configuration _config;
  int _parentID;
  
  public:
		RRTNode(const Configuration &config)
		{
    		_config = config;
    		_parentID = -1;	
		}
	    void setParent(int parent)
		{
			_parentID = parent;
		}

		int getParent()
   		{
    		return _parentID;
   		}	

	  	Configuration getConfig()
	  	{
  			return _config;
  		}
};


class NodeTree
{
  EnvironmentBasePtr env;
  vector<RRTNode> _nodes;
  int _TotNode;
  int _configSize;
  bool _FindGoal;
  vector<double>  _GoalConfig;
  float _StepSize;
  vector<double> weight;

public:
  NodeTree(EnvironmentBasePtr envir,vector<double> goal,Configuration config, float step)
  {
      
      RRTNode currentNode(config);
      _nodes.push_back(currentNode);
      _configSize = currentNode.getConfig().size();
      _FindGoal = false;
      _GoalConfig = goal;
      _StepSize = step;
      env = envir;
      
    weight = config;
     weight[0] = 3.17104446;
      weight[1] = 2.75674493;
       weight[2] = 1.7894779;
        weight[3] = 2.23250068;
         weight[4] = 1.42903194;
          weight[5] = 0.80901263;
           weight[6] = 0.59308423;
    weight[4] = 0.0;
    weight[6] = 0.0;
    weight[0] =15.0;
    weight[1] = 12.0;

  }
  bool FindGoal()
  {
  	return _FindGoal;
  }
  void add(Configuration config)
  {
        int nnID = NearestNode(config);
      
		if (!withinStepSize(nnID,config))
		{
			int TotSteps = distance(_nodes[nnID].getConfig(),config)/_StepSize;
	    	float TotStepsPos = (TotSteps*_StepSize)/distance(_nodes[nnID].getConfig(),config); 
			
			for (int step = 1; step <= TotSteps; step++)
	    	{
		        Configuration newConfig( _configSize);
		        for (int i = 0; i <  _configSize; ++i)
		        	newConfig[i] = _nodes[nnID].getConfig()[i] + ((config[i]-_nodes[nnID].getConfig()[i])*TotStepsPos*step)/TotSteps;

		        if (!CollideCheck(newConfig))
		        {

		        	RRTNode newNode( newConfig);
		        	newNode.setParent(nnID);
		        	_nodes.push_back(newNode);
	    	        nnID = _nodes.size()-1;  
	    	    }
	    	    else
	    	    {
	    	    	return;		    	 
	    	    }
	    	}
		}
			
		if (!CollideCheck(config))
	    {

	    	RRTNode newNode(config);
	    	newNode.setParent(nnID);
	    	_nodes.push_back(newNode);
	    	if (GoalCheck(newNode.getConfig()))
	    	{
						cout<< "goal Found!!!!"<< endl;
						_FindGoal =true;
						return;
	    	}

	    }
	    else
	    {
	    	return ;
		}
      return ;
  }
  
  
	int NearestNode(Configuration sampleConfig)
	{
			float dist = 100000.0;
			int index = -1;

	    for (int i = 0; i < _nodes.size(); i++)
			{
        if (distance(_nodes[i].getConfig() , sampleConfig) < dist)
				{
					if (dist <= 0.01)
					{
						index = i;
						break;
					}
					else
					{
						dist =distance(_nodes[i].getConfig() , sampleConfig);
						index = i;
					}
				}

			}
			return index;
		} 
		
  double distance(const Configuration &a , const Configuration &b)
	{
      		double dist = 0.0;
          int langth = a.size();
      		for (int i = 0; i < langth; ++i)
      			dist += weight[i]*(a[i] - b[i]) * (a[i] - b[i]);
      
      		return sqrt(dist);
	}
 
  bool CollideCheck(const Configuration &sampleConfig)
  {
        	env->GetRobot("PR2")->SetActiveDOFValues(sampleConfig);
		    if ( env->CheckCollision(env->GetRobot("PR2")) || env->GetRobot("PR2")->CheckSelfCollision() )
		      	return true;
		    else
		       	return false;
  }

  bool GoalCheck(const Configuration &sampleConfig)
	{
	  int length = sampleConfig.size();
	  for (int i = 0; i < length; i++)
	  {
	  	if( (_GoalConfig[i] != sampleConfig[i]) )
	  	{
	  		return false;
	  	}
	  }
	  return true;
	}
	
	bool withinStepSize(int nnID,const Configuration &config)
	{
		
		if ( distance( _nodes[nnID].getConfig(),config) > _StepSize)
			return false;
		else
			return true;
	}
	vector<Configuration> GivePath()
	{
		int ID =  _nodes.size()-1;
		vector<Configuration> path;
		// vector<handleBase> myhandle;

		while(ID >=0)
		{
			path.push_back( _nodes[ID].getConfig());
			ID = _nodes[ID].getParent();
		}
		reverse(path.begin(),path.end());
		return path;
		
	}

};




