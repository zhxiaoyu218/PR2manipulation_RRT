#include <openrave/plugin.h>
#include <openrave/planningutils.h>
#include <boost/bind.hpp>

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

 #include "myRRT.h";


using namespace OpenRAVE;
using namespace std;


std::vector<double> _upper;
std::vector<double> _lower;



class myRRT : public ModuleBase
{

    // vector<double> _StrConfig;
    // vector<double> _GoalConfig;
    float _GoalBias;
    float _StepSize;
    bool _biFlag ;
    int _MaxIteration;

    int _configSize;
    vector<double> _lower ;
    vector<double> _upper ;
    vector<double> _StrConfig;
    vector<double> _GoalConfig;

        vector<Configuration> path;

public:
    myRRT(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("SetStrGoal",boost::bind(&myRRT::SetStrGoal,this,_1,_2),"This is an example command");

        RegisterCommand("SetPara",boost::bind(&myRRT::SetPara,this,_1,_2),"This is an example command");

        RegisterCommand("FindPath",boost::bind(&myRRT::FindPath,this,_1,_2),"find a path to goal");

        RegisterCommand("PathSmooth",boost::bind(&myRRT::PathSmooth,this,_1,_2),"smooth path");
          
    }

    virtual ~myRRT() {}
    
    bool SetStrGoal(std::ostream& sout, std::istream& sinput)
    {
        std::string input;       
        char temp = '1';
        sinput >> input;
        float q;

        if (input == "str")
            while (temp!= ';')
            {
                sinput >> q;
                _StrConfig.push_back(q);
                sinput>>temp;
            }

        sinput >> input;
        temp = '1';
        if (input == "goal")
            while(temp != ';')
            {
                sinput >> q;
                _GoalConfig.push_back(q);
                sinput>>temp;
            }
        // cout<<"_StrConfig is: "; 
        // for (int i = 0; i < _StrConfig.size(); i++)
        // {
        //     cout<< _StrConfig[i]<<",";
        // }
        // cout<<endl;
        // cout<<"_GoalConfig is: ";
        // for (int i = 0; i < _GoalConfig.size(); i++)
        // {
        //     cout<< _GoalConfig[i]<<",";
        // }
        // cout<<endl;
        
        // return true;
    }
    bool SetPara(std::ostream& sout, std::istream& sinput)
    {
        char temp = '1';

        sinput >> _GoalBias;
        sinput >>temp;
        sinput >> _StepSize;
        sinput >> temp;
        sinput >> _biFlag;
        sinput >> temp;
        sinput >> _MaxIteration;
         _MaxIteration = 200;
        cout<<"goalbias is: "<<_GoalBias<<endl;
        cout<<"step: "<<_StepSize<<endl;
        // cout<<"bidirection? : "<<_biFlag<<endl;
        // cout<<"MaxIteration : "<<_MaxIteration<<endl;

        GetEnv()->GetRobot("PR2")->GetActiveDOFLimits(_lower, _upper);
        _lower[4] = _lower[6] = -3.1415926;
        _upper[4] = _upper[6] = 3.1415926;

        _configSize =  _lower.size();
        // cout<<"lower is: "; 
        // for (int i = 0; i < _configSize; i++)
        // {
        //     cout<< _lower[i]<<",";
        // }
        // cout<<endl;
        // cout<<"upper is: ";
        // for (int i = 0; i < _configSize; i++)
        // {
        //     cout<< _upper[i]<<",";
        // }
        // cout<<endl;

        // return true;
    }

    bool FindPath(std::ostream& sout, std::istream& sinput)
    {
         NodeTree RRTtree( GetEnv(),_GoalConfig,_StrConfig,_StepSize);
    
          srand((int)time(0));
          int counter=0;
          
          do
          {
            vector<double> temp =  Config();               
            RRTtree.add(temp);
          }while(!RRTtree.FindGoal());
           path = RRTtree.GivePath();

          RobotBasePtr robot = GetEnv()->GetRobot("PR2");


         for(int i=0;i<path.size();i++)
         {
           for (int j=0;j < 7 ;j++)
           {
               sout<< path[i][j];
               if (j !=6) 
                sout<<",";
           }
               if (i !=path.size()-1) 
                sout<<endl;
         } 
          return true;
    }

   
    Configuration Config()
    {
 
        int _configSize = _lower.size();
        if ((float) rand()/(float) RAND_MAX < _GoalBias)
        {
            return _GoalConfig;
        }

        else
        {
            Configuration sample(_configSize);

            for(int i = 0; i < _configSize; ++i) 
            {

                sample[i] =  ((float)rand()/RAND_MAX) * (_upper[i]- _lower[i]) +  _lower[i];
               
            }
            return sample;
        }
    }
    bool PathSmooth(std::ostream& sout, std::istream& sinput)
    {

      float ssc_step = 0.05;
		for (int i = 0; i < 200; i++)
		{
            if (path.size() <= 10)
                break;

            int pos1 = rand()%path.size();
            int pos2 = rand()%path.size();
 
            while ((pos1 == pos2) || (abs(pos1-pos2) <= 1) )
            {
                pos1 = rand()%path.size();
                pos2 = rand()%path.size();
            }

            int first = min(pos1, pos2);
            int second = max(pos1, pos2);

            if (distanceTo( path[first], path[second]) > ssc_step)
            {
                int TotSteps =distanceTo(path[first], path[second])/ ssc_step;  
                float TotStepsPos = (TotSteps * ssc_step)/distanceTo(path[first], path[second]);  
                vector<Configuration> stepPath;

                bool shortCutFailed = false;
                for (int step = 1; step <= TotSteps; step++)
                {
                    Configuration NewNode(7);

                    for (int d = 0; d < 7; d++)
                    {
                        NewNode[d] = path[first][d] + ( (path[second][d]-path[first][d])*TotStepsPos*step)/TotSteps;
                    }

                    if (CollideCheck(NewNode))
                    {
                        shortCutFailed = true;
                        break;
                    }
                    stepPath.push_back( NewNode);
                }
                if (shortCutFailed == false)
                {
                   path.erase(path.begin()+first, path.begin()+second+1);
        			     for (int step = 0; step < TotSteps; step++)
               		 {
               			path.insert(path.begin()+first+step, stepPath[step]);
               		 }
                  
                    
                }

                // cout << "path len: " << path.size() << endl;

            }

        }
         for(int i=0;i<path.size();i++)
         {
         for (int j=0;j < 7 ;j++)
             {
             sout<< path[i][j];
             if (j !=6) sout<<",";
             }
             if (i !=path.size()-1) sout<<endl;
          } 
        return true;
    }
  double distanceTo(const Configuration &a , const Configuration &b)
  {
      double weight[] =  { 3.17104446,2.75674493,1.7894779,2.23250068,0.0,0.80901263,0.0};
         double dist = 0.0;
          int langth = a.size();
            for (int i = 0; i < langth; ++i)
                dist += weight[i]*(a[i] - b[i]) * (a[i] - b[i]);
      
            return sqrt(dist);
  }
  bool CollideCheck(const Configuration &Config)
  {
         GetEnv()->GetRobot("PR2")->SetActiveDOFValues(Config);
        if ( GetEnv()->CheckCollision(GetEnv()->GetRobot("PR2")) || GetEnv()->GetRobot("PR2")->CheckSelfCollision() )
            return true;
        else
            return false;
  }

};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "myrrt" ) {
        return InterfaceBasePtr(new myRRT(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("myRRT");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}




