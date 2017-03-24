#ifndef SIMULATION_VREP_TASK_TASK_HPP
#define SIMULATION_VREP_TASK_TASK_HPP

#include "simulation_vrep/TaskBase.hpp"
#include <vrep/vrep.hpp>

namespace simulation_vrep
{
    class Task : public TaskBase
    {
  	friend class TaskBase;
    protected:
        vrep::VREP *vrep;
    public:
        Task(std::string const& name = "simulation_vrep::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
	       ~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
