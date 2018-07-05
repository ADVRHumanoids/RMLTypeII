#ifndef __MPL_TRAJINTERPOL_H__
#define __MPL_TRAJINTERPOL_H__

#include <ReflexxesAPI.h>
#include <eigen3/Eigen/Dense>
#include <memory>



namespace Reflexxes 
{
    namespace Utils {
    
    class TrajectoryGenerator
    {
        
    public:
        
        typedef std::shared_ptr<TrajectoryGenerator> Ptr;
        
        TrajectoryGenerator(int n_dof, double period, const Eigen::VectorXd& x_0);
        
        void setReference(Eigen::Ref<const Eigen::VectorXd> pos, 
                          Eigen::Ref<const Eigen::VectorXd> vel = Eigen::VectorXd());
        
        bool update(Eigen::Ref<Eigen::VectorXd> pos, 
                    Eigen::Ref<Eigen::VectorXd> vel);
        
        void setVelocityLimits(Eigen::Ref<const Eigen::VectorXd> qdot_max);
        void setAccelerationLimits(Eigen::Ref<const Eigen::VectorXd> qddot_max);
        
        void setVelocityLimits(const double qdot_max);
        void setAccelerationLimits(const double qddot_max);
        
    private:
        
        const int N_DOF;
        const double CYCLE_TIME;
        
        ReflexxesAPI _interpolator;
        RMLPositionInputParameters _input;
        RMLPositionOutputParameters _output;
        RMLPositionFlags _flags;
        
    };
    
    }
}





#endif