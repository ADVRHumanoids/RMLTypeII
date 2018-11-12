#ifndef __MPL_TRAJINTERPOLVEL_H__
#define __MPL_TRAJINTERPOLVEL_H__

#include <ReflexxesAPI.h>
#include <eigen3/Eigen/Dense>
#include <memory>



namespace Reflexxes 
{
    namespace Utils {
    
    class TrajectoryGeneratorVelocity
    {
        
    public:
        
        typedef std::shared_ptr<TrajectoryGeneratorVelocity> Ptr;
        
        TrajectoryGeneratorVelocity(int n_dof, double period);
        
        template <typename DerivedVel>
        void setReference(const Eigen::MatrixBase<DerivedVel>& vel);
        
        bool update(Eigen::Ref<Eigen::VectorXd> vel);
        
        void reset();
        
        void setVelocityLimits(Eigen::Ref<const Eigen::VectorXd> qdot_max);
        void setAccelerationLimits(Eigen::Ref<const Eigen::VectorXd> qddot_max);
        
        void setVelocityLimits(const double qdot_max);
        void setAccelerationLimits(const double qddot_max);
        
    private:
        
        const int N_DOF;
        const double CYCLE_TIME;
        
        ReflexxesAPI _interpolator;
        RMLVelocityInputParameters _input;
        RMLVelocityOutputParameters _output;
        RMLVelocityFlags _flags;
        
        Eigen::VectorXd _qdot_max;
        
    };
    
    }
}

template <typename DerivedVel>
void Reflexxes::Utils::TrajectoryGeneratorVelocity::setReference(const Eigen::MatrixBase< DerivedVel >& vel)
{
    
    Eigen::Map<Eigen::VectorXd> target_vel(_input.TargetVelocityVector->VecData, 
                                           _input.TargetVelocityVector->GetVecDim());
    
    
    if(vel.size() != N_DOF)
    {
        throw std::invalid_argument("TrajectoryGeneratorVelocity::setReference: vel.size() != N_DOF");
    }
    
    target_vel = vel.array().min(_qdot_max.array()).max(-_qdot_max.array());
}





#endif