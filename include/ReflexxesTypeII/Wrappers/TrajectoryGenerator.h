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
        
        template <typename DerivedPos, typename DerivedVel>
        void setReference(const Eigen::MatrixBase<DerivedPos>& pos, 
                          const Eigen::MatrixBase<DerivedVel>& vel);
        
        bool update(Eigen::Ref<Eigen::VectorXd> pos, 
                    Eigen::Ref<Eigen::VectorXd> vel);
        
        void reset(Eigen::Ref<const Eigen::VectorXd> pos);
        
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

template <typename DerivedPos, typename DerivedVel>
void Reflexxes::Utils::TrajectoryGenerator::setReference(const Eigen::MatrixBase< DerivedPos >& pos, 
                                                         const Eigen::MatrixBase< DerivedVel >& vel)
{
    Eigen::Map<Eigen::VectorXd> target_pos(_input.TargetPositionVector->VecData, 
                                           _input.TargetPositionVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> target_vel(_input.TargetVelocityVector->VecData, 
                                           _input.TargetVelocityVector->GetVecDim());
    
    target_pos = pos;
    
    if(vel.size() > 0)
    {
        target_vel = vel;
    }
    else
    {
        target_vel.setZero();
    }
}





#endif