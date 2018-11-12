#include <Wrappers/TrajectoryGeneratorVelocity.h>
#include <iostream>


Reflexxes::Utils::TrajectoryGeneratorVelocity::TrajectoryGeneratorVelocity(int n_dof, double period):
    N_DOF(n_dof),
    CYCLE_TIME(period),
    _interpolator(n_dof, period),
    _input(n_dof),
    _output(n_dof)
{
    
    Eigen::Map<Eigen::VectorXd> cur_vel(_input.CurrentVelocityVector->VecData, 
                                        _input.CurrentVelocityVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> cur_acc(_input.CurrentAccelerationVector->VecData, 
                                        _input.CurrentAccelerationVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> target_vel(_input.TargetVelocityVector->VecData, 
                                           _input.TargetVelocityVector->GetVecDim());
    
    cur_vel.setZero();
    cur_acc.setZero();
    target_vel.setZero();
    
    
    Eigen::Map<Eigen::VectorXd> max_acc(_input.MaxAccelerationVector->VecData, 
                                        _input.MaxAccelerationVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> max_jerk(_input.MaxJerkVector->VecData, 
                                         _input.MaxJerkVector->GetVecDim());
    
    _qdot_max.setConstant(N_DOF, 1.0);
    max_acc.setConstant(10.0);
    max_jerk.setConstant(100.0);
    
    _input.SelectionVector->Set(true);
    
    
}

bool Reflexxes::Utils::TrajectoryGeneratorVelocity::update(Eigen::Ref< Eigen::VectorXd > vel)
{
    auto result =   _interpolator.RMLVelocity(_input,
                                              &_output,
                                              _flags);
    

    if (result < 0)
    {
        printf("TrajectoryGeneratorVelocity::update: An error occurred (%d).\n", result );
        return false;
    }

    *_input.CurrentVelocityVector      =   *_output.NewVelocityVector      ;
    *_input.CurrentAccelerationVector  =   *_output.NewAccelerationVector  ;
    
    
    Eigen::Map<Eigen::VectorXd> next_vel(_output.NewVelocityVector->VecData, 
                                         _output.NewVelocityVector->GetVecDim());
    
    vel = next_vel;
    
    return true;
}


void Reflexxes::Utils::TrajectoryGeneratorVelocity::setAccelerationLimits(const double qddot_max)
{
    Eigen::Map<Eigen::VectorXd> max_acc(_input.MaxAccelerationVector->VecData, 
                                        _input.MaxAccelerationVector->GetVecDim());
    
    if(qddot_max <= 0)
    {
        throw std::invalid_argument("qddot_max <= 0");
    }
    
    max_acc.setConstant(qddot_max);
}

void Reflexxes::Utils::TrajectoryGeneratorVelocity::setVelocityLimits(const double qdot_max)
{
    
    
    if(qdot_max <= 0)
    {
        throw std::invalid_argument("qdot_max <= 0");
    }
    
    _qdot_max.setConstant(N_DOF, qdot_max);
}

void Reflexxes::Utils::TrajectoryGeneratorVelocity::reset()
{
    Eigen::Map<Eigen::VectorXd> cur_vel(_input.CurrentVelocityVector->VecData, 
                                        _input.CurrentVelocityVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> cur_acc(_input.CurrentAccelerationVector->VecData, 
                                        _input.CurrentAccelerationVector->GetVecDim());
    
    cur_vel.setZero();
    cur_acc.setZero();
    
    setReference(cur_vel);
}

void Reflexxes::Utils::TrajectoryGeneratorVelocity::setAccelerationLimits(Eigen::Ref<const Eigen::VectorXd> qddot_max)
{
    Eigen::Map<Eigen::VectorXd> max_acc(_input.MaxAccelerationVector->VecData, 
                                        _input.MaxAccelerationVector->GetVecDim());
    
    if(qddot_max.size() != max_acc.size())
    {
        throw std::invalid_argument("TrajectoryGeneratorVelocity::setAccelerationLimits: qddot_max.size() != max_acc.size()");
    }
    
    if( (qddot_max.array() <= 0).any() )
    {
        throw std::invalid_argument("TrajectoryGeneratorVelocity::setAccelerationLimits: some acc limits are <= 0");
    }
    
    max_acc = qddot_max;
}


void Reflexxes::Utils::TrajectoryGeneratorVelocity::setVelocityLimits(Eigen::Ref<const Eigen::VectorXd> qdot_max)
{
    
    
    if(qdot_max.size() != N_DOF)
    {
        throw std::invalid_argument("TrajectoryGeneratorVelocity::setVelocityLimits: qdot_max.size() != N_DOF");
    }
    
    if( (qdot_max.array() <= 0).any() )
    {
        throw std::invalid_argument("TrajectoryGeneratorVelocity::setVelocityLimits: some vel limits are <= 0");
    }
    
    _qdot_max = qdot_max;
    
}
