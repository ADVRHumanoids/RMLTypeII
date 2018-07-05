#include <Wrappers/TrajectoryGenerator.h>

mpl::Utils::TrajectoryGenerator::TrajectoryGenerator(int n_dof, double period, const Eigen::VectorXd& x_0):
    N_DOF(n_dof),
    CYCLE_TIME(period),
    _interpolator(n_dof, period),
    _input(n_dof),
    _output(n_dof)
{
    Eigen::Map<Eigen::VectorXd> cur_pos(_input.CurrentPositionVector->VecData, 
                                        _input.CurrentPositionVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> cur_vel(_input.CurrentVelocityVector->VecData, 
                                        _input.CurrentVelocityVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> cur_acc(_input.CurrentAccelerationVector->VecData, 
                                        _input.CurrentAccelerationVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> target_pos(_input.TargetPositionVector->VecData, 
                                           _input.TargetPositionVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> target_vel(_input.TargetVelocityVector->VecData, 
                                           _input.TargetVelocityVector->GetVecDim());
    
    if(x_0.size() != N_DOF)
    {
        throw std::invalid_argument("x_0.size() != N_DOF");
    }
    
    cur_pos = x_0;
    cur_vel.setZero();
    cur_acc.setZero();
    target_pos = cur_pos;
    target_vel.setZero();
    
    Eigen::Map<Eigen::VectorXd> max_vel(_input.MaxVelocityVector->VecData, 
                                        _input.MaxVelocityVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> max_acc(_input.MaxAccelerationVector->VecData, 
                                        _input.MaxAccelerationVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> max_jerk(_input.MaxJerkVector->VecData, 
                                         _input.MaxJerkVector->GetVecDim());
    
    max_vel.setConstant(1.0);
    max_acc.setConstant(10.0);
    max_jerk.setConstant(100.0);
    
    _input.SelectionVector->Set(true);
    
    
}

void mpl::Utils::TrajectoryGenerator::setReference(Eigen::Ref<const Eigen::VectorXd> pos, Eigen::Ref<const Eigen::VectorXd> vel)
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

bool mpl::Utils::TrajectoryGenerator::update(Eigen::Ref< Eigen::VectorXd > pos, Eigen::Ref< Eigen::VectorXd > vel)
{
    auto result =   _interpolator.RMLPosition(_input,
                                              &_output,
                                              _flags);

    if (result < 0)
    {
        printf("An error occurred (%d).\n", result );
        return false;
    }

    *_input.CurrentPositionVector      =   *_output.NewPositionVector      ;
    *_input.CurrentVelocityVector      =   *_output.NewVelocityVector      ;
    *_input.CurrentAccelerationVector  =   *_output.NewAccelerationVector  ;
    
    Eigen::Map<Eigen::VectorXd> next_pos(_output.NewPositionVector->VecData, 
                                         _output.NewPositionVector->GetVecDim());
    
    Eigen::Map<Eigen::VectorXd> next_vel(_output.NewVelocityVector->VecData, 
                                         _output.NewVelocityVector->GetVecDim());
    
    pos = next_pos;
    vel = next_vel;
    
    return true;
}


void mpl::Utils::TrajectoryGenerator::setAccelerationLimits(const double qddot_max)
{
    Eigen::Map<Eigen::VectorXd> max_acc(_input.MaxAccelerationVector->VecData, 
                                        _input.MaxAccelerationVector->GetVecDim());
    
    if(qddot_max <= 0)
    {
        throw std::invalid_argument("qddot_max <= 0");
    }
    
    max_acc.setConstant(qddot_max);
}

void mpl::Utils::TrajectoryGenerator::setVelocityLimits(const double qdot_max)
{
    Eigen::Map<Eigen::VectorXd> max_vel(_input.MaxVelocityVector->VecData, 
                                        _input.MaxVelocityVector->GetVecDim());
    
    if(qdot_max <= 0)
    {
        throw std::invalid_argument("qdot_max <= 0");
    }
    
    max_vel.setConstant(qdot_max);
}