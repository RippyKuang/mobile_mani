#pragma once

#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <Eigen/Dense>
#include <osqp/osqp.h>

namespace dynamic {

struct HQPTask {
    std::string name;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    double weight = 1.0;
    // Corridor width used when this task is handed to the *next* priority level as a
    // hard equality. It must be expressed in the task's own units: a fixed 0.05 m/s^2
    // corridor on a base task is impossible to satisfy, so tolerance is per task.
    double tolerance = 0.05;
};

class HQPSolver {
public:
    explicit HQPSolver(int n_vars);
    ~HQPSolver();

    void reset();
    
    // Set variable bounds: lb <= x <= ub
    void setVariableBounds(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub);
    
    // Set general linear inequality constraints: lc <= C * x <= uc
    void setLinearConstraints(const Eigen::MatrixXd &C, const Eigen::VectorXd &lc, const Eigen::VectorXd &uc);
    
    // Add task at priority level (0 = highest)
    void addTask(int priority_level, const HQPTask &task);
    
    // Solve HQP and return the optimal x
    bool solve(Eigen::VectorXd &x_opt);

    // Hyperparameter setters
    void setSlackPenalty(double rho) { rho_ = rho; }
    void setRegularization(double lambda) { lambda_ = lambda; }
    void setEqualityTolerance(double delta) { delta_ = delta; }

private:
    int n_vars_;
    double rho_ = 1e3;        // Penalty on slack variables
    double lambda_ = 1e-4;    // Tikhonov regularization
    double delta_ = 0.05;     // Fallback tolerance for HQPTask::tolerance
    
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
    
    Eigen::MatrixXd C_user_;
    Eigen::VectorXd lc_user_;
    Eigen::VectorXd uc_user_;
    
    // Priority levels -> list of tasks at this level
    std::vector<std::vector<HQPTask>> levels_;
    
    // Internal single-level QP solver using OSQP
    bool solveSingleLevelQP(
        const Eigen::MatrixXd &A_task, const Eigen::VectorXd &b_task,
        const Eigen::MatrixXd &A_eq, const Eigen::VectorXd &b_eq_lb, const Eigen::VectorXd &b_eq_ub,
        const Eigen::MatrixXd &C_ineq, const Eigen::VectorXd &l_ineq, const Eigen::VectorXd &u_ineq,
        Eigen::VectorXd &x_opt, Eigen::VectorXd &slack_opt);
        
    static void eigenToCSC(const Eigen::MatrixXd &mat, std::vector<OSQPFloat> &x, std::vector<OSQPInt> &i, std::vector<OSQPInt> &p);
};

} // namespace dynamic
