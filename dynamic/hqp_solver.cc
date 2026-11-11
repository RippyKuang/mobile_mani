#include "hqp_solver.h"
#include <cmath>
#include <iostream>

namespace dynamic {

HQPSolver::HQPSolver(int n_vars) : n_vars_(n_vars) {
    reset();
}

HQPSolver::~HQPSolver() = default;

void HQPSolver::reset() {
    lb_.resize(0);
    ub_.resize(0);
    C_user_.resize(0, 0);
    lc_user_.resize(0);
    uc_user_.resize(0);
    levels_.clear();
}

void HQPSolver::setVariableBounds(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub) {
    lb_ = lb;
    ub_ = ub;
}

void HQPSolver::setLinearConstraints(const Eigen::MatrixXd &C, const Eigen::VectorXd &lc, const Eigen::VectorXd &uc) {
    C_user_ = C;
    lc_user_ = lc;
    uc_user_ = uc;
}

void HQPSolver::addTask(int priority_level, const HQPTask &task) {
    if (priority_level < 0) return;
    if (priority_level >= static_cast<int>(levels_.size())) {
        levels_.resize(priority_level + 1);
    }
    levels_[priority_level].push_back(task);
}

void HQPSolver::eigenToCSC(const Eigen::MatrixXd &mat, std::vector<OSQPFloat> &x, std::vector<OSQPInt> &i, std::vector<OSQPInt> &p) {
    p.clear();
    x.clear();
    i.clear();
    p.push_back(0);
    for (int col = 0; col < mat.cols(); ++col) {
        for (int row = 0; row < mat.rows(); ++row) {
            double val = mat(row, col);
            if (std::abs(val) > 1e-14) {
                x.push_back(val);
                i.push_back(row);
            }
        }
        p.push_back(static_cast<OSQPInt>(x.size()));
    }
}

bool HQPSolver::solveSingleLevelQP(
    const Eigen::MatrixXd &A_task, const Eigen::VectorXd &b_task,
    const Eigen::MatrixXd &A_eq, const Eigen::VectorXd &b_eq_lb, const Eigen::VectorXd &b_eq_ub,
    const Eigen::MatrixXd &C_ineq, const Eigen::VectorXd &l_ineq, const Eigen::VectorXd &u_ineq,
    Eigen::VectorXd &x_opt, Eigen::VectorXd &slack_opt)
{
    const int m = static_cast<int>(A_task.rows());
    const int n = n_vars_;
    const int n_var = n + m;

    // Cost: 0.5 * ||A x - eps - b||^2 + 0.5 * rho * ||eps||^2 + 0.5 * lambda * ||x||^2
    // z = [x; eps]
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_var, n_var);
    if (m > 0) {
        P.block(0, 0, n, n) = A_task.transpose() * A_task;
        P.block(0, n, n, m) = -A_task.transpose();
        P.block(n, 0, m, n) = -A_task;
        P.block(n, n, m, m) = (1.0 + rho_) * Eigen::MatrixXd::Identity(m, m);
    }
    // Add Tikhonov regularization
    P.block(0, 0, n, n) += lambda_ * Eigen::MatrixXd::Identity(n, n);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_var);
    if (m > 0) {
        q.head(n) = -A_task.transpose() * b_task;
        q.tail(m) = b_task;
    }

    // Constraints:
    // 1. Variable bounds: lb <= x <= ub (n rows)
    // 2. Prior level equalities: b_eq_lb <= A_eq x <= b_eq_ub (n_eq rows)
    // 3. User linear inequalities: l_ineq <= C_ineq x <= u_ineq (n_ineq rows)
    const int n_eq = static_cast<int>(A_eq.rows());
    const int n_ineq = static_cast<int>(C_ineq.rows());
    const int n_constr = n + n_eq + n_ineq;

    Eigen::MatrixXd A_osqp = Eigen::MatrixXd::Zero(n_constr, n_var);
    Eigen::VectorXd l_osqp = Eigen::VectorXd::Zero(n_constr);
    Eigen::VectorXd u_osqp = Eigen::VectorXd::Zero(n_constr);

    int row_idx = 0;
    // 1. Variable bounds
    A_osqp.block(row_idx, 0, n, n) = Eigen::MatrixXd::Identity(n, n);
    if (lb_.size() == n) {
        l_osqp.segment(row_idx, n) = lb_;
    } else {
        l_osqp.segment(row_idx, n) = Eigen::VectorXd::Constant(n, -1e6);
    }
    if (ub_.size() == n) {
        u_osqp.segment(row_idx, n) = ub_;
    } else {
        u_osqp.segment(row_idx, n) = Eigen::VectorXd::Constant(n, 1e6);
    }
    row_idx += n;

    // 2. Prior level equalities
    if (n_eq > 0) {
        A_osqp.block(row_idx, 0, n_eq, n) = A_eq;
        l_osqp.segment(row_idx, n_eq) = b_eq_lb;
        u_osqp.segment(row_idx, n_eq) = b_eq_ub;
        row_idx += n_eq;
    }

    // 3. User linear inequalities
    if (n_ineq > 0) {
        A_osqp.block(row_idx, 0, n_ineq, n) = C_ineq;
        l_osqp.segment(row_idx, n_ineq) = l_ineq;
        u_osqp.segment(row_idx, n_ineq) = u_ineq;
        row_idx += n_ineq;
    }

    // Upper triangular part of P for OSQP
    Eigen::MatrixXd P_tri = P.triangularView<Eigen::Upper>();

    std::vector<OSQPFloat> P_x, A_x;
    std::vector<OSQPInt> P_i, P_p, A_i, A_p;
    eigenToCSC(P_tri, P_x, P_i, P_p);
    eigenToCSC(A_osqp, A_x, A_i, A_p);

    OSQPCscMatrix P_csc, A_csc;
    OSQPCscMatrix_set_data(&P_csc, n_var, n_var, static_cast<OSQPInt>(P_x.size()), P_x.data(), P_i.data(), P_p.data());
    OSQPCscMatrix_set_data(&A_csc, n_constr, n_var, static_cast<OSQPInt>(A_x.size()), A_x.data(), A_i.data(), A_p.data());

    OSQPSettings *settings = OSQPSettings_new();
    settings->verbose = 0;
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    settings->max_iter = 4000;
    settings->adaptive_rho = 0;

    OSQPSolver *solver = nullptr;
    OSQPInt exitflag = osqp_setup(&solver, &P_csc, q.data(), &A_csc, l_osqp.data(), u_osqp.data(), n_constr, n_var, settings);
    if (exitflag != 0) {
        std::cerr << "[HQPSolver] osqp_setup failed with error: " << osqp_error_message(exitflag) << std::endl;
        OSQPSettings_free(settings);
        return false;
    }

    exitflag = osqp_solve(solver);
    bool success = (exitflag == 0) && (solver->info->status_val == OSQP_SOLVED || solver->info->status_val == OSQP_SOLVED_INACCURATE);
    if (!success) {
        static int fail_count = 0;
        if (++fail_count % 500 == 1) {
            std::cerr << "[HQPSolver] osqp_solve failed with status: " << solver->info->status << std::endl;
        }
    } else {
        x_opt.resize(n);
        for (int k = 0; k < n; ++k) {
            x_opt[k] = solver->solution->x[k];
        }
        if (m > 0) {
            slack_opt.resize(m);
            for (int k = 0; k < m; ++k) {
                slack_opt[k] = solver->solution->x[n + k];
            }
        } else {
            slack_opt.resize(0);
        }
    }

    osqp_cleanup(solver);
    OSQPSettings_free(settings);
    return success;
}

bool HQPSolver::solve(Eigen::VectorXd &x_opt) {
    Eigen::MatrixXd A_eq(0, n_vars_);
    Eigen::VectorXd b_eq_lb(0);
    Eigen::VectorXd b_eq_ub(0);

    Eigen::VectorXd current_x = Eigen::VectorXd::Zero(n_vars_);

    for (size_t lvl = 0; lvl < levels_.size(); ++lvl) {
        const auto &tasks = levels_[lvl];
        if (tasks.empty()) continue;

        // Concatenate all tasks in this priority level
        int total_rows = 0;
        for (const auto &t : tasks) {
            total_rows += static_cast<int>(t.A.rows());
        }
        if (total_rows == 0) continue;

        Eigen::MatrixXd A_lvl(total_rows, n_vars_);
        Eigen::VectorXd b_lvl(total_rows);
        int cur_row = 0;
        for (const auto &t : tasks) {
            int rows = static_cast<int>(t.A.rows());
            A_lvl.block(cur_row, 0, rows, n_vars_) = t.weight * t.A;
            b_lvl.segment(cur_row, rows) = t.weight * t.b;
            cur_row += rows;
        }

        Eigen::VectorXd x_lvl, slack_lvl;
        bool ok = solveSingleLevelQP(A_lvl, b_lvl, A_eq, b_eq_lb, b_eq_ub, C_user_, lc_user_, uc_user_, x_lvl, slack_lvl);
        if (!ok) {
         //   std::cerr << "[HQPSolver] Failed to solve priority level " << lvl << std::endl;
            if (lvl == 0) return false;
            // Fall back to previous level's solution
            break;
        }

        current_x = x_lvl;

        // Add this level's achieved task value to equality constraints for next levels
        Eigen::VectorXd achieved = A_lvl * current_x;
        int old_eq_rows = static_cast<int>(A_eq.rows());
        A_eq.conservativeResize(old_eq_rows + total_rows, n_vars_);
        b_eq_lb.conservativeResize(old_eq_rows + total_rows);
        b_eq_ub.conservativeResize(old_eq_rows + total_rows);

        A_eq.block(old_eq_rows, 0, total_rows, n_vars_) = A_lvl;
        b_eq_lb.segment(old_eq_rows, total_rows) = achieved - Eigen::VectorXd::Constant(total_rows, delta_);
        b_eq_ub.segment(old_eq_rows, total_rows) = achieved + Eigen::VectorXd::Constant(total_rows, delta_);
    }

    x_opt = current_x;
    return true;
}

} // namespace dynamic
