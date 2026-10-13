#include "tech_core/arm_kinematics.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

// Machine-readable FK oracle for the independent source-URDF verification.
int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::invalid_argument("robot_probe MODEL.xml < configurations");
        tech_core::ArmKinematics arm(argv[1]);
        const auto n = arm.robot().joint_names.size();
        std::cout << std::setprecision(17);
        for (std::string line; std::getline(std::cin, line);) {
            if (line.empty()) continue;
            std::istringstream input(line);
            tech_core::planning::JointVector q(n);
            for (std::size_t i=0;i<n;++i) if (!(input>>q[i])) throw std::invalid_argument("Invalid probe input");
            std::string extra;
            if (input>>extra) throw std::invalid_argument("Extra probe coordinates");
            Eigen::Vector3d p; Eigen::Matrix3d r; tech_core::planning::Jacobian j;
            arm.forward(q,p,r,&j);
            auto frame = [](const double* pos, const double* rotation) {
                std::cout << '[';
                for(int i=0;i<3;++i) std::cout<<(i?",":"")<<pos[i];
                for(int i=0;i<9;++i) std::cout<<','<<rotation[i];
                std::cout<<']';
            };
            std::cout << "{\"bodies\":{";
            for(int b=1;b<arm.model()->nbody;++b) {
                std::cout<<(b>1?",":"")<<std::quoted(mj_id2name(arm.model(),mjOBJ_BODY,b))<<':';
                frame(arm.data()->xpos+3*b, arm.data()->xmat+9*b);
            }
            const Eigen::Matrix<double,3,3,Eigen::RowMajor> row=r;
            std::cout<<"},\"tool\":";frame(p.data(),row.data());
            std::cout<<",\"jacobian\":[";
            for(int row=0;row<6;++row) for(int col=0;col<q.size();++col)
                std::cout<<(row||col?",":"")<<j(row,col);
            std::cout<<"]}\n";
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
