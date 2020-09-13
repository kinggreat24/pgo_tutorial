/*
 * @Author: kinggreat24
 * @Date: 2019-12-28 15:16:41
 * @LastEditTime : 2019-12-28 23:43:55
 * @LastEditors  : kinggreat24
 * @Description: 
 * @FilePath: /ch11/ceres_SE3.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include "ceres/ceres.h"


#include <sophus/se3.h>
#include <sophus/so3.h>

#include <chrono>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;


class PoseGraphError{
public:
    PoseGraphError(Sophus::SE3 pose_ij)
        : pose_ij_(pose_ij)
    {}
    
    template<typename T>
    bool operator()(const T* const pose_i,
                const T* const pose_j,
                T* residuals)const
    {  
        // Sophus::SE3 pose_i_= Sophus::SE3(
        //     Sophus::SO3(pose_i[0],pose_i[1], pose_i[2]),
        //     Eigen::Vector3d ( pose_i[3], pose_i[4], pose_i[5])
        // );
        
        // Sophus::SE3 pose_j_= Sophus::SE3(
        //     Sophus::SO3(pose_j[0],pose_j[1], pose_j[2]),
        //     Eigen::Vector3d ( pose_j[3], pose_j[4], pose_j[5] )
        // );
        // Sophus::Vector6d se3 = (pose_ij_.inverse()*(pose_i_.inverse()*pose_j_)).log();
        // residuals = se3.data();
    }    

    static ceres::CostFunction* Create(const Sophus::SE3 pose_ij){
        return (new ceres::AutoDiffCostFunction<PoseGraphError,6,6,6>(
            new PoseGraphError(pose_ij)));
    }

private:
    Sophus::SE3 pose_ij_;
};


int main(int argc, char** argv)
{
    if ( argc != 2 )
    {
        cout<<"Usage: pose_graph_g2o_SE3 sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }
    std::map<int,Sophus::SE3> poses;
    ceres::Problem problem;

    while ( !fin.eof() )
    {
        std::string name="";
        fin>>name;
        if("VERTEX_SE3:QUAT" == name)
        {
            int index = 0;
            fin>>index;

            double data[7];
            for ( int i=0; i<7; i++ )
                fin>>data[i];
            poses[index] =   Sophus::SE3 (
                    Eigen::Quaterniond ( data[6],data[3], data[4], data[5] ),
                    Eigen::Vector3d ( data[0], data[1], data[2] )
            );
        }
        else if("EDGE_SE3:QUAT" == name)
        {
            int index1=0,index2 = 0;
            fin>>index1>>index2;

            double data[7];
            for ( int i=0; i<7; i++ )
                fin>>data[i];

            
            Sophus::SE3 pose_ij = Sophus::SE3 (
                    Eigen::Quaterniond ( data[6],data[3], data[4], data[5] ),
                    Eigen::Vector3d ( data[0], data[1], data[2] )
            );

            ceres::CostFunction * cost_function = PoseGraphError::Create(pose_ij);
            problem.AddResidualBlock( cost_function, new ceres::CauchyLoss(1.),poses[index1].matrix().data(),poses[index2].matrix().data());
        }
        
    }

     // 配置求解器
    ceres::Solver::Options options;                               // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;                 // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;                  // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.BriefReport() <<endl;
  

    return 0;
}