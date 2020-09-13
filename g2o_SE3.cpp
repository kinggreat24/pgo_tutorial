/*
 * @Author: kinggreat24
 * @Date: 2019-12-28 14:45:24
 * @LastEditTime : 2019-12-28 15:01:51
 * @LastEditors  : kinggreat24
 * @Description: 
 * @FilePath: /ch11/g2o_SE3.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */
#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
using namespace std;



int main(int argc,char** argv)
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

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;  // 6x6 BlockSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    g2o::SparseOptimizer optimizer;                            // 图模型
    optimizer.setAlgorithm( solver );                          // 设置求解器

    int vertexCnt = 0, edgeCnt = 0;                            // 顶点和边的数量
    while ( !fin.eof() )
    {
        std::string name="";
        fin>>name;
        if(name=="VERTEX_SE3:QUAT")
        {
            //添加顶点
            int index = 0;
            fin>>index;

            g2o::VertexSE3* v= new g2o::VertexSE3();

            v->setId(index);
            v->read(fin);

            optimizer.addVertex(v);
            vertexCnt++;

            if(0 == index)
                v->setFixed(true);
        }
        else if(name=="EDGE_SE3:QUAT")
        {
            //添加观测值
            int index1 = 0, index2=0;
            fin>>index1;
            fin>>index2;

            g2o::EdgeSE3* e = new g2o::EdgeSE3();

            e->setId(edgeCnt);
            edgeCnt_++;
            e->setVertex(0,optimizer.vertices()[index1]);
            e->setVertex(1,optimizer.vertices()[index2]);

            e->read(fin);

            optimizer.addEdge(e);
        }
        if ( !fin.good() ) break;
    }

    cout<<"read total "<<vertexCnt<<" vertices, "<<edgeCnt<<" edges."<<endl;
    
    cout<<"prepare optimizing ..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(30);
    
    cout<<"saving optimization results ..."<<endl;
    optimizer.save("result.g2o");

    return 0;
}