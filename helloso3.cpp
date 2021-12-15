#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/geometry.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

using namespace std;
int main( int argc, char** argv )
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addCoordinateSystem(100.0);

    const double kPi = Sophus::Constants<double>::pi();
    Sophus::SO3d R1 = Sophus::SO3d::rotX(kPi / 4);
    Sophus::SO3d R2 = Sophus::SO3d::rotY(kPi / 6);
    Sophus::SO3d R3 = Sophus::SO3d::rotZ(-kPi / 3);
    std::vector<Sophus::SO3d> R_vec;
    for (double i = 0; i < 1; i = i + 0.01) {
        // Eigen::Vector3d res_log = ;
        Eigen::Vector3d log12 = (R1.inverse()*R2).log();
        Eigen::Vector3d log23 = (R2.inverse()*R3).log();
        Eigen::Vector3d log_long = Sophus::SO3d::exp(-log12*i)*R1.inverse()*R1*Sophus::SO3d::exp(log23*i).log();
        R_vec.push_back(R1*Sophus::SO3d::exp(log12*i)*Sophus::SO3d::exp(log_long*i));
    }

    // cout<<R_vec[1].matrix()<<endl;

    Eigen::Vector3d * vs = new Eigen::Vector3d[4];
    vs[0] = Eigen::Vector3d(0,0,0);
    vs[1] = Eigen::Vector3d(200,0,0);
    vs[2] = Eigen::Vector3d(200,200,200);
    vs[3] = Eigen::Vector3d(0,0,200);
    uint size =  R_vec.size();
    // cout<<size<<endl;
    for ( int i = 0; i < 50; i++ ) {
        Eigen::Affine3d r(R_vec[i].matrix());
        Eigen::Affine3d t(Eigen::Translation3d(vs[1]+((double)i/(double)50)*(vs[2]-vs[1])));
        Eigen::Affine3d mat = r*t;
        //  = Eigen::Translation3d(vs[2]+((double)i/(double)size)*(vs[3]-vs[2]))*Eigen::Rotation3D(R_vec[i].matrix());
    //     mat.rotation() = 
    //     // pcl::PointXYZ p(100*q_vec(i).imag_i(), 100*q_vec(i).imag_j(), 100*q_vec(i).imag_k());
    //     // viewer->addSphere(p, 5, to_string(i));
        viewer->addCoordinateSystem(20, mat.cast<float>());
    }
    for ( int i = 0; i < 50; i++ ) {
        Eigen::Affine3d r(R_vec[i+50].matrix());
        Eigen::Affine3d t(Eigen::Translation3d(vs[2]+((double)i/(double)50)*(vs[3]-vs[2])));
        Eigen::Affine3d mat = r*t;
        //  = Eigen::Translation3d(vs[2]+((double)i/(double)size)*(vs[3]-vs[2]))*Eigen::Rotation3D(R_vec[i].matrix());
    //     mat.rotation() = 
    //     // pcl::PointXYZ p(100*q_vec(i).imag_i(), 100*q_vec(i).imag_j(), 100*q_vec(i).imag_k());
    //     // viewer->addSphere(p, 5, to_string(i));
        viewer->addCoordinateSystem(20, mat.cast<float>());
    }

    // Calculate speed
    std::vector<double> diff;
    for (int i = 0; i < (R_vec.size() - 2); i++) {
        Eigen::Quaterniond q_vec(R_vec[i].matrix());
        Eigen::Quaterniond q_vec_plus1(R_vec[i+1].matrix());
        Eigen::Quaterniond q_vec_plus2(R_vec[i+2].matrix());
        diff.push_back(((q_vec_plus1.coeffs()-q_vec.coeffs()).norm()+(q_vec_plus1.coeffs()-q_vec_plus2.coeffs()).norm())/2);
    }
    for (int i = 0; i < diff.size(); i++) {
        std::cout << diff[i] << std::endl;
    }


    // Eigen::Vector3d r1 = R1.log();
    // cout << r1.transpose() << endl;
    // cout << Sophus::SO3d::hat(r1) << endl;
    // cout << Sophus::SO3d::vee(Sophus::SO3d::hat(r1)).transpose() << endl;

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
    viewer->resetStoppedFlag();
    viewer->close();
    return 0;

}