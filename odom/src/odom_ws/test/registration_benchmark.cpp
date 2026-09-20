// No ROS node or transport: benchmark the production motion and NanoGICP
// kernels against a fixed first-scan map. This is NOT the complete DLIO
// geometric observer / asynchronous keyframe pipeline.
#include "dlio/dlio.h"
#include "dlio/motion.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <chrono>
#include <algorithm>
#include <cstdlib>

int main(int argc, char **argv) {
  if (argc != 6) {std::cerr << "imu.bin clouds.bin correction.txt output.csv deskew(0|1)\n";return 2;}
  const bool deskew = std::stoi(argv[5]) != 0;
  const int cap = std::getenv("DLIO_CAP") ? std::stoi(std::getenv("DLIO_CAP")) : 4000;
  const double epsilon = std::getenv("DLIO_EPS") ? std::stod(std::getenv("DLIO_EPS")) : .01;
  dlio::ImuConditioner conditioner;
  std::ifstream cfg(argv[3]);
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)cfg>>conditioner.rotation(i,j);
  for(int i=0;i<3;++i)cfg>>conditioner.lever[i];
  if (!cfg || cap<64) return 3;
  std::ifstream im(argv[1],std::ios::binary),clouds(argv[2],std::ios::binary);
  std::ofstream out(argv[4]);
  std::vector<dlio::MotionImu> imu;
  double row[7];
  while(im.read(reinterpret_cast<char*>(row),sizeof(row))) {
    dlio::MotionImu raw,corrected;
    raw.stamp=row[0];raw.ang_vel={row[1],row[2],row[3]};raw.lin_accel={row[4],row[5],row[6]};
    if(conditioner.apply(raw,corrected))imu.push_back(corrected);
  }
  if(imu.size()<100)return 4;
  Eigen::Vector3f accel=Eigen::Vector3f::Zero(),bias=Eigen::Vector3f::Zero();
  for(int i=0;i<100;++i){accel+=imu[i].lin_accel;bias+=imu[i].ang_vel;}
  accel/=100;bias/=100;
  const Eigen::Vector3f abias=accel-accel.normalized()*9.80665;
  for(auto &s:imu){s.ang_vel-=bias;s.lin_accel-=abias;}
  Eigen::Quaternionf q=Eigen::Quaternionf::FromTwoVectors(accel,Eigen::Vector3f(0,0,9.80665));
  Eigen::Vector3f p=Eigen::Vector3f::Zero(),v=Eigen::Vector3f::Zero();
  double previous=imu[99].stamp;
  using Cloud=pcl::PointCloud<PointType>;
  Cloud::Ptr reference;
  nano_gicp::NanoGICP<PointType,PointType> gicp;
  gicp.setNumThreads(4);gicp.setCorrespondenceRandomness(16);gicp.setMaxCorrespondenceDistance(.5);
  gicp.setMaximumIterations(32);gicp.setTransformationEpsilon(epsilon);gicp.setRotationEpsilon(epsilon);
  gicp.setInitialLambdaFactor(1e-9);
  pcl::VoxelGrid<PointType> voxel;voxel.setLeafSize(.25,.25,.25);
  out<<"stamp,x,y,z,points,ms,converged\n"<<std::setprecision(15);
  double stamp;std::uint32_t count;std::size_t nout=0;
  while(clouds.read(reinterpret_cast<char*>(&stamp),8)&&clouds.read(reinterpret_cast<char*>(&count),4)) {
    std::vector<float> data(count*5);clouds.read(reinterpret_cast<char*>(data.data()),data.size()*4);
    if(!clouds)return 5;
    const auto begin=std::chrono::steady_clock::now();
    Cloud::Ptr points(new Cloud);
    for(std::size_t i=0;i<count;++i){
      PointType point;point.x=data[5*i];point.y=data[5*i+1];point.z=data[5*i+2];point.intensity=data[5*i+3];point.time=data[5*i+4];
      if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)||
          (std::abs(point.x)<1&&std::abs(point.y)<1&&std::abs(point.z)<1))continue;
      points->push_back(point);
    }
    if(points->empty())continue;
    std::sort(points->begin(),points->end(),[](const auto&a,const auto&b){return a.time<b.time;});
    std::vector<double> times;std::vector<std::size_t> indices;
    for(std::size_t i=0;i<points->size();++i)if(i==0||(*points)[i].time!=(*points)[i-1].time){times.push_back(stamp+(*points)[i].time);indices.push_back(i);}
    indices.push_back(points->size());
    const double scan_time=times[times.size()/2];
    auto a=std::upper_bound(imu.begin(),imu.end(),previous,[](double t,const auto&s){return t<s.stamp;});
    if(a==imu.begin())continue;
    --a;
    auto b=std::lower_bound(imu.begin(),imu.end(),times.back(),[](const auto&s,double t){return s.stamp<t;});
    if(b==imu.end())continue;
    ++b;
    const std::vector<dlio::MotionImu> samples(a,b);
    const auto frames=dlio::integrateMotion(previous,q,p,v,deskew?times:std::vector<double>{scan_time},samples,9.80665);
    if(frames.empty())continue;
    const Eigen::Matrix4f prior=frames[deskew?frames.size()/2:0];
    if(deskew){
      for(std::size_t j=0;j<times.size();++j)for(std::size_t i=indices[j];i<indices[j+1];++i)
        (*points)[i].getVector4fMap()=frames[j]*(*points)[i].getVector4fMap();
    }else pcl::transformPointCloud(*points,*points,prior);
    Cloud::Ptr filtered(new Cloud);voxel.setInputCloud(points);voxel.filter(*filtered);
    if(filtered->size()>static_cast<std::size_t>(cap)){
      Cloud::Ptr limited(new Cloud);limited->reserve(cap);
      for(int i=0;i<cap;++i)limited->push_back((*filtered)[static_cast<std::size_t>(i)*filtered->size()/cap]);
      filtered=limited;
    }
    Eigen::Matrix4f transform=prior;
    bool converged=true;
    if(!reference){reference=filtered;gicp.setInputTarget(reference);}
    else {
      gicp.setInputSource(filtered);Cloud aligned;gicp.align(aligned);
      transform=gicp.getFinalTransformation()*prior;converged=gicp.hasConverged();
    }
    const Eigen::Vector3f next=transform.block<3,1>(0,3);
    if(previous>0&&scan_time>previous)v=.8f*v+.2f*(next-p)/(scan_time-previous);
    p=next;q=Eigen::Quaternionf(transform.block<3,3>(0,0)).normalized();previous=scan_time;
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    out<<scan_time<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<filtered->size()<<','<<ms<<','<<converged<<'\n';
    if(++nout%100==0)std::cout<<nout<<" frames, "<<p.transpose()<<", "<<ms<<" ms\n"<<std::flush;
  }
}
