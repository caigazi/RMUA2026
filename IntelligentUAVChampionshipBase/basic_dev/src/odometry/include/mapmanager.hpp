#include "keyframe.hpp"
#include <iostream>
#include <map>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <thread>


class MapManager
{
public:
    MapManager();
    ~MapManager();
    void setCamParameter(Eigen::Matrix4d& Trl, Eigen::Matrix4d& caml_intrinsic, Eigen::Matrix4d& camr_intrinsic, Eigen::Vector4d& cmal_distor, Eigen::Vector4d& cmar_distor);
    void insertKF(KeyFrame& kf);
    std::map<int, KeyFrame> m_keyframes;
    void getLastRTFrameImgAndFeats(cv::Mat& img, cv::Mat& img_bgr, std::vector<RealTimeFrameFeature>& realf_feat_l);
    void updateFrame(cv::Mat& img, cv::Mat& img_bgr, std::vector<RealTimeFrameFeature>& realf_feat_l);
    void updateUsefulRTFrame(Eigen::Vector3d& gps_position, long long timecode);
    KeyFrame& curKeyFrame();
    // void triangulatePointsOnCamL(Eigen::Matrix4d& Trl,
    //         std::vector<Eigen::Vector2d>& pointls, std::vector<Eigen::Vector2d>& pointrs, std::vector<Eigen::Vector3d>& point_3ds);
    pcl::PointCloud<pcl::PointXYZ> m_global_PCL;
    void startOptimal();
private:
    void triangulatePointsOnCamLCurKF();
    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u, bool is_caml) const;
    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P, bool is_caml) const;
    cv::Ptr<cv::StereoSGBM> m_sgbm_ptr = cv::StereoSGBM::create(
      0,          // minDisparity（最小视差）
      64,         // numDisparities（视差范围）
      10,         // blockSize（块大小）
      120,        // P1（正则化参数1）
      15,         // P2（正则化参数2）
      0,          // disp12MaxDiff（左右视差最大差值）
      1,          // preFilterCap（预滤波阈值）
      1,          // uniquenessRatio（唯一性比率）
      0,          // speckleWindowSize（斑点窗口大小）
      0,          // speckleRange（斑点范围）
      cv::StereoSGBM::MODE_SGBM   // 算法模式
    );
    int m_KFCount=0; //use to set kf id;
    int m_curKFIdx=0;//当前使用中的kf
    Eigen::Matrix4d m_Cam_Trl;
    Eigen::Matrix4d m_CamL_Intrinsic;
    Eigen::Matrix4d m_CamR_Intrinsic;
    Eigen::Vector4d m_CamL_Distor;
    Eigen::Vector4d m_CamR_Distor;
    std::unique_ptr<g2o::SparseOptimizer> m_optimizer_ptr;
    // std::vector<int> m_pcl_coresponed_vertexs_id;
    std::vector<bool> m_pcl_is_optimal;
    // std::vector<g2o::VertexPointXYZ*> m_pcl_coresponed_vertexs;
    // int m_Vertex_Cnt;
    std::mutex m_kf_mtx;
};