// ========================================
// 优化函数实现
// ========================================

#include "locator.h"
#include "utils/math.h" // 可能需要数学工具函数

// 优化1: 计算有效样本数 (Effective Sample Size)
double Locator::calcEffectiveSampleSize()
{
    int rows = hypos.rows();
    if (rows <= 0) return 0.0;
    
    // ESS = 1 / Σ(w_i²)
    double sumSquaredWeights = (hypos.col(4).square()).sum();
    
    if (fabs(sumSquaredWeights) < 1e-10) return 0.0;
    
    return 1.0 / sumSquaredWeights;
}

// 优化1: 系统重采样 (Systematic Resampling)
int Locator::systematicResample()
{
    int rows = hypos.rows();
    if (rows <= 1) return 1;
    
    auto old_hypos = hypos;
    hypos.resize(rows, 6);
    hypos.setZero();
    
    double r = ((double)rand() / RAND_MAX) / rows;
    double c = old_hypos(0, 5);
    int i = 0;
    
    for (int m = 0; m < rows; m++)
    {
        double u = r + (double)m / rows;
        while (u > c && i < old_hypos.rows() - 1)
        {
            i++;
            c = old_hypos(i, 5);
        }
        hypos.row(m).head(3) = old_hypos.row(i).head(3);
    }
    
    // 粗化：添加小噪声防止粒子退化
    double rougheningScale = 0.01;
    Eigen::ArrayXXd noise = Eigen::ArrayXXd::Random(rows, 3) * rougheningScale;
    hypos.leftCols(3) += noise;
    
    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);
    
    return 0;
}

// 优化2: 动态粒子数调整
int Locator::getDynamicParticleCount()
{
    if (hypos.rows() <= 0) return baseParticles;
    
    double xRange = hypos.col(0).maxCoeff() - hypos.col(0).minCoeff();
    double yRange = hypos.col(1).maxCoeff() - hypos.col(1).minCoeff();
    double thetaRange = hypos.col(2).maxCoeff() - hypos.col(2).minCoeff();
    
    double normalizedUncertainty = 
        (xRange / fieldDimensions.length + 
         yRange / fieldDimensions.width + 
         thetaRange / (M_PI * 2)) / 3.0;
    
    int particleCount;
    if (normalizedUncertainty > uncertaintyThreshold)
    {
        particleCount = maxParticles;
    }
    else if (normalizedUncertainty < uncertaintyThreshold / 2.0)
    {
        particleCount = minParticles;
    }
    else
    {
        particleCount = baseParticles;
    }
    
    return particleCount;
}

// 优化3: 观测引导的粒子生成
int Locator::genObservationGuidedParticles(vector<FieldMarker> markers_r)
{
    auto old_hypos = hypos;
    int num = static_cast<int>(hypos.rows() * numShrinkRatio);
    if (num <= 0) return 1;
    
    int guidedCount = static_cast<int>(num * observationGuidedRatio);
    int randomCount = num - guidedCount;
    
    hypos.resize(num, 6);
    hypos.setZero();
    Eigen::ArrayXd rands = (Eigen::ArrayXd::Random(randomCount) + 1) / 2;
    
    // 第一部分：基于权重的随机采样
    for (int i = 0; i < randomCount; i++)
    {
        double rand = rands(i);
        int j;
        for (j = 0; j < old_hypos.rows(); j++)
        {
            if (old_hypos(j, 5) >= rand)
                break;
        }
        hypos.row(i).head(3) = old_hypos.row(j).head(3);
    }
    
    // 第二部分：观测引导采样
    if (markers_r.size() > 0 && guidedCount > 0)
    {
        Pose2D basePose = bestPose;
        double totalDx = 0, totalDy = 0;
        for (const auto& marker_r : markers_r)
        {
            auto marker_f = markerToFieldFrame(marker_r, basePose);
            auto offset = getOffset(marker_f);
            totalDx += offset[0];
            totalDy += offset[1];
        }
        totalDx /= markers_r.size();
        totalDy /= markers_r.size();
        
        Pose2D guidedPose{basePose.x + totalDx, basePose.y + totalDy, basePose.theta};
        
        for (int i = randomCount; i < num; i++)
        {
            double noise_x = ((double)rand() / RAND_MAX - 0.5) * offsetX * 0.5;
            double noise_y = ((double)rand() / RAND_MAX - 0.5) * offsetY * 0.5;
            double noise_theta = ((double)rand() / RAND_MAX - 0.5) * offsetTheta * 0.5;
            
            hypos(i, 0) = guidedPose.x + noise_x;
            hypos(i, 1) = guidedPose.y + noise_y;
            hypos(i, 2) = guidedPose.theta + noise_theta;
        }
    }
    else
    {
        Eigen::ArrayXd extraRands = (Eigen::ArrayXd::Random(guidedCount) + 1) / 2;
        for (int i = randomCount; i < num; i++)
        {
            double rand = extraRands(i - randomCount);
            int j;
            for (j = 0; j < old_hypos.rows(); j++)
            {
                if (old_hypos(j, 5) >= rand)
                    break;
            }
            hypos.row(i).head(3) = old_hypos.row(j).head(3);
        }
    }
    
    offsetX *= offsetShrinkRatio;
    offsetY *= offsetShrinkRatio;
    offsetTheta *= offsetShrinkRatio;
    
    Eigen::ArrayXXd offsets = Eigen::ArrayXXd::Random(num, 3);
    offsets.col(0) *= offsetX * 0.5;
    offsets.col(1) *= offsetY * 0.5;
    offsets.col(2) *= offsetTheta * 0.5;
    hypos.leftCols(3) += offsets;
    
    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);
    
    logParticles();
    
    return 0;
}
