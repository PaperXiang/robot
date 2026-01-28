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

// 优化3: 观测引导的粒子生成 + 自适应重采样
int Locator::genObservationGuidedParticles(vector<FieldMarker> markers_r, double ess)
{
    auto old_hypos = hypos;
    // 动态调整粒子数逻辑：如果 hypos 行数发生变化，需要强制重采样
    int num = static_cast<int>(hypos.rows() * numShrinkRatio);
    
    // 如果想要动态调整粒子数，这里可以根据 getDynamicParticleCount() 来决定 num
    // 但保持简单，这里沿用原有的 numShrinkRatio 逻辑，或者如果需要动态调整：
    // int dynamicNum = getDynamicParticleCount();
    // if (dynamicNum != hypos.rows()) num = dynamicNum; 
    
    if (num <= 0) return 1;
    
    int guidedCount = static_cast<int>(num * observationGuidedRatio);
    int randomCount = num - guidedCount;
    
    hypos.resize(num, 6);
    hypos.setZero();
    
    // --- 步骤 1: 选择/重采样 (Selection) ---
    // 注意：虽然实现了自适应重采样逻辑，但由于 locator.cpp 中的 calcProbs 函数
    // 在计算概率时没有乘以上一时刻的权重 (即假设了每次迭代开始时权重都是均为 1/N)，
    // 因此我们必须在每次迭代都进行重采样(Selection)，以体现粒子的优胜劣汰。
    // 如果跳过重采样，会导致高权重粒子的优势无法传递到下一代，引起定位不稳定。
    // 因此，这里强制使用 Systemtic Resampling (低方差重采样)，它比随机重采样更好。
    
    // 强制执行重采样
    {
        // 使用低方差采样 (Systematic Resampling)
        double r = ((double)rand() / RAND_MAX) / randomCount; // 注意分母是 randomCount
        double c = old_hypos(0, 5);
        int i = 0;
        
        for (int m = 0; m < randomCount; m++)
        {
            double u = r + (double)m / randomCount;
            while (u > c && i < old_hypos.rows() - 1)
            {
                i++;
                c = old_hypos(i, 5);
            }
            hypos.row(m).head(3) = old_hypos.row(i).head(3);
        }
    }
    
    
    // --- 步骤 2: 观测引导 (Observation Guidance) ---
    // 这部分粒子不从旧粒子中重采样，而是直接从观测生成
    if (markers_r.size() > 0 && guidedCount > 0)
    {
        Pose2D basePose = bestPose;
        double totalDx = 0, totalDy = 0;
        int validMarkerCount = 0;
        
        for (const auto& marker_r : markers_r)
        {
            auto marker_f = markerToFieldFrame(marker_r, basePose);
            auto offset = getOffset(marker_f);
            
            // Outlier Rejection
            double dist = sqrt(offset[0]*offset[0] + offset[1]*offset[1]);
            if (dist > 1.0) continue; 
            
            totalDx += offset[0];
            totalDy += offset[1];
            validMarkerCount++;
        }

        if (validMarkerCount > 0)
        {
            totalDx /= validMarkerCount;
            totalDy /= validMarkerCount;
            
            // Damping (0.5)
            double dampingFactor = 0.5;
            Pose2D guidedPose{basePose.x + totalDx * dampingFactor, basePose.y + totalDy * dampingFactor, basePose.theta};
            
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
             // Fallback: 如果没有有效观测，这部分粒子也从旧粒子重采样
             // 使用随机采样填充剩余部分
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
    }
    else
    {
        // 没有观测开启，全部用旧粒子填充
        // 使用随机采样（或者如果之前是 Systematic，这里继续 Systematic 可能比较麻烦，简单用 Random）
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
    
    // --- 步骤 3: 变异 (Mutation / Prediction) ---
    // 对所有粒子加噪声（第一部分粒子也要加，第二部分引导粒子已经在生成时加了，但再加一次也无妨，或者分开处理）
    // 注意：上面的代码中，引导粒子已经加了 noise。
    // 所以只需要对前 randomCount 个粒子加 Mutation 噪声
    
    offsetX *= offsetShrinkRatio;
    offsetY *= offsetShrinkRatio;
    offsetTheta *= offsetShrinkRatio;
    
    // 只对非引导粒子加噪声（引导粒子是新生成的，自带位置）
    // 但原来的代码是全部加。我们这里为了保持一致性，还是全部加？
    // 不，引导粒子已经在 guidedPose 基础上加了 noise。如果在下面再加一次，方差会变大。
    // 但是 offsets 是根据 shrinking ratio 来的。
    
    Eigen::ArrayXXd offsets = Eigen::ArrayXXd::Random(num, 3);
    offsets.col(0) *= offsetX * 0.5;
    offsets.col(1) *= offsetY * 0.5;
    offsets.col(2) *= offsetTheta * 0.5;
    
    // 只应用给非引导部分 (0 ~ randomCount)
    // 或者如果 markers_r 为空，则是全部
    int mutationEndIndex = (markers_r.size() > 0 && guidedCount > 0) ? randomCount : num;
    
    hypos.block(0, 0, mutationEndIndex, 3) += offsets.block(0, 0, mutationEndIndex, 3);
    
    // 约束检查
    hypos.col(0) = hypos.col(0).cwiseMax(constraints.xmin).cwiseMin(constraints.xmax);
    hypos.col(1) = hypos.col(1).cwiseMax(constraints.ymin).cwiseMin(constraints.ymax);
    hypos.col(2) = hypos.col(2).cwiseMax(constraints.thetamin).cwiseMin(constraints.thetamax);
    
    logParticles();
    
    return 0;
}
