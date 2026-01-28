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
    // 根据 ESS 决定是否需要重采样
    bool needResample = true;
    if (ess > 0 && ess > essThreshold * old_hypos.rows())
    {
        // ESS 足够高，不需要重采样。直接复制父粒子（防止粒子贫化）
        // 如果新粒子数 != 旧粒子数，这里需要处理一下。
        // 为了简单，我们只在粒子数不变或成比例缩放时做简单的 Index Mapping
        // 或者是随机抽取但不根据权重（即认为通过了的粒子权重都差不多）
        // 但最安全的是：如果 ESS 高，我们依然做 Systemtic Resampling 或者
        // 更简单的：直接过，不做权重轮盘赌
        needResample = false;
    }
    
    // 强制重采样的情况：粒子数变化太大，或者没有提供 ESS
    if (num != old_hypos.rows()) needResample = true;

    if (!needResample)
    {
        // 直接复制（跳过 Selection），保留多样性
        // 注意：这里假设 num <= old_hypos.rows()，我们直接取前 num 个
        // 或者随机取 num 个但不看权重
        for (int i = 0; i < randomCount; i++)
        {
            // 简单地循环复制，或者随机取索引
            int idx = i % old_hypos.rows(); 
            hypos.row(i).head(3) = old_hypos.row(idx).head(3);
        }
    }
    else
    {
        // 需要重采样：使用低方差采样 (Systematic Resampling)
        // 这比原来的随机轮盘赌 (Random Sampling) 更好，方差更小
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
