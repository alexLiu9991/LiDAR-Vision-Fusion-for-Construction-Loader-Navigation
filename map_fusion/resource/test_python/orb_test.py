#!/usr/bin/env python 
# -*- coding: utf-8 -*- 
# author：lowkeyway time:11/13/2019 

import sys 
import cv2 as cv 
import numpy as np 

def main_func(argv): 
    # 读取图像
    imgLidarMap = cv.imread("lslidar_cartographer_map.png") 
    imgDepthMap = cv.imread("depthimage_cartorgrapher_map.png") 
    
    if imgLidarMap is None or imgDepthMap is None:
        print("Error: Could not load one or both images")
        return
    
    # ORB参数配置
    orb_params = {
        'nfeatures': 1000,        # 特征点数量
        'scaleFactor': 2.2,      # 金字塔缩放因子
        'nlevels': 8,            # 金字塔层数
        'edgeThreshold': 31,     # 边缘阈值
        'firstLevel': 0,         # 第一层级别
        'WTA_K': 2,              # 产生描述子时每个点的邻域数
        'scoreType': cv.ORB_HARRIS_SCORE,  # 评分类型
        'patchSize': 31          # 描述子补丁大小
    }
    
    # 创建ORB检测器
    orb = cv.ORB_create(
        nfeatures=orb_params['nfeatures'],
        scaleFactor=orb_params['scaleFactor'], 
        nlevels=orb_params['nlevels'],
        edgeThreshold=orb_params['edgeThreshold'],
        firstLevel=orb_params['firstLevel'],
        WTA_K=orb_params['WTA_K'],
        scoreType=orb_params['scoreType'],
        patchSize=orb_params['patchSize']
    )
    
    # 检测特征点和计算描述子
    kpLidarMap, desLidarMap = orb.detectAndCompute(imgLidarMap, None) 
    kpDepthMap, desDepthMap = orb.detectAndCompute(imgDepthMap, None) 
    
    print(f"LiDAR map features: {len(kpLidarMap)}")
    print(f"Depth map features: {len(kpDepthMap)}")
    
    if desLidarMap is None or desDepthMap is None:
        print("Error: Could not compute descriptors")
        return
    
    # 匹配参数
    match_params = {
        'cross_check': True,           # 交叉检查
        'ratio_threshold': 0.7,        # Lowe's ratio test阈值
        'min_match_count': 10,         # 最小匹配数量
        'ransac_threshold': 5.0,       # RANSAC阈值
        'max_iterations': 1000         # RANSAC最大迭代次数
    }
    
    # 创建BF匹配器
    bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=match_params['cross_check']) 
    
    # 进行匹配
    if match_params['cross_check']:
        # 直接匹配（已启用交叉检查）
        matches = bf.match(desLidarMap, desDepthMap)
        matches = sorted(matches, key=lambda x: x.distance)
        print(f"Initial matches: {len(matches)}")
    else:
        # knnMatch + Lowe's ratio test
        matches_raw = bf.knnMatch(desLidarMap, desDepthMap, k=2)
        matches = []
        for match_pair in matches_raw:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < match_params['ratio_threshold'] * n.distance:
                    matches.append(m)
        print(f"Matches after ratio test: {len(matches)}")
    
    # 筛选好的匹配
    if len(matches) >= match_params['min_match_count']:
        # 提取匹配点坐标
        src_pts = np.float32([kpLidarMap[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kpDepthMap[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
        
        # 使用RANSAC估计单应性矩阵
        homography, mask = cv.findHomography(
            src_pts, 
            dst_pts, 
            cv.RANSAC, 
            match_params['ransac_threshold'],
            maxIters=match_params['max_iterations']
        )
        
        if homography is not None:
            # 筛选内点
            inlier_matches = [matches[i] for i in range(len(matches)) if mask[i]]
            print(f"Inlier matches: {len(inlier_matches)}")
            
            # 绘制匹配结果
            matchImg = cv.drawMatches(
                imgLidarMap, kpLidarMap, 
                imgDepthMap, kpDepthMap, 
                inlier_matches, None,
                matchColor=(0, 255, 0),      # 绿色表示好的匹配
                singlePointColor=(255, 0, 0), # 红色表示单独的点
                flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
            )
            
            # 计算匹配质量指标
            match_ratio = len(inlier_matches) / len(matches) if len(matches) > 0 else 0
            print(f"Match quality ratio: {match_ratio:.2f}")
            
        else:
            print("Could not find valid homography")
            matchImg = cv.drawMatches(imgLidarMap, kpLidarMap, imgDepthMap, kpDepthMap, matches[:50], None)
    else:
        print(f"Not enough matches found: {len(matches)}/{match_params['min_match_count']}")
        matchImg = cv.drawMatches(imgLidarMap, kpLidarMap, imgDepthMap, kpDepthMap, matches, None)
    
    # 显示结果
    cv.imshow("LiDAR Map", imgLidarMap) 
    cv.imshow("Depth Map", imgDepthMap) 
    cv.imshow('Feature Matches', matchImg) 
    
    # 显示特征点
    img_kp_lidar = cv.drawKeypoints(imgLidarMap, kpLidarMap, None, color=(0, 255, 0))
    img_kp_depth = cv.drawKeypoints(imgDepthMap, kpDepthMap, None, color=(0, 255, 0))
    cv.imshow('LiDAR Keypoints', img_kp_lidar)
    cv.imshow('Depth Keypoints', img_kp_depth)
    
    cv.waitKey(0) 
    cv.destroyAllWindows()

if __name__ == '__main__': 
    main_func(sys.argv) 
