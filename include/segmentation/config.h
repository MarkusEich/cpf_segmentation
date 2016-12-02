/*
 * Copyright (C) 2016, Australian Centre for Robotic Vision, ACRV
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 11.07.2016
 *
 *      Authors:
 *         Trung T. Pham <trung.pham@adelaide.edu.au>
 *         Markus Eich <markus.eich@qut.edu.au>
 *
 * Last update: 20 Oct 2016
 */


#ifndef CONFIG
#define CONFIG

#endif // CONFIG

namespace APC {

class Config{
public:

    Config():voxel_resolution(0.01f),
        seed_resolution(0.1f),
        color_importance (1.0f),
        spatial_importance (0.4f),
        normal_importance  (1.0f),
        use_single_cam_transform (false),
        use_supervoxel_refinement (false),

        // Default parameters for model fitting
        use_random_sampling (false),
        noise_threshold(0.02f),
        smooth_cost (0.001),
        min_inliers_per_plane (100),
	min_plane_area(0.025),
        max_num_iterations (25),
        max_curvature (0.01f),
        gc_scale (1e3){}

    public:

    float voxel_resolution;
    float seed_resolution;
    float color_importance;
    float spatial_importance;
    float normal_importance;
    bool use_single_cam_transform;
    bool use_supervoxel_refinement;

    // Default parameters for model fitting
    bool use_random_sampling;
    float noise_threshold;
    float smooth_cost;
    int min_inliers_per_plane;
    float min_plane_area;	
    float label_cost;
    int max_num_iterations;
    float max_curvature;
    int gc_scale;


    Config& operator=(const Config& config){

        voxel_resolution=config.voxel_resolution;
        seed_resolution=config.seed_resolution;
        color_importance=config.color_importance;
        spatial_importance=config.spatial_importance;
        normal_importance=config.normal_importance;
        use_single_cam_transform=config.use_single_cam_transform;
        use_supervoxel_refinement=config.use_supervoxel_refinement;

        use_random_sampling=config.use_random_sampling;
        noise_threshold=config.noise_threshold;
        smooth_cost=config.smooth_cost;
        min_inliers_per_plane=config.min_inliers_per_plane;
        min_plane_area=config.min_plane_area;
        label_cost=config.label_cost;
        max_num_iterations=config.max_num_iterations;
        max_curvature=config.max_curvature;
        gc_scale=config.gc_scale;

    }

};

}

