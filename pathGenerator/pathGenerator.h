//
// Created by fongsu on 3/21/19.
//

#ifndef PCL_REAL_PATHGENERATOR_H
#define PCL_REAL_PATHGENERATOR_H

#include <librealsense2/rs.hpp>


class pathGenerator {
public:

private:
    rs2::pipeline pipe;
    rs2::pointcloud points;
    rs2::pipeline_profile pipe_profile;
    std::vector<PCDPointNormal> Clouds;

};


#endif //PCL_REAL_PATHGENERATOR_H
