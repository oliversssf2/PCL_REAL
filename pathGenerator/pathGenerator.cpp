//
// Created by fongsu on 3/21/19.
//

#include <pcl/filters/statistical_outlier_removal.h>
#include "pathGenerator.h"

pathGenerator::pathGenerator() : align_to_color(rs2::align(RS2_STREAM_COLOR)) {
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480);

	//align_to_color = rs2::align(RS2_STREAM_COLOR);

	pipe_profile = pipe.start(cfg);
	std::string t{"dist.csv"};
	distFileName = DISTOUTPUTPREFIX + t;
	distfile.open(distFileName, std::ios::out);
}

pathGenerator::~pathGenerator() {
	distfile.close();
	pipe.stop();
}

void pathGenerator::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(passfield);
    pass.setFilterLimits(limitMin, limitMax);
    pass.filter(*output);
    std::cout << "============================passthrough=========================" << std::endl;
    for (auto x : output->points)
        std::cout << x << std::endl;
}

void pathGenerator::StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> stat_sort;
    stat_sort.setInputCloud(input);
    stat_sort.setMeanK(MeanK);
    stat_sort.setStddevMulThresh(StddevMulThresh);
    stat_sort.filter(*output);
    std::cout << "============================stat=========================" << std::endl;
    for (auto x : output->points)
        std::cout << x << std::endl;
}

void VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setInputCloud(input);
	grid.setLeafSize(0.05f, 0.05f, 0.05f);
	grid.filter(*output);
}

void pathGenerator::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointNormal>::Ptr output, pcl::PointCloud<pcl::PointXYZ>::Ptr searchSurface) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(input);
    ne.setSearchSurface(searchSurface);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(searchRadius);
    ne.compute(*output);

    pcl::copyPointCloud(*input, *output);

    std::cout << "============================norm=========================" << std::endl;
    for (auto x : output->points)
        std::cout << x << std::endl;

}

void pathGenerator::NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr input,pcl::PointCloud<pcl::PointNormal>::Ptr output) {
    NormalEstimation(input, output, input);
}

void pathGenerator::NaNRemoval(pcl::PointCloud<pcl::PointNormal>::Ptr input,pcl::PointCloud<pcl::PointNormal>::Ptr output) {
    std::vector<int> indices_p; //NaN point indices
    std::vector<int> indices_n; //NaN normal indices

    pcl::PointCloud<pcl::PointNormal>::Ptr tem (new pcl::PointCloud<pcl::PointNormal>);
    pcl::removeNaNFromPointCloud(*input, *tem, indices_p);
    pcl::removeNaNNormalsFromPointCloud(*tem, *output, indices_n);
    std::cout << "============================nan=========================" << std::endl;
    for (auto x : output->points)
        std::cout << x << std::endl;

}

void pathGenerator::Reorganize(pcl::PointCloud<pcl::PointNormal>::Ptr input) {
    std::vector<size_t> idx(input->width);
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&input](size_t i, size_t i2){ return (input->points.at(i).x < input->points.at(i2).x);});
    for(auto v : idx)
        std::cout << input->points.at(v) << std::endl;
    auto it_tgt = idx.begin();
    for(auto it_flag = idx.begin(); it_flag != idx.end(); it_flag++)
    {
        if((input->points.at(*it_flag).x - input->points.at(*it_tgt).x) > reorganizeRange)
        {
            std::sort(it_tgt, it_flag, [&input](size_t i, size_t i2){ return (input->points.at(i).y < input->points.at(i2).y);});
            it_tgt = it_flag;
        }
    }
    for(auto v : idx)
        std::cout << input->points.at(v) << std::endl;
    data.indices_order.push_back(idx);
}

void pathGenerator::Exaggerate(pcl::PointCloud<pcl::PointNormal>::Ptr input) {
	for(auto &p : input->points)
	{

	}
}

void pathGenerator::updateSettings(){
    std::cout << "use defalut settings?" << std::endl;

    std::cout << "downsample: " << downsample << std::endl;
    std::cout << "passthrough Max Limit: " << limitMax << std::endl;
    std::cout << "passthrough Min Limit: " << limitMin << std::endl;
    std::cout << "passthrough field: " << passfield << std::endl;
    std::cout << "MeanK: " << MeanK << std::endl;
    std::cout << "StddevMulThresh: " << StddevMulThresh << std::endl;
    std::cout << "NormalSearchRadius: " << searchRadius << std::endl;
    std::cout << "ReorganizeRange: " << reorganizeRange << std::endl;

    std::cout << "\nChange Settings? 'y' for yes 'n' for no" << std::endl;
    char c;
    scanf("%c", &c);
    if(c == 'y')
    {
        std::cout << "setPassLimit(float _limitMax, float _limitMin, std::string _passfield): ";
        float i, j;
        std::string k;
        std::cin >> i >> j >> k;
        setPassLimit(i, j, k);
        std::cin.ignore();
        std::cout << std::endl;

        std::cout << "setDownsample(int _downsample): ";
        int l;
        std::cin >> l;
        setDownsample(l);
        std::cin.ignore();
        std::cout << std::endl;

        std::cout << "setStatOutRem(int _MeanK, float _StddevMulThresh): ";
        int m;
        float n;
        std::cin >> m >> n;
        setStatOutRem(m, n);
        std::cin.ignore();
        std::cout << std::endl;

        std::cout << "setNormalEst(float _searchRadius): ";
        float o;
        std::cin >> o;
        setNormalEst(o);
        std::cin.ignore();
        std::cout << std::endl;

        std::cout << "setReorganizeRange(float _reorganizeRange): ";
        float p;
        std::cin >> p;
        setNormalEst(p);
        std::cin.ignore();
        std::cout << std::endl;
    } else
        std::cout << "using default settings now" << std::endl;

}


void pathGenerator::Gen_compute() {
    frames = pipe.wait_for_frames();
	frames = align_to_color.process(frames);
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);

    ++data.count;
    PCDPointNormal m;
    std::string str = "cloud_" + std::to_string(data.count-1); //give the Pointcloud a name
    m.f_name = str;
    m.index = data.count-1;

	pcl::PointCloud<pcl::PointXYZ>::Ptr original = points_to_pcl(points, downsample);
    data.original_Clouds.push_back(original);

    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_1 = points_to_pcl(points, downsample);
    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr stage_3 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr stage_4 (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr stage_5 (new pcl::PointCloud<pcl::PointNormal>);

    PassThrough(stage_1, stage_2);
    StatisticalOutlierRemoval(stage_2, stage_3);
    NormalEstimation(stage_3, stage_4, original);
    NaNRemoval(stage_4, m.cloud);
    Reorganize(m.cloud);

    data.processed_Clouds.push_back(m);
    savePointNormal(m, false, true, data.indices_order.back());
}

rs2::frameset pathGenerator::wait_for_frames() {
	frames = pipe.wait_for_frames();
	return frames;
}

float pathGenerator::midDist() {
	if (frameCnt % captureRate == 0) {
		rs2::depth_frame depth = frames.get_depth_frame();
		float ret = depth.get_distance(depth.get_width() / 2, depth.get_height() / 2);
		distfile << ret << ',' << std::endl;
		return ret;
	}
	++frameCnt;
}

