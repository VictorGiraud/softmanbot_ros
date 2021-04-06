#include "vision_reza.hpp"

//================================================================================
//================================================================================
void show_array(std::vector<double>& array)
{
    auto begin{ &array[0] };
    auto end{ begin + array.size() };
    cout << "\n1D Array (" << array.size() << ") ============================" << endl;
    int i{ 0 };
    for (auto ptr{ begin }; ptr != end; ++ptr)
    {
        cout << *ptr << ' ';
    }
    cout << "\n";
}
//================================================================================
//================================================================================
void show_array_2D(std::vector<std::vector<double>>& array1)
{

    int rows{ static_cast<int>(array1.size()) };
    int cols{ static_cast<int>(array1[0].size()) };
    cout << "\n2D Array (" << rows << "," << cols << ") ============================" << endl;
    if (cols > 0)
    {
        for (int j = 0; j < rows; j++)
        {
            for (int k = 0; k < cols; k++)
            {
                cout << array1[j][k] << '\t';
            }
            cout << '\n';
        }
    }
    cout << "\n";
}
//================================================================================
//================================================================================
DoubleVector2D normalize_boundary(vector<cv::Point> contour, int num_points)
{
    int size_initial(contour.size());
    DoubleVector2D ctr(2, vector<double>(size_initial));
    for (int i = 0; i < size_initial; i++)
    {
        ctr[0][i] = contour[i].x;
        ctr[1][i] = contour[i].y;
    }
    double contour_length(arcLength(contour, true));
    double interval(contour_length / num_points);
    DoubleVector2D contour_interpolated(2, vector<double>(2 * num_points));
    contour_interpolated[0][0] = ctr[0][0];
    contour_interpolated[1][0] = ctr[1][0];
    int index(0);
    int index_ctr(1);
    double distance_with_first_node(100000);
    while (index < 2 * num_points && index_ctr < size_initial)
    {
        double distance(sqrt(pow(ctr[0][index_ctr] - contour_interpolated[0][index], 2) + pow(ctr[1][index_ctr] - contour_interpolated[1][index], 2)));
        if (distance > interval)
        {
            double cos((ctr[0][index_ctr] - contour_interpolated[0][index]) / distance);
            double sin((ctr[1][index_ctr] - contour_interpolated[1][index]) / distance);
            contour_interpolated[0][index + 1] = contour_interpolated[0][index] + cos * interval;
            contour_interpolated[1][index + 1] = contour_interpolated[1][index] + sin * interval;
            distance_with_first_node = sqrt(pow(contour_interpolated[0][index + 1] - contour_interpolated[0][0], 2) + pow(contour_interpolated[1][index + 1] - contour_interpolated[1][0], 2));
            index++;
            if (distance_with_first_node < interval && index > 1) break;
        }
        else
        {
            index_ctr++;
        }
    }

    DoubleVector2D contour_interpolated_final(2, vector<double>(index));
    for (int i = 0; i < index; i++)
    {
        contour_interpolated_final[0][i] = contour_interpolated[0][i];
        contour_interpolated_final[1][i] = contour_interpolated[1][i];
    }
    return contour_interpolated_final;
}
//================================================================================
//================================================================================
DoubleVector2D normalize_boundary(DoubleVector2D contour_v, int num_points)
{
    vector<cv::Point> contour(contour_v[0].size() + 1);
    for (int i = 0; i < contour_v[0].size(); i++)
    {
        contour[i].x = contour_v[0][i];
        contour[i].y = contour_v[1][i];
    }
    contour[contour_v[0].size()].x = contour_v[0][0];
    contour[contour_v[0].size()].y = contour_v[1][0];

    int size_initial(contour.size());
    DoubleVector2D ctr(2, vector<double>(size_initial));
    for (int i = 0; i < size_initial; i++)
    {
        ctr[0][i] = contour[i].x;
        ctr[1][i] = contour[i].y;
    }
    double contour_length(arcLength(contour, true));
    double interval(contour_length / num_points);
    DoubleVector2D contour_interpolated(2, vector<double>(2 * num_points));
    contour_interpolated[0][0] = ctr[0][0];
    contour_interpolated[1][0] = ctr[1][0];
    int index(0);
    int index_ctr(1);
    double distance_with_first_node(100000);
    while (index < 2 * num_points && index_ctr < size_initial)
    {
        double distance(sqrt(pow(ctr[0][index_ctr] - contour_interpolated[0][index], 2) + pow(ctr[1][index_ctr] - contour_interpolated[1][index], 2)));
        if (distance > interval)
        {
            double cos((ctr[0][index_ctr] - contour_interpolated[0][index]) / distance);
            double sin((ctr[1][index_ctr] - contour_interpolated[1][index]) / distance);
            contour_interpolated[0][index + 1] = contour_interpolated[0][index] + cos * interval;
            contour_interpolated[1][index + 1] = contour_interpolated[1][index] + sin * interval;
            distance_with_first_node = sqrt(pow(contour_interpolated[0][index + 1] - contour_interpolated[0][0], 2) + pow(contour_interpolated[1][index + 1] - contour_interpolated[1][0], 2));
            index++;
            if (distance_with_first_node < interval && index > 1) break;
        }
        else
        {
            index_ctr++;
        }
    }

    DoubleVector2D contour_interpolated_final(2, vector<double>(index));
    for (int i = 0; i < index; i++)
    {
        contour_interpolated_final[0][i] = contour_interpolated[0][i];
        contour_interpolated_final[1][i] = contour_interpolated[1][i];
    }

    return contour_interpolated_final;
}
//================================================================================
//================================================================================
Mat plot_points(Mat image_init, DoubleVector2D points, string window, bool line_show, bool text_show, Scalar point_clr, Scalar line_clr, Scalar string_clr, int size_image)
{
    if (!getWindowProperty(window, WND_PROP_VISIBLE)) // 0 if window is closed, 1 when it is open
    {
        namedWindow(window, cv::WINDOW_NORMAL);
        resizeWindow(window, 600, 600);
    }
    Vec3b white(255, 255, 255);
    Mat image;

    int rows = image_init.rows;
    if (rows != 0) {
        image = image_init;
    }
    else
    {
        image = Mat::ones(size_image, size_image, CV_8UC3);
        image.setTo(white);
    }

    /*double maxx;
    double minn;
    double max_points(0);
    double min_points(10000000);
    for (int i = 0; i < size(points[0]); i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (max_points > points[j][i]) max_points = points[j][i];
            if (min_points < points[j][i]) min_points = points[j][i];
        }
    }

    double image_min, image_max;
    minMaxLoc(image, &image_min, &image_max);

    if (1.1 * abs(max_points - min_points) > abs(image_max - image_min))
    {

    }*/


    int n_points(static_cast<int>(points[0].size()));
    int size_circle(size_image/250);
    int size_line(size_image / 500);
    double size_charac(size_image / 1000.0);

    /*double max_x(0);
    double max_y(0);
    double min_x(100000000);
    double min_y(100000000);
    for (int i = 0; i < n_points; i++)
    {
        if (points[0][i] > max_x) max_x = points[0][i];
        if (points[1][i] > max_y) max_y = points[1][i];
        if (points[0][i] < min_x) min_x = points[0][i];
        if (points[1][i] < min_y) min_y = points[1][i];
    }

    cout << min_x << " " << max_x << " " << min_y << " " << max_y << endl;
    if((max_x - min_x) > 500 || (max_y - min_y) > 500 )
        resize(image, image, Size(1.2*(max_x - min_x), 1.2 * (max_y - min_y)));*/

    for (int i = 0; i < n_points; i++) 
    {
        int index{ i };
        int index_next{ i + 1 };
        if (i == n_points - 1) {
            index_next = 0;
        }

        circle(image, Point2d(points[0][index], points[1][index]), size_circle, point_clr, size_circle, LINE_8, 0);
        if (line_show) line(image, Point2d(points[0][index], points[1][index]), Point2d(points[0][index_next], points[1][index_next]), line_clr, size_line, LINE_8, 0);
        if (text_show)  putText(image, to_string(i), cv::Point(points[0][index] + 5, points[1][index]), FONT_HERSHEY_COMPLEX_SMALL, size_charac, string_clr, 2* size_charac, LINE_AA);
    }
    imshow(window, image);
    return image;
}
//================================================================================
//================================================================================
void overlayImage(Mat* src, Mat* overlay, const Point& location)
{
    for (int y = max(location.y, 0); y < src->rows; ++y)
    {
        int fY = y - location.y;

        if (fY >= overlay->rows)
            break;

        for (int x = max(location.x, 0); x < src->cols; ++x)
        {
            int fX = x - location.x;

            if (fX >= overlay->cols)
                break;

            double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

            for (int c = 0; opacity > 0 && c < src->channels(); ++c)
            {
                unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
                unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
                src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
}
//================================================================================
//================================================================================
void reza_vision(DoubleVector2D& final_transformation, DoubleVector1D& left_point, DoubleVector1D& right_point, bool show_image)
{

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR);
	rs2::pipeline_profile selection = pipe.start(cfg);

	// Calibration ------------
	auto color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	auto intrinsics = color_stream.get_intrinsics();
	DoubleVector2D K{ {intrinsics.fx, 0, intrinsics.ppx}, {0, intrinsics.fy, intrinsics.ppy}, {0, 0, 1} };
	float cameraMatrix_temp[9] = { intrinsics.fx, 0., intrinsics.ppx, 0., intrinsics.fy, intrinsics.ppy, 0., 0., 1. };
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraMatrix_temp);
	float distCoeffs_temp[5] = { 0, 0, 0, 0, 0 };
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, distCoeffs_temp);

	int num_frame(0);
	bool marker_available = true;
	int Total_num_frames(10);
	while(num_frame < Total_num_frames)
    {
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::align align_to_depth(RS2_STREAM_DEPTH);
		rs2::align align_to_color(RS2_STREAM_COLOR);
		// ...
		data = align_to_color.process(data);	
		
		rs2::depth_frame depth_orig = data.get_depth_frame();
		rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
		rs2::frame color = data.get_color_frame();            // Find the color data

		// rs2::frame depth_frame_aligned = processed.first(RS2_STREAM_DEPTH).apply_filter(color_map);
		const int w = depth.as<rs2::video_frame>().get_width();
		const int h = depth.as<rs2::video_frame>().get_height();
		const int w2 = color.as<rs2::video_frame>().get_width();
		const int h2 = color.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat depth_image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		Mat color_image(Size(w2, h2), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
		cvtColor(color_image, color_image, cv::COLOR_BGR2RGB); 

		Mat depth_color= depth_image.clone();
		overlayImage(&depth_color, &color_image, Point());
		
		Mat image1;
		cvtColor(color_image, image1, COLOR_BGR2GRAY);
		Mat image1_temp1;
		vector<Point> contour;
		GaussianBlur(image1, image1_temp1, Size( 9, 9 ), 1.0);
		cv::threshold(image1_temp1, image1_temp1, 100, 255, THRESH_BINARY_INV);
		// image1_temp1 = ~image1_temp1;

		//Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours( image1_temp1, contours, RETR_LIST, CHAIN_APPROX_NONE );

		//Extracting the largest contour =======================================================
		int contour_index;
		double max_area(0.0);
		cv::Mat contourImage(image1.size(), CV_8UC3, cv::Scalar(0,0,0));
		for (int idx = 0; idx < contours.size(); idx++) 
		{
			double area = contourArea(contours[idx]);
			if(area > max_area)
			{
				max_area = area;
				contour_index= idx;
			}
		}

		DoubleVector2D contour_silhouette(normalize_boundary(contours[contour_index], 300));

		// Finding Corners ===================================================================
		DoubleVector1D slope_silhouette(contour_silhouette[0].size());
		for (int i=0;i<contour_silhouette[0].size();i++) 
		{
			int current = i;
			int next = i+1;
			int prev = i-1;
			if(next == contour_silhouette[0].size()) next = 0;
			if(prev == -1) prev = contour_silhouette[0].size()-1;

			double slope_next = atan2( (contour_silhouette[1][next] - contour_silhouette[1][current]),(contour_silhouette[0][next] - contour_silhouette[0][current]) )*180/3.14;
			double slope_prev = atan2( (contour_silhouette[1][prev] - contour_silhouette[1][current]),(contour_silhouette[0][prev] - contour_silhouette[0][current]) )*180/3.14;

			if(slope_next >= 180) slope_next -= 180;
			if(slope_prev >= 180) slope_prev -= 180;
			if(slope_next < 0) slope_next += 180;
			if(slope_prev < 0) slope_prev += 180;
			if(slope_next >= 90) slope_next = 180 - slope_next;
			if(slope_prev >= 90) slope_prev = 180 - slope_prev;

			slope_silhouette[i] = abs(slope_next - slope_prev);

			// cout << slope_prev << " " << slope_next << " " << slope_silhouette[i] << endl;
		}

		
		vector<int> sorted_indices(contour_silhouette[0].size());
		size_t n(0);
		generate(std::begin(sorted_indices), std::end(sorted_indices), [&]{ return n++; });
		sort( begin(sorted_indices), end(sorted_indices), [&](int i1, int i2) { return slope_silhouette[i1] > slope_silhouette[i2]; } );

		DoubleVector2D corners(2);
		corners[0].push_back(contour_silhouette[0][sorted_indices[0]]);
		corners[1].push_back(contour_silhouette[1][sorted_indices[0]]);
		corners[0].push_back(contour_silhouette[0][sorted_indices[1]]);
		corners[1].push_back(contour_silhouette[1][sorted_indices[1]]);
		corners[0].push_back(contour_silhouette[0][sorted_indices[2]]);
		corners[1].push_back(contour_silhouette[1][sorted_indices[2]]);
		corners[0].push_back(contour_silhouette[0][sorted_indices[3]]);
		corners[1].push_back(contour_silhouette[1][sorted_indices[3]]);

		n = 0;
		vector<int> sorted_indices_corners(corners[0].size());
		generate(std::begin(sorted_indices_corners), std::end(sorted_indices_corners), [&]{ return n++; });
		sort( begin(sorted_indices_corners), end(sorted_indices_corners), [&](int i1, int i2) { return corners[1][i1] < corners[1][i2]; } );

		DoubleVector2D lowerEdge_corners(2);
		for (int i=0; i<2; i++) 
		{
			lowerEdge_corners[0].push_back(corners[0][sorted_indices_corners[i]]);
			lowerEdge_corners[1].push_back(corners[1][sorted_indices_corners[i]]);
		}

		DoubleVector2D lowerEdge_corners_3D(3, vector<double>(2));
		float point[3];
		for (int i=0; i<2; i++) 
		{
			float pixel[2]{ static_cast<float>(lowerEdge_corners[0][i]), static_cast<float>(lowerEdge_corners[1][i]) };
			rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_orig.get_distance(lowerEdge_corners[0][i], lowerEdge_corners[1][i]));
			lowerEdge_corners_3D[0][i] = point[0];
			lowerEdge_corners_3D[1][i] = point[1];
			lowerEdge_corners_3D[2][i] = point[2];
		}

		left_point.resize(3);
		right_point.resize(3);

		for (int i=0; i<3; i++) 
		{
			left_point[i] = lowerEdge_corners_3D[i][0];
			right_point[i] = lowerEdge_corners_3D[i][1];
		}

		//====================================================================
		// Aruco Marker
		//====================================================================
		Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
		vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		Ptr<cv::aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
		//parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
		// parameters->adaptiveThreshConstant=true;	
		
		aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
		aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);
				
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
		
		DoubleVector2D transformation(4, vector<double>(4));
		if(rvecs.size() > 0)
		{
			auto rvec = rvecs[0];
			auto tvec = tvecs[0];
			cv::aruco::drawAxis(color_image, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
			// cout << tvec << endl;
			Mat rot_mat;
			Rodrigues(rvec, rot_mat);
			// cout << rot_mat << endl;
			transformation[0][0] = rot_mat.at<double>(0,0);
			transformation[0][1] = rot_mat.at<double>(0,1);
			transformation[0][2] = rot_mat.at<double>(0,2);
			transformation[0][3] = tvec[0];
			transformation[1][0] = rot_mat.at<double>(1,0);
			transformation[1][1] = rot_mat.at<double>(1,1);
			transformation[1][2] = rot_mat.at<double>(1,2);
			transformation[1][3] = tvec[1];
			transformation[2][0] = rot_mat.at<double>(2,0);
			transformation[2][1] = rot_mat.at<double>(2,1);
			transformation[2][2] = rot_mat.at<double>(2,2);
			transformation[2][3] = tvec[2];
			transformation[3][0] = 0;
			transformation[3][1] = 0;
			transformation[3][2] = 0;
			transformation[3][3] = 1;
		}
		else
		{
			marker_available = false;
		}

		final_transformation = transformation;

		//====================================================================
		// Plotting
		//====================================================================
		if(show_image && num_frame==Total_num_frames-1)
		{
			// namedWindow("color", cv::WINDOW_NORMAL);
			// resizeWindow("color", 600, 600);
			namedWindow("contour", cv::WINDOW_NORMAL);
			resizeWindow("contour", 600, 600);

			// imshow("depth", depth_image);	
			// imshow("result1", depth_color);
			// imshow("binary", image1_temp1);
			plot_points(color_image, contour_silhouette, "contour", false, false, Scalar(0, 0, 255), Scalar(0, 0, 255), Scalar(0, 255, 0), 1000);
			// plot_points(color_image, corners, "contour", false, true, Scalar(255, 0, 0), Scalar(255, 0, 0), Scalar(255, 0, 0), 1000);
			plot_points(color_image, lowerEdge_corners, "contour", false, false, Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(255, 0, 0), 1000);
			
			waitKey(0);
		}

		num_frame++;
	}

	if(!marker_available) 
		cout << "No marker is detected!" << endl;

	// if(show_image)
	// 	waitKey(0);
}//================================================================================
//================================================================================
#ifdef STANDALONE
int main(int argc, char* argv[])
{

	// vector<double> left_point;
	// nvector<double> right_point;
	// nDoubleVector2D final_transformation;
	// nreza_vision(final_transformation, left_point, right_point, false);

	// nshow_array(left_point);
	// nshow_array(right_point);
	// nshow_array_2D(final_transformation);

	// namedWindow("color", cv::WINDOW_NORMAL);
	// resizeWindow("color", 600, 600);
	// namedWindow("contour", cv::WINDOW_NORMAL);
	// resizeWindow("contour", 600, 600);

    // // Declare depth colorizer for pretty visualization of depth data
    // rs2::colorizer color_map;

    // // Declare RealSense pipeline, encapsulating the actual device and sensors
    // rs2::pipeline pipe;
	// rs2::config cfg;
	// cfg.enable_stream(RS2_STREAM_DEPTH);
	// cfg.enable_stream(RS2_STREAM_COLOR);
	// rs2::pipeline_profile selection = pipe.start(cfg);

	// // Calibration ------------
	// auto color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	// auto intrinsics = color_stream.get_intrinsics();
	// DoubleVector2D K{ {intrinsics.fx, 0, intrinsics.ppx}, {0, intrinsics.fy, intrinsics.ppy}, {0, 0, 1} };

	// float cameraMatrix_temp[9] = { intrinsics.fx, 0., intrinsics.ppx, 0., intrinsics.fy, intrinsics.ppy, 0., 0., 1. };
	// cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, cameraMatrix_temp);
	// float distCoeffs_temp[5] = { 0, 0, 0, 0, 0 };
	// cv::Mat distCoeffs = cv::Mat(1, 5, CV_32F, distCoeffs_temp);

	// Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    // while(true)
    // {
    //     rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
	// 	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	// 	rs2::align align_to_color(RS2_STREAM_COLOR);
	// 	// ...
	// 	data = align_to_color.process(data);	
		
	// 	rs2::depth_frame depth_orig = data.get_depth_frame();
    //     rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
    //     rs2::frame color = data.get_color_frame();            // Find the color data

	// 	// rs2::frame depth_frame_aligned = processed.first(RS2_STREAM_DEPTH).apply_filter(color_map);
	// 	const int w = depth.as<rs2::video_frame>().get_width();
	// 	const int h = depth.as<rs2::video_frame>().get_height();
	// 	const int w2 = color.as<rs2::video_frame>().get_width();
    // 	const int h2 = color.as<rs2::video_frame>().get_height();

	// 	// Create OpenCV matrix of size (w,h) from the colorized depth data
	// 	Mat depth_image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
	// 	Mat color_image(Size(w2, h2), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
	// 	cvtColor(color_image, color_image, cv::COLOR_BGR2RGB); 

	// 	Mat depth_color= depth_image.clone();
	// 	overlayImage(&depth_color, &color_image, Point());
		
	// 	Mat image1;
	// 	cvtColor(color_image, image1, COLOR_BGR2GRAY);
	// 	Mat image1_temp1;
	// 	vector<Point> contour;
	// 	GaussianBlur(image1, image1_temp1, Size( 9, 9 ), 1.0);
	// 	cv::threshold(image1_temp1, image1_temp1, 100, 255, THRESH_BINARY_INV);
	// 	// image1_temp1 = ~image1_temp1;

	// 	//Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
	// 	std::vector<std::vector<cv::Point> > contours;
	// 	cv::findContours( image1_temp1, contours, RETR_LIST, CHAIN_APPROX_NONE );

	// 	//Extracting the largest contour =======================================================
	// 	int contour_index;
	// 	double max_area(0.0);
	// 	cv::Mat contourImage(image1.size(), CV_8UC3, cv::Scalar(0,0,0));
	// 	for (int idx = 0; idx < contours.size(); idx++) 
	// 	{
	// 		double area = contourArea(contours[idx]);
	// 		if(area > max_area)
	// 		{
	// 			max_area = area;
	// 			contour_index= idx;
	// 		}
	// 	}

	// 	DoubleVector2D contour_silhouette(normalize_boundary(contours[contour_index], 300));

	// 	// Finding Corners ===================================================================
	// 	DoubleVector1D slope_silhouette(contour_silhouette[0].size());
	// 	for (int i=0;i<contour_silhouette[0].size();i++) 
	// 	{
	// 		int current = i;
	// 		int next = i+1;
	// 		int prev = i-1;
	// 		if(next == contour_silhouette[0].size()) next = 0;
	// 		if(prev == -1) prev = contour_silhouette[0].size()-1;

	// 		double slope_next = atan2( (contour_silhouette[1][next] - contour_silhouette[1][current]),(contour_silhouette[0][next] - contour_silhouette[0][current]) )*180/3.14;
	// 		double slope_prev = atan2( (contour_silhouette[1][prev] - contour_silhouette[1][current]),(contour_silhouette[0][prev] - contour_silhouette[0][current]) )*180/3.14;

	// 		if(slope_next >= 180) slope_next -= 180;
	// 		if(slope_prev >= 180) slope_prev -= 180;
	// 		if(slope_next < 0) slope_next += 180;
	// 		if(slope_prev < 0) slope_prev += 180;
	// 		if(slope_next >= 90) slope_next = 180 - slope_next;
	// 		if(slope_prev >= 90) slope_prev = 180 - slope_prev;

	// 		slope_silhouette[i] = abs(slope_next - slope_prev);

	// 		// cout << slope_prev << " " << slope_next << " " << slope_silhouette[i] << endl;
	// 	}

		
    // 	vector<int> sorted_indices(contour_silhouette[0].size());
    // 	size_t n(0);
    // 	generate(std::begin(sorted_indices), std::end(sorted_indices), [&]{ return n++; });
    // 	sort( begin(sorted_indices), end(sorted_indices), [&](int i1, int i2) { return slope_silhouette[i1] > slope_silhouette[i2]; } );

	// 	DoubleVector2D corners(2);
	// 	corners[0].push_back(contour_silhouette[0][sorted_indices[0]]);
	// 	corners[1].push_back(contour_silhouette[1][sorted_indices[0]]);
	// 	corners[0].push_back(contour_silhouette[0][sorted_indices[1]]);
	// 	corners[1].push_back(contour_silhouette[1][sorted_indices[1]]);
	// 	corners[0].push_back(contour_silhouette[0][sorted_indices[2]]);
	// 	corners[1].push_back(contour_silhouette[1][sorted_indices[2]]);
	// 	corners[0].push_back(contour_silhouette[0][sorted_indices[3]]);
	// 	corners[1].push_back(contour_silhouette[1][sorted_indices[3]]);

	// 	n = 0;
	// 	vector<int> sorted_indices_corners(corners[0].size());
    // 	generate(std::begin(sorted_indices_corners), std::end(sorted_indices_corners), [&]{ return n++; });
    // 	sort( begin(sorted_indices_corners), end(sorted_indices_corners), [&](int i1, int i2) { return corners[1][i1] < corners[1][i2]; } );

	// 	DoubleVector2D lowerEdge_corners(2);
	// 	for (int i=0; i<2; i++) 
	// 	{
	// 		lowerEdge_corners[0].push_back(corners[0][sorted_indices_corners[i]]);
	// 		lowerEdge_corners[1].push_back(corners[1][sorted_indices_corners[i]]);
	// 	}

	// 	DoubleVector2D lowerEdge_corners_3D(3, vector<double>(2));
	// 	float point[3];
	// 	for (int i=0; i<2; i++) 
	// 	{
	// 		float pixel[2]{ lowerEdge_corners[0][i], lowerEdge_corners[1][i] };
	// 		rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_orig.get_distance(lowerEdge_corners[0][i], lowerEdge_corners[1][i]));
	// 		lowerEdge_corners_3D[0][i] = point[0];
	// 		lowerEdge_corners_3D[1][i] = point[1];
	// 		lowerEdge_corners_3D[2][i] = point[2];
	// 	}

	// 	show_array_2D(lowerEdge_corners_3D);
	// 	cout << depth_orig.get_distance(lowerEdge_corners[0][0], lowerEdge_corners[1][0]) << endl;
	// 	cout << "=================" << endl;

	// 	//====================================================================
	// 	// Aruco Marker
	// 	//====================================================================

	// 	vector<int> markerIds;
	// 	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	// 	Ptr<cv::aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	// 	parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
	// 	// parameters->adaptiveThreshConstant=true;	
		
	// 	aruco::detectMarkers(color_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
	// 	aruco::drawDetectedMarkers(color_image, markerCorners, markerIds);
				
	// 	std::vector<cv::Vec3d> rvecs, tvecs;
	// 	cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
		
	// 	DoubleVector2D transformation(4, vector<double>(4));
	// 	if(rvecs.size() > 0)
	// 	{
	// 		auto rvec = rvecs[0];
	// 		auto tvec = tvecs[0];
	// 		cv::aruco::drawAxis(color_image, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
	// 		// cout << tvec << endl;
	// 		Mat rot_mat;
    //  		Rodrigues(rvec, rot_mat);
	// 		// cout << rot_mat << endl;
	// 		transformation[0][0] = rot_mat.at<double>(0,0);
	// 		transformation[0][1] = rot_mat.at<double>(0,1);
	// 		transformation[0][2] = rot_mat.at<double>(0,2);
	// 		transformation[0][3] = tvec[0];
	// 		transformation[1][0] = rot_mat.at<double>(1,0);
	// 		transformation[1][1] = rot_mat.at<double>(1,1);
	// 		transformation[1][2] = rot_mat.at<double>(1,2);
	// 		transformation[1][3] = tvec[1];
	// 		transformation[2][0] = rot_mat.at<double>(2,0);
	// 		transformation[2][1] = rot_mat.at<double>(2,1);
	// 		transformation[2][2] = rot_mat.at<double>(2,2);
	// 		transformation[2][3] = tvec[2];
	// 		transformation[3][0] = 0;
	// 		transformation[3][1] = 0;
	// 		transformation[3][2] = 0;
	// 		transformation[3][3] = 1;
	// 	}

	// 	// show_array_2D(transformation);
		
	// 	//====================================================================
	// 	// Plotting
	// 	//====================================================================

	// 	imshow("depth", depth_image);	
	// 	imshow("result1", depth_color);

	// 	imshow("binary", image1_temp1);
	// 	plot_points(color_image, contour_silhouette, "contour", false, false, Scalar(0, 0, 255), Scalar(0, 0, 255), Scalar(0, 255, 0), 1000);
	// 	// plot_points(color_image, corners, "contour", false, true, Scalar(255, 0, 0), Scalar(255, 0, 0), Scalar(255, 0, 0), 1000);
	// 	plot_points(color_image, lowerEdge_corners, "contour", false, false, Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(255, 0, 0), 1000);
		
	// 	waitKey(1);
    // }

    return 0;  

}
#endif
