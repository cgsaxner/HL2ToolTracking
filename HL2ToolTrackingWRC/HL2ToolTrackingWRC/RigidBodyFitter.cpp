#include "pch.h"

#define DBG_ENABLE_INFORMATIONAL_LOGGING 1
#define DBG_ENABLE_TIMING_LOGGING 0
#define DBG_ENABLE_VERBOSE_LOGGING 0

RigidBodyFitter::RigidBodyFitter(
	cv::Mat fMat,
	cv::Mat lfProj,
	cv::Mat rfProj) :
	m_fMat(fMat), 
	m_lfProj(lfProj), 
	m_rfProj(rfProj)
{

}

void RigidBodyFitter::Triangulate(
	_In_ std::vector<cv::Point2f> lfPoints,
	_In_ std::vector<cv::Point2f> rfPoints,
	_Out_ cv::Mat& lfCameraPoints)
{
	lfCameraPoints = cv::Mat((int)lfPoints.size(), 3, CV_32F);

	std::vector<float> triangCoordsX, triangCoordsY, triangCoordsZ, triangCoordsW;
	cv::Mat lfPoints4D;

	cv::correctMatches(m_fMat, lfPoints, rfPoints, lfPoints, rfPoints);
	cv::triangulatePoints(m_lfProj, m_rfProj, lfPoints, rfPoints, lfPoints4D);

	lfPoints4D.row(0).copyTo(triangCoordsX);
	lfPoints4D.row(1).copyTo(triangCoordsY);
	lfPoints4D.row(2).copyTo(triangCoordsZ);
	lfPoints4D.row(3).copyTo(triangCoordsW);

	for (int i = 0; i < lfPoints4D.cols; i++)
	{
		lfCameraPoints.row(i).at<float>(0) = triangCoordsX[i] / triangCoordsW[i];
		lfCameraPoints.row(i).at<float>(1) = triangCoordsY[i] / triangCoordsW[i];
		lfCameraPoints.row(i).at<float>(2) = triangCoordsZ[i] / triangCoordsW[i];
	}
}

void RigidBodyFitter::FitRigidBody(
	_In_ const cv::Mat& refPoints,
	_In_ Marker& marker,
	_In_ float& max_deviation,
	_Out_ float& deviation,
	_Out_ cv::Mat& transform,
	_Out_ std::vector<int>& assignment)
{
	assignment = std::vector<int>(marker.numPoints, -1);
	deviation = 100;
	transform = cv::Mat::eye(4, 4, CV_32F);

	// first, check if we have previous information about world points that might fit.
	if (!marker.lastWorldPoints.empty())
	{
#if DBG_ENABLE_VERBOSE_LOGGING
		OutputDebugStringW(L"RigidBodyFitter::FitRigidBody: Trying to use previous information...\n");
#endif
		int correspondences = 0;
		for (int i = 0; i < marker.lastWorldPoints.cols; i++)
		{
			for (int j = 0; j < refPoints.rows; j++)
			{

				float diff = (float)norm(marker.lastWorldPoints.col(i) - refPoints.row(j).t());
				if (diff < 0.02)
				{
					assignment[i] = j;
					correspondences++;
				}
			}
		}
		cv::Mat worldPointsSorted = cv::Mat::zeros(3, correspondences, CV_32F);
		cv::Mat modelPointsSorted = cv::Mat::zeros(3, correspondences, CV_32F);
		for (int x = 0; x < assignment.size(); x++)
		{
			if (assignment[x] != -1)
			{
				worldPointsSorted.at<float>(0, x) = refPoints.at<float>(assignment[x], 0);
				worldPointsSorted.at<float>(1, x) = refPoints.at<float>(assignment[x], 1);
				worldPointsSorted.at<float>(2, x) = refPoints.at<float>(assignment[x], 2);
				modelPointsSorted.at<float>(0, x) = marker.markerPoints.at<float>(0, x);
				modelPointsSorted.at<float>(1, x) = marker.markerPoints.at<float>(1, x);
				modelPointsSorted.at<float>(2, x) = marker.markerPoints.at<float>(2, x);
			}
		}
		// if we have 4 or more correspondences, we can fit
		if (correspondences >= 4)
		{
#if DBG_ENABLE_VERBOSE_LOGGING
			OutputDebugStringW(L"RigidBodyFitter::FitRigidBody: Correspondences found. Fitting...\n");
#endif
			FitTwoPointSets(modelPointsSorted, worldPointsSorted, worldPointsSorted.cols, transform, deviation);
			if (deviation < max_deviation)
			{
#if DBG_ENABLE_VERBOSE_LOGGING
				OutputDebugStringW(L"RigidBodyFitter::FitRigidBody: Sucessful using old information.\n");
#endif
				return;
			}
		}
	}

#if DBG_ENABLE_VERBOSE_LOGGING
	OutputDebugStringW(L"RigidBodyFitter::FitRigidBody: No reliable old information. Recomputing...\n");
#endif
	Fit3DPointsToObjectTemplate(
		refPoints.t(), marker, transform, deviation, assignment);
}

// Note:
// adapted from https://github.com/gaschler/tiy
void RigidBodyFitter::Fit3DPointsToObjectTemplate(
	_In_ const cv::Mat& points_3D,
	_In_ Marker& marker,
	_Out_ cv::Mat& RT,
	_Out_ float& avg_deviation,
	_Out_ std::vector<int>& assignment)
{
	assignment = std::vector<int>(marker.numPoints, -1);

	// Constraint:
	const float max_distance = 0.01f;
	const int min_correspondences = 4;
	cv::Mat marker_template = marker.markerPoints;
	cv::Mat edges_template = marker.distanceMatrix;
	int num_temp = marker.numPoints;
	int num_p = points_3D.cols;

	// Constraint:
	float edges_template_max = marker.maxDistance + max_distance;

	// Constraint:
	float edges_template_min = marker.minDistance - max_distance;
	if (edges_template_min < 0.0)
		edges_template_min = 0.0f;


	// Find the best few edge matches and get edge lengths of ALL points (adjacency matrix)
	cv::Mat edges_world = cv::Mat::zeros(num_p, num_p, CV_32F);
	std::priority_queue<edge_match, std::vector<edge_match>, edge_match_comp> edge_matches;
	for (int a = 0; a < num_p; a++)
	{
		// Undirected graph => adjacency matrix symmetric => fill only upper triangular matrix
		for (int b = a + 1; b < num_p; b++)
		{
			edges_world.at<float>(a, b) = (float)norm(points_3D.col(a) - points_3D.col(b));
			if ((edges_world.at<float>(a, b) > edges_template_max) || (edges_world.at<float>(a, b) < edges_template_min))
				continue;

			for (int x = 0; x < num_temp; x++)
			{
				for (int y = x + 1; y < num_temp; y++)
				{
					float dist = abs(edges_world.at<float>(a, b) - edges_template.at<float>(x, y));
					// Constraint:
					if (dist < max_distance)
					{
						edge_matches.push(edge_match(dist, a, b, x, y));
					}
				}
			}
		}
	}


	// LIST of the best found template/world point indexes (i. column = values for i corresponding points)
	std::vector<std::vector<int> > best_world_idx;
	std::vector<std::vector<int> > best_template_idx;
	// Best ASSIGNMENT between template points <-> world points (i. column = values for i corresponding points)
	std::vector<std::vector<int> > best_template_to_world;
	std::vector<std::vector<int> > best_world_to_template;

	best_template_to_world.resize(num_temp);
	best_template_idx.resize(num_temp);
	best_world_to_template.resize(num_p);
	best_world_idx.resize(num_p);

	for (int i = 0; i < num_temp; i++)
	{
		best_template_to_world[i].resize(num_temp);
		best_template_idx[i].resize(num_temp);
	}
	for (int i = 0; i < num_p; i++)
	{
		best_world_to_template[i].resize(num_temp);
		best_world_idx[i].resize(num_temp);
	}

	for (int i = 0; i < num_temp; i++)
	{
		for (int j = 0; j < num_temp; j++)
		{
			best_template_to_world[i][j] = -1;
			best_template_idx[i][j] = -1;
		}
	}
	for (int i = 0; i < num_p; i++)
	{
		for (int j = 0; j < num_temp; j++)
		{
			best_world_to_template[i][j] = -1;
			best_world_idx[i][j] = -1;
		}
	}


	float my_residuum = std::numeric_limits<float>::infinity();
	std::vector<float> best_residuum, residuum_max;
	//std::cout << "Residuum max:" << std::endl;
	for (int i = 0; i < num_temp; i++)
	{
		best_residuum.push_back(std::numeric_limits<float>::infinity());
		// Constraint:
		residuum_max.push_back((i + 1) * max_distance);
	}

	// Constraint:
	int num_test_edges = 15 + num_temp * (num_temp - 1); // number of tested edges is 2*(number of edges in the object template)

	cv::Mat my_template = cv::Mat::zeros(3, num_temp, CV_32F), my_points = cv::Mat::zeros(3, num_temp, CV_32F);

	// 2 correspondences (a,b) <-> (x,y)
	for (int e = 0; e < (int)edge_matches.size() && e < num_test_edges; e++)
	{
		edge_match m = edge_matches.top();
		int a = m.a, b = m.b, x = m.x, y = m.y;

		// 3 correspondences (a,b,c) <-> (x,y,z)
		for (int c = 0; c < num_p; c++)
		{
			if (a == c || b == c)
				continue;

			float edge_a_b = 0.0;
			float edge_a_c = 0.0;
			float edge_b_c = 0.0;

			// Lower triangle matrix not filled (symmetric) -> take correspondent from upper matrix
			if (a > b)
				edge_a_b = edges_world.at<float>(b, a);
			else
				edge_a_b = edges_world.at<float>(a, b);

			if (a > c)
				edge_a_c = edges_world.at<float>(c, a);
			else
				edge_a_c = edges_world.at<float>(a, c);

			if (b > c)
				edge_b_c = edges_world.at<float>(c, b);
			else
				edge_b_c = edges_world.at<float>(b, c);

			// Test if edges of new point in cv::Range
			if (edge_a_c > edges_template_max || edge_a_c < edges_template_min || edge_b_c > edges_template_max || edge_b_c < edges_template_min)
				continue;

			for (int z = 0; z < num_temp; z++)
			{
				int my_num_corres = 3;

				float dist_a_b = edge_a_b - edges_template.at<float>(x, y);
				float dist_a_c = edge_a_c - edges_template.at<float>(x, z);
				float dist_b_c = edge_b_c - edges_template.at<float>(y, z);

				my_residuum = dist_a_b * dist_a_b + dist_a_c * dist_a_c + dist_b_c * dist_b_c;

				// Test if triangle does match AND is better than best fit so far
				if (x == z || y == z || my_residuum > residuum_max[my_num_corres - 1] || my_residuum > best_residuum[my_num_corres - 1])
					continue;

				best_residuum[my_num_corres - 1] = my_residuum;


				for (int i = 0; i < num_temp; i++)
				{
					best_template_idx[i][my_num_corres - 1] = -1;
					best_template_to_world[i][my_num_corres - 1] = -1;
				}
				for (int i = 0; i < num_p; i++)
				{
					best_world_idx[i][my_num_corres - 1] = -1;
					best_world_to_template[i][my_num_corres - 1] = -1;
				}

				best_template_idx[0][my_num_corres - 1] = x;
				best_template_idx[1][my_num_corres - 1] = y;
				best_template_idx[2][my_num_corres - 1] = z;
				best_world_idx[0][my_num_corres - 1] = a;
				best_world_idx[1][my_num_corres - 1] = b;
				best_world_idx[2][my_num_corres - 1] = c;

				best_template_to_world[x][my_num_corres - 1] = a;
				best_template_to_world[y][my_num_corres - 1] = b;
				best_template_to_world[z][my_num_corres - 1] = c;
				best_world_to_template[a][my_num_corres - 1] = x;
				best_world_to_template[b][my_num_corres - 1] = y;
				best_world_to_template[c][my_num_corres - 1] = z;

				// 4+ correspondences (a,b,c,...) ~ (x,y,z,...)
				while (my_num_corres < num_temp)
				{
					// Find closest point
					int best_world_idx_local = -1, best_template_idx_local = -1;
					float new_residuum = std::numeric_limits<float>::infinity(); // only additional terms for residuum
					float best_new_residuum = std::numeric_limits<float>::infinity();

					// Go through ALL template points (that are NOT assigned yet) -> take template point with smallest residuum
					for (int i = 0; i < num_temp; i++)
					{
						// Test if already assigned
						if (best_template_to_world[i][my_num_corres - 1] >= 0)
							continue;

						// Go through ALL world points (that are NOT assigned yet) -> take world correspondent with smallest residuum
						for (int j = 0; j < num_p; j++)
						{
							// Test if already assigned
							if (best_world_to_template[j][my_num_corres - 1] >= 0)
								continue;

							new_residuum = 0.0;

							// Go through ALL correspondences (world points, that ARE assigned yet)
							// -> check if edges (corresp <-> ACTUAL world point) are in range and best residuum so far
							for (int k = 0; k < my_num_corres; k++)
							{
								float new_edge_world = 0.0;

								// only upper triangle matrix filled
								if (j > best_world_idx[k][my_num_corres - 1])
									new_edge_world = edges_world.at<float>(best_world_idx[k][my_num_corres - 1], j);
								else
									new_edge_world = edges_world.at<float>(j, best_world_idx[k][my_num_corres - 1]);

								// edge in [min...max] range?
								if ((new_edge_world > edges_template_max) || (new_edge_world < edges_template_min))
								{
									new_residuum = std::numeric_limits<float>::infinity();
									break; // not in range => world point definitely NOT a correspondant => break
								}

								// Test if the edge from the actual (j.) world candidate to the other (k.) correspondants fit to
								// the edges from the assigned object template points to the "next" template point
								float new_edge_template = edges_template.at<float>(i, best_template_idx[k][my_num_corres - 1]);
								float new_dist = abs(new_edge_world - new_edge_template);
								// Constraint:
								if (new_dist > max_distance + 0.05)
								{
									new_residuum = std::numeric_limits<float>::infinity();
									break; // distance too big => world point definitely NOT a correspondant => break
								}

								new_residuum += new_dist * new_dist;
							}

							if (new_residuum > best_new_residuum)
								continue;

							best_new_residuum = new_residuum;
							best_template_idx_local = i;
							best_world_idx_local = j;
						}
					}

					my_residuum = best_residuum[my_num_corres - 1] + best_new_residuum;

					if ((my_residuum > residuum_max[my_num_corres]) || (my_residuum > best_residuum[my_num_corres]))
						break;

					best_residuum[my_num_corres] = my_residuum;

					for (int i = 0; i < my_num_corres; i++)
					{
						best_template_idx[i][my_num_corres] = best_template_idx[i][my_num_corres - 1];
						best_world_idx[i][my_num_corres] = best_world_idx[i][my_num_corres - 1];
					}
					best_template_idx[my_num_corres][my_num_corres] = best_template_idx_local;
					best_world_idx[my_num_corres][my_num_corres] = best_world_idx_local;


					for (int i = 0; i < num_temp; i++)
						best_template_to_world[i][my_num_corres] = best_template_to_world[i][my_num_corres - 1];
					for (int i = 0; i < num_p; i++)
						best_world_to_template[i][my_num_corres] = best_world_to_template[i][my_num_corres - 1];
					best_template_to_world[best_template_idx_local][my_num_corres] = best_world_idx_local;
					best_world_to_template[best_world_idx_local][my_num_corres] = best_template_idx_local;

					my_num_corres++;
				}

			}
		}
		edge_matches.pop();
	}

	if (best_residuum[min_correspondences - 1] == std::numeric_limits<float>::infinity())
	{
#if DBG_ENABLE_ERROR_LOGGING
		wchar_t msgBuffer[200];
		swprintf_s(msgBuffer, L"RigidBodyFitter::FitArbritrary3DPointsToObjectTemplate: Not enough correspondences found - num_temp = %i.\n",
			num_temp);
		OutputDebugStringW(msgBuffer);
#endif // DBG_ENABLE_ERROR_LOGGING
		RT = cv::Mat::zeros(4, 4, CV_32F);
		avg_deviation = std::numeric_limits<float>::infinity();
		return;
	}

	// Decide which result with which number of correspondances to take
	// (the smaller the residuum the better but also the more correspondants the better)
	unsigned int best_num_corres = min_correspondences;

	int num_edges = 0;
	std::vector<float>avg_edge_residuum;
	for (int i = 0; i < num_temp; i++)
		avg_edge_residuum.push_back(0);
	avg_edge_residuum[0] = std::numeric_limits<float>::infinity();

	for (int i = 1; i < num_temp; i++)
	{
		// Compute average edge residuum by dividing the residuum by the number of quadratic terms (= number of edges)
		num_edges += i;
		avg_edge_residuum[i] = best_residuum[i] / num_edges;

		float factor_ = 1.0;
		// Always better to have more points, when the avg_edg_residuums are nearly the same
		for (int j = i + 1; j < num_temp; j++)
			factor_ = factor_ * 1.65f;
		avg_edge_residuum[i] = factor_ * avg_edge_residuum[i];
	}

	for (int i = 4; i < num_temp; i++)
	{
		if (avg_edge_residuum[i] <= avg_edge_residuum[best_num_corres - 1])
			best_num_corres = i + 1;
	}

	//TODO: simplify this
	for (unsigned int i = 0; i < best_num_corres; i++)
	{
		assignment[best_template_idx[i][best_num_corres - 1]] = best_world_idx[i][best_num_corres - 1];
		my_points.at<float>(0, i) = points_3D.at<float>(0, best_world_idx[i][best_num_corres - 1]);
		my_points.at<float>(1, i) = points_3D.at<float>(1, best_world_idx[i][best_num_corres - 1]);
		my_points.at<float>(2, i) = points_3D.at<float>(2, best_world_idx[i][best_num_corres - 1]);
		my_template.at<float>(0, i) = marker_template.at<float>(0, best_template_idx[i][best_num_corres - 1]);
		my_template.at<float>(1, i) = marker_template.at<float>(1, best_template_idx[i][best_num_corres - 1]);
		my_template.at<float>(2, i) = marker_template.at<float>(2, best_template_idx[i][best_num_corres - 1]);
	}

	FitTwoPointSets(my_template.colRange(0, best_num_corres), my_points.colRange(0, best_num_corres), best_num_corres, RT, avg_deviation);

	if (avg_deviation < 0.01)
	{
		cv::Mat worldPointsSorted = cv::Mat::zeros(3, best_num_corres, CV_32F);
		for (unsigned int i = 0; i < best_num_corres; i++)
		{
			int template_id = best_template_idx[i][best_num_corres - 1];
			int world_id = best_world_idx[i][best_num_corres - 1];
			worldPointsSorted.at<float>(0, template_id) = points_3D.at<float>(0, world_id);
			worldPointsSorted.at<float>(1, template_id) = points_3D.at<float>(1, world_id);
			worldPointsSorted.at<float>(2, template_id) = points_3D.at<float>(2, world_id);
		}
	}
}

// Note: adapted from https://github.com/gaschler/tiy
// Minimizes point_set_1 - RT*point_set_0 in the least-squares sense. Fast implementation.
//
// Arun, Huang & Blostein 1987: Least-Squares Fittig of Two 3-D Point Sets
// see http://www.math.ltu.se/courses/c0002m/least_squares.pdf page 11
// http://portal.acm.org/citation.cfm?id=28821
// http://portal.acm.org/citation.cfm?id=105525
//
// Andre Gaschler, 2010
void RigidBodyFitter::FitTwoPointSets(
	_In_ const cv::Mat& point_set_0,
	_In_ const cv::Mat& point_set_1,
	_In_ int num_points,
	_Out_ cv::Mat& RT,
	_Out_ float& avg_deviation)
{
	assert(num_points <= point_set_0.cols && num_points <= point_set_1.cols);

	cv::Scalar centroid[2][3];
	cv::Mat point_set_0_c(3, num_points, CV_32F), point_set_1_c(3, num_points, CV_32F), C(num_points, num_points, CV_32F), t(3, 1, CV_32F);

	for (int j = 0; j < 3; j++)
	{
		centroid[0][j] = mean(point_set_0.row(j));
		centroid[1][j] = mean(point_set_1.row(j));
		point_set_0_c.row(j) = point_set_0.row(j) - centroid[0][j][0];
		point_set_1_c.row(j) = point_set_1.row(j) - centroid[1][j][0];
	}

	C = point_set_1_c * point_set_0_c.t();
	cv::SVD C_svd(C);

	//det(U*V') Umeyama correction
	float det_U_Vt = (float)cv::determinant(C_svd.u * C_svd.vt);
	cv::Mat Det_U_Vt = cv::Mat::eye(3, 3, CV_32F);
	Det_U_Vt.at<float>(2, 2) = det_U_Vt;

	RT = cv::Mat::eye(4, 4, CV_32F);

	cv::Mat R = RT(cv::Range(0, 3), cv::Range(0, 3));
	R = C_svd.u * Det_U_Vt * C_svd.vt;

	cv::Mat point_set_0_centroid(3, 1, CV_32F), point_set_1_centroid(3, 1, CV_32F);
	for (int j = 0; j < 3; j++)
	{
		point_set_0_centroid.at<float>(j, 0) = (float)centroid[0][j][0];
		point_set_1_centroid.at<float>(j, 0) = (float)centroid[1][j][0];
	}

	t = RT(cv::Range(0, 3), cv::Range(3, 4));
	t = point_set_1_centroid - (R * point_set_0_centroid);

	// calculate average deviation
	cv::Mat point_set_1_t(3, num_points, CV_32F), dev(3, num_points, CV_32F);
	point_set_1_t.row(0) = point_set_1.row(0) - t.at<float>(0, 0);
	point_set_1_t.row(1) = point_set_1.row(1) - t.at<float>(1, 0);
	point_set_1_t.row(2) = point_set_1.row(2) - t.at<float>(2, 0);

	dev = point_set_1_t - (R * point_set_0);
	float dev_point, dev_sum = 0;
	for (int i = 0; i < num_points; i++)
	{
		dev_point = (float)norm(dev.col(i));
		dev_sum += dev_point;
	}

	avg_deviation = dev_sum / num_points;
}

