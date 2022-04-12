#pragma once
// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
// adapted from:
// https://github.com/Smorodov/Multitarget-tracker/blob/master/src/Tracker/HungarianAlg/HungarianAlg.h

///
/// \brief The AssignmentProblemSolver class
///
class AssignmentProblemSolver
{
private:
	// Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
	void assignmentoptimal(
		std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns);
	void buildassignmentvector(
		std::vector<int>& assignment, bool* starMatrix, size_t nOfRows, size_t nOfColumns);
	void computeassignmentcost(
		const std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, size_t nOfRows);
	void step2a(
		std::vector<int>& assignment, float* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, 
		bool* coveredColumns, bool* coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step2b(
		std::vector<int>& assignment, float* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, 
		bool* coveredColumns, bool* coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step3_5(
		std::vector<int>& assignment, float* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, 
		bool* coveredColumns, bool* coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
	void step4(
		std::vector<int>& assignment, float* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix, 
		bool* coveredColumns, bool* coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col);

	// Computes a suboptimal solution. Good for cases with many forbidden assignments.
	void assignmentsuboptimal1(std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, 
		size_t nOfRows, size_t nOfColumns);
	// Computes a suboptimal solution. Good for cases with many forbidden assignments.
	void assignmentsuboptimal2(std::vector<int>& assignment, float& cost, const std::vector<float>& distMatrixIn, 
		size_t nOfRows, size_t nOfColumns);

public:
	enum TMethod
	{
		optimal,
		many_forbidden_assignments,
		without_forbidden_assignments
	};

	AssignmentProblemSolver() = default;
	~AssignmentProblemSolver() = default;
	float Solve(
		const std::vector<float>& distMatrixIn, size_t nOfRows, size_t nOfColumns, std::vector<int>& assignment, TMethod Method = optimal);
};

